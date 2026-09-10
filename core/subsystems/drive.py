from typing import Callable, Optional
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import MotorIdleMode, SpeedMode, DifferentialModuleLocation, DifferentialModulePositions
from lib.components.differential_module import DifferentialModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._modules = tuple(DifferentialModule(c) for c in self._constants.DIFFERENTIAL_MODULE_CONFIGS)

    self._drivetrain = DifferentialDrive(
      self._modules[DifferentialModuleLocation.Left].getMotorController(),
      self._modules[DifferentialModuleLocation.Right].getMotorController()
    )

    self._translationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._rotationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedMode = SendableChooser()
    speedMode.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedMode.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedMode.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedMode)

    idleMode = SendableChooser()
    idleMode.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleMode.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleMode.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleMode)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getTranslationInput: Callable[[], units.percent], getRotationInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._runDrive(getTranslationInput(), getRotationInput())
    ).withName("Drive:Drive")

  def _runDrive(self, translationInput: units.percent, rotationInput: units.percent) -> None:
    if self._speedMode == SpeedMode.Demo:
      translationInput = self._translationInputLimiter.calculate(translationInput * self._constants.INPUT_LIMIT_DEMO) if translationInput != 0 else 0
      rotationInput = self._rotationInputLimiter.calculate(rotationInput * self._constants.INPUT_LIMIT_DEMO) if rotationInput != 0 else 0

    self._drivetrain.arcadeDrive(translationInput, rotationInput, False)

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: Optional[DriveFeedforwards] = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.DRIVE_KINEMATICS.toChassisSpeeds(self._getModuleStates())

  def getModulePositions(self) -> DifferentialModulePositions:
    return DifferentialModulePositions(
      self._modules[DifferentialModuleLocation.Left].getPosition(),
      self._modules[DifferentialModuleLocation.Right].getPosition()
    )

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    wheelSpeeds = self._constants.DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds)
    self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)

  def _getModuleStates(self) -> DifferentialDriveWheelSpeeds:
    return DifferentialDriveWheelSpeeds(
      self._modules[DifferentialModuleLocation.Left].getVelocity(), 
      self._modules[DifferentialModuleLocation.Right].getVelocity()
    )

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for m in self._modules: m.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
  
  def _updateTelemetry(self) -> None:
    pass
