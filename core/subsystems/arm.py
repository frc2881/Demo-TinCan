from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import State, MotorIdleMode, SpeedMode, DriveOrientation, TargetAlignmentMode, DifferentialModuleLocation, DifferentialDriveModulePositions
from lib.components.differential_module import DifferentialModule
import core.constants as constants

import math
from commands2 import Command, cmd, Subsystem
from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, LimitSwitchConfig
from lib.classes import PositionControlModuleConfig, MotorDirection, Value
from lib import logger, utils

class LimitPositionControlModule:
  def __init__(
    self,
    config: PositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    self._hasZeroReset: bool = False
    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .inverted(self._config.isInverted))
    if self._config.leaderMotorCANId is not None:
      self._motorConfig.follow(self._config.leaderMotorCANId, self._config.isInverted)
    else:
      self._motorConfig.apply(
        LimitSwitchConfig()
        .forwardLimitSwitchEnabled(True)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen))

      (self._motorConfig.softLimit
        .reverseSoftLimitEnabled(True)
        .reverseSoftLimit(self._config.constants.motorSoftLimitReverse)
        .forwardSoftLimitEnabled(True)
        .forwardSoftLimit(self._config.constants.motorSoftLimitForward))
    utils.setSparkConfig(
      self._motor.configure(
        self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._forwardLimit = self._motor.getForwardLimitSwitch()
    self._reverseLimit = self._motor.getReverseLimitSwitch()
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._motor.set(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetPosition()
    
  def setPosition(self, position: float) -> None:
    if position < 0:
       position = -1.0
    elif position > 0:
       position = 1.0
    else:
      position = 0.0

    self._targetPosition = position
    self._motor.set(0.2 * position)

    self._isAtTargetPosition = math.isclose(self.getPosition(), self._targetPosition, abs_tol = self._config.constants.motorMotionAllowedClosedLoopError)

  def getPosition(self) -> float:
    if self._forwardLimit.get():
      return 1.0
    if self._reverseLimit.get():
      return -1.0
    return 0.0

  def _resetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSparkSoftLimitsEnabled(self._motor, isEnabled)
  
  def hasZeroReset(self) -> bool:
    return self._hasZeroReset

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetPosition()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsAtTargetPosition', self._isAtTargetPosition)
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
    SmartDashboard.putNumber(f'{self._baseKey}/Position', self.getPosition())


class Arm(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Arm
    self._config = self._constants.kArmConfig

    self._hasInitialZeroReset: bool = False

    self._arm = LimitPositionControlModule(self._config)

    self._motor = SparkMax(11, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .inverted(self._config.isInverted))
    if self._config.leaderMotorCANId is not None:
      self._motorConfig.follow(10, True)


  def periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._arm.setSpeed(getInput() * self._constants.kInputLimit),
      lambda: self.reset()
    ).withName("Arm:SetSpeed")
  
  def setPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._arm.setPosition(position)
    ).withName("Arm:SetPosition")
  
  def getPosition(self) -> units.inches:
    return self._arm.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._arm.isAtTargetPosition()

  def resetToZero(self) -> Command:
    return self._arm.resetToZero(self).withName("Arm:ResetToZero")

  def hasZeroReset(self) -> bool:
    return self._arm.hasZeroReset()

  def reset(self) -> None:
    self._arm.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Arm/IsAtTargetPosition", self.isAtTargetPosition())

    
    