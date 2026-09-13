from typing import Callable
from commands2 import Subsystem, Command
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.filter import SlewRateLimiter
from lib import logger, utils
from lib.classes import DifferentialModuleLocation
from lib.components.differential_module import DifferentialModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Drive

    self._modules = tuple(DifferentialModule(c) for c in self._constants.DIFFERENTIAL_MODULE_CONFIGS)

    self._drivetrain = DifferentialDrive(
      self._modules[DifferentialModuleLocation.Left].getMotorController(),
      self._modules[DifferentialModuleLocation.Right].getMotorController()
    )

    self._drivetrain.setExpiration(0.1)

    self._translationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._rotationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getTranslationInput: Callable[[], units.percent], getRotationInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._runDrive(getTranslationInput(), getRotationInput())
    ).withName("Drive:Drive")

  def _runDrive(self, translationInput: units.percent, rotationInput: units.percent) -> None:
    translationInput = self._translationInputLimiter.calculate(translationInput * self._constants.INPUT_LIMIT_DEMO) if translationInput != 0 else 0
    rotationInput = self._rotationInputLimiter.calculate(rotationInput * self._constants.INPUT_LIMIT_DEMO) if rotationInput != 0 else 0

    self._drivetrain.arcadeDrive(translationInput, rotationInput, False)

  def reset(self) -> None:
    self._drivetrain.arcadeDrive(0, 0)
  
  def _updateTelemetry(self) -> None:
    pass
