from commands2 import Subsystem, cmd, Command
from rev import SparkBase, SparkBaseConfig, SparkMax, SparkMaxConfig
import core.constants as constants
from lib import utils  

class Gripper(Subsystem):
  def __init__(self):
    super().__init__()
    self._config = constants.Subsystems.Gripper
    self._frontMotor = SparkMax(self._config.FRONT_MOTOR_CAN_ID, self._config.MOTOR_TYPE)
    self._backMotor = SparkMax(self._config.BACK_MOTOR_CAN_ID, self._config.MOTOR_TYPE)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.MOTOR_CURRENT_LIMIT)
      .inverted(False))
    utils.setSparkConfig(
      self._frontMotor.configure(self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters))
    utils.setSparkConfig(
      self._backMotor.configure(self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters))

  def intake(self) -> Command:
    return self.run(lambda: [
      self._frontMotor.set(-self._config.MOTOR_SPEED),
      self._backMotor.set(self._config.MOTOR_SPEED)
    ])
  
  def score(self) -> Command:
    return self.run(lambda: [
      self._frontMotor.set(self._config.MOTOR_SPEED),
      self._backMotor.set(-self._config.MOTOR_SPEED)
    ])

  def stop(self) -> Command:
    return self.run(lambda: [
      self._frontMotor.stopMotor(),
      self._backMotor.stopMotor()
    ])