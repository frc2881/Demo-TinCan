from wpimath import units
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import (
  RobotType, 
  Range,
  DifferentialModuleConstants, 
  DifferentialModuleConfig, 
  DifferentialModuleLocation,
  LimitPositionControlModuleConfig,
  LimitPositionControlModuleConstants,
  SpeedModuleConfig,
  SpeedModuleConstants,
  FollowerModuleConfig,
  FollowerModuleConstants
)
import lib.constants

class Subsystems:
  class Drive:
    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(4.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 50,
      drivingMotorReduction = 8.46
    )

    DIFFERENTIAL_MODULE_CONFIGS: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 2, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 4, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 3, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 5, 4, False, _differentialModuleConstants)
    )

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Arm:
    ARM_LEADER_CONFIG = LimitPositionControlModuleConfig("Arm/Leader", 10, True, LimitPositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = 80,
      motorOutputRange = Range(-0.5, 0.5)
    ))

    ARM_FOLLOWER_CONFIG = FollowerModuleConfig("Arm/Follower", 11, 10, False, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = ARM_LEADER_CONFIG.constants.motorCurrentLimit
    ))

  class Gripper:
    GRIPPER_LEADER_CONFIG = SpeedModuleConfig("Gripper/Leader", 12, False, SpeedModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = 20
    ))

    GRIPPER_FOLLOWER_CONFIG = FollowerModuleConfig("Gripper/Follower", 13, 12, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = GRIPPER_LEADER_CONFIG.constants.motorCurrentLimit
    ))

    GRIPPER_INTAKE_SPEED: units.percent = 1.0
    GRIPPER_SCORE_SPEED: units.percent = 1.0

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Demo
    NAME: str = "TinCan (Demo)"

  class Commands:
    pass
