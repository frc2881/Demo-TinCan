from wpimath import units
from rev import SparkLowLevel
from lib import logger, telemetry, utils
from lib.classes import (
  RobotType, 
  Range, 
  DifferentialDriveModuleConfig,
  DifferentialDriveModuleConfigConstants,
  DifferentialDriveModuleLocation,
  LimitPositionControlModuleConfig,
  SpeedControlModuleConfig,
  FollowerControlModuleConfig,
  XboxControllerConfig
)
import lib.constants

class Subsystems:
  class Drive:
    _differentialModuleConstants = DifferentialDriveModuleConfigConstants(
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      gearReduction = 8.46,
      currentLimit = 50,
      wheelDiameter = units.inchesToMeters(4.0),
      telemetryName = "Robot/Subsystems/Drive/Modules"
    )

    DIFFERENTIAL_MODULE_CONFIGS: tuple[DifferentialDriveModuleConfig, ...] = (
      DifferentialDriveModuleConfig(DifferentialDriveModuleLocation.Left, 2, None, True, _differentialModuleConstants),
      DifferentialDriveModuleConfig(DifferentialDriveModuleLocation.Right, 4, None, False, _differentialModuleConstants),
      DifferentialDriveModuleConfig(DifferentialDriveModuleLocation.Left, 3, 2, True, _differentialModuleConstants),
      DifferentialDriveModuleConfig(DifferentialDriveModuleLocation.Right, 5, 4, False, _differentialModuleConstants)
    )

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Arm:
    ARM_LEADER_CONFIG = LimitPositionControlModuleConfig(
      id = 10,
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      currentLimit = 80,
      isInverted = True,
      outputRange = Range(-0.5, 0.5),
      telemetryName = "Robot/Subsystems/Arm/Leader"
    )

    ARM_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 11,
      leaderId = 10,
      controllerType = ARM_LEADER_CONFIG.controllerType,
      motorType = ARM_LEADER_CONFIG.motorType,
      currentLimit = ARM_LEADER_CONFIG.currentLimit,
      isInverted = False,
      telemetryName = "Robot/Subsystems/Arm/Follower"
    )

  class Gripper:
    GRIPPER_LEADER_CONFIG = SpeedControlModuleConfig(
      id = 12,
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      currentLimit = 20,
      isInverted = False,
      telemetryName = "Robot/Subsystems/Gripper/Leader"
    )

    GRIPPER_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 13,
      leaderId = 12,
      controllerType = GRIPPER_LEADER_CONFIG.controllerType,
      motorType = GRIPPER_LEADER_CONFIG.motorType,
      currentLimit = GRIPPER_LEADER_CONFIG.currentLimit,
      isInverted = True,
      telemetryName = "Robot/Subsystems/Gripper/Follower"
    )

    GRIPPER_INTAKE_SPEED: units.percent = 1.0
    GRIPPER_SCORE_SPEED: units.percent = 1.0

class Controllers:
  DRIVER_CONTROLLER_CONFIG = XboxControllerConfig(port = 0, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Driver")
  # OPERATOR_CONTROLLER_CONFIG = XboxControllerConfig(port = 1, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Operator")
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Demo
    NAME: str = "TinCan (Demo)"

  class Commands:
    pass
