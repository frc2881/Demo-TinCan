import wpilib
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import DifferentialDriveKinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from lib import logger, utils
from lib.classes import (
  RobotType, 
  Alliance,
  PID,
  Zone,
  Range,
  MotorModel,
  FeedForwardGains,
  DifferentialModuleConstants, 
  DifferentialModuleConfig, 
  DifferentialModuleLocation,
  PoseSensorConfig, 
  RelativePositionControlModuleConfig,
  RelativePositionControlModuleConstants
)
from core.classes import Target
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/default.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(38.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(25.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(27.0)
    TRACK_WIDTH: units.meters = units.inchesToMeters(17.0)

    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(4.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 60,
      drivingMotorReduction = 8.46
    )

    DIFFERENTIAL_MODULE_CONFIGS: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 2, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 4, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 3, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 5, 4, False, _differentialModuleConstants)
    )

    DRIVE_KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH)

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = 4.46
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 360.0

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPLTVController(0.02)

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Arm:
    ARM_CONFIG = RelativePositionControlModuleConfig("Arm", 10, True, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = 80,
      motorRelativeEncoderPositionConversionFactor = 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-1.0, 0.3),
      motorFeedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEO]),
      motorMotionCruiseVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionAllowedProfileError = 0.25,
      motorSoftLimitForward = 35.0,
      motorSoftLimitReverse = 1.0,
      motorHomingSpeed = 0.4,
      motorHomedPosition = 0
    ))

    INPUT_LIMIT: units.percent = 1.0

  class Gripper:
    FRONT_MOTOR_CAN_ID = 12
    BACK_MOTOR_CAN_ID = 13
    MOTOR_TYPE = SparkLowLevel.MotorType.kBrushed 
    MOTOR_SPEED: units.percent = 1.0
    MOTOR_CURRENT_LIMIT = 20

class Services:
  class Localization:
    MAX_TARGET_AMBIGUITY: units.percent = 0.2
    MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    MAX_TARGET_DISTANCE: units.meters = 5.0
    MAX_POSE_CHANGE: units.meters = 1.0
    STDDEV_XY_COEFF: float = 0.08
    STDDEV_Z_COEFF: float = 0.1
    STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 5.0
    STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5
    VALID_POSE_SENSOR_RESULT_TIMEOUT: units.seconds = 0.3
  
  class Targeting:
    pass

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kMXP_SPI

  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   name = "Front",
      #   transform = Transform3d(
      #     Translation3d(x = units.inchesToMeters(4.25), y = units.inchesToMeters(-1.77), z = units.inchesToMeters(9.47)), 
      #     Rotation3d(roll = units.degreesToRadians(-0.18), pitch = units.degreesToRadians(-32.77), yaw = units.degreesToRadians(-0.18))
      #   ),
      #   stream = "http://10.28.81.6:1182/?action=stream", 
      #   aprilTagFieldLayout = _aprilTagFieldLayout
      # ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Demo
    NAME: str = "TinCan"

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    ZONE = Zone(start = Translation2d(0, 0), end = Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Blue: {
          Target.Default: Pose3d(0, 0, 0, Rotation3d(Rotation2d.fromDegrees(0)))
        },
        Alliance.Red: {
          Target.Default: Pose3d(0, 0, 0, Rotation3d(Rotation2d.fromDegrees(0)))
        }
      }

      TARGET_ZONES: dict[Alliance, dict[Target, Zone]] = {
        Alliance.Blue: {},
        Alliance.Red: {}
      }
