import wpilib
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import DifferentialDriveKinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import (
  Alliance, 
  PID,
  Range, 
  Tolerance,
  DifferentialModuleConstants, 
  DifferentialModuleConfig, 
  DifferentialModuleLocation,
  DriftCorrectionConstants, 
  TargetAlignmentConstants,
  PoseSensorConstants,
  PoseSensorConfig,
  RelativePositionControlModuleConfig,
  RelativePositionControlModuleConstants
)
from core.classes import (
  Target, 
  TargetType
)

_APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/default.json')
_PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    CHASSIS_LENGTH: units.meters = units.inchesToMeters(38.0)
    CHASSIS_WIDTH: units.meters = units.inchesToMeters(25.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(27.0)
    TRACK_WIDTH: units.meters = units.inchesToMeters(17.0)

    TRANSLATION_SPEED_MAX: units.meters_per_second = 4.46
    ROTATION_SPEED_MAX: units.degrees_per_second = 360.0

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.33

    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(4.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 80,
      drivingMotorReduction = 8.46
    )

    DIFFERENTIAL_MODULE_CONFIGS: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 2, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 4, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 3, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 5, 4, False, _differentialModuleConstants)
    )

    DRIVE_KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH)

    PATHPLANNER_ROBOT_CONFIG = _PATHPLANNER_ROBOT_CONFIG
    PATHPLANNER_CONTROLLER = PPLTVController(0.02)

    DRIFT_CORRECTION_CONSTANTS = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    TARGET_ALIGNMENT_CONSTANTS = TargetAlignmentConstants(
      translationPID = PID(4.0, 0, 0),
      translationMaxVelocity = 2.0,
      translationMaxAcceleration = 1.0,
      translationTolerance = Tolerance(0.05, 0.1),
      rotationPID = PID(4.0, 0, 0), 
      rotationMaxVelocity = 360.0,
      rotationMaxAcceleration = 180.0,
      rotationTolerance = Tolerance(0.5, 1.0),
      rotationHeadingModeOffset = 0,
      rotationTranslationModeOffset = 180.0
    )

  class Arm:
    ARM_CONFIG = RelativePositionControlModuleConfig("Arm", 10, True, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushed,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.07),
      motorOutputRange = Range(-1.0, 0.3),
      motorMotionMaxVelocity = 15000.0,
      motorMotionMaxAcceleration = 30000.0,
      motorMotionAllowedClosedLoopError = 0.25,
      motorSoftLimitForward = 35.0,
      motorSoftLimitReverse = 1.0,
      motorResetSpeed = 0.4,
      distancePerRotation = 1.0
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
    VISION_MAX_TARGET_DISTANCE: units.meters = 4.0
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_GROUND_PLANE_DELTA: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kMXP_SPI

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = _APRILTAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   name = "Front",
      #   transform = Transform3d(Translation3d(0.149506, -0.055318, 0.271137), Rotation3d(-0.001852, -0.181301, 0.020370)),
      #   stream = "http://10.28.81.6:1182/?action=stream", 
      #   constants = _poseSensorConstants
      # ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game: 
  class Commands:
    pass

  class Field:
    APRILTAG_FIELD_LAYOUT = _APRILTAG_FIELD_LAYOUT
    LENGTH = _APRILTAG_FIELD_LAYOUT.getFieldLength()
    WIDTH = _APRILTAG_FIELD_LAYOUT.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.Default, _APRILTAG_FIELD_LAYOUT.getTagPose(1))
        },
        Alliance.Blue: {
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.Default, _APRILTAG_FIELD_LAYOUT.getTagPose(2))
        }
      }
