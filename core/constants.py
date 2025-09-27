from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import DifferentialDriveKinematics
import wpilib
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from photonlibpy.photonPoseEstimator import PoseStrategy
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import (
  Alliance, 
  PID, 
  Tolerance,
  DifferentialModuleConstants, 
  DifferentialModuleConfig, 
  DifferentialModuleLocation,
  DriftCorrectionConstants, 
  TargetAlignmentConstants,
  PoseSensorConstants,
  PoseSensorConfig
)
from core.classes import Target, TargetType

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/default.json')
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kRobotWidth: units.meters = units.inchesToMeters(25.5)
    kRobotLength: units.meters = units.inchesToMeters(38.0)
    kTrackWidth: units.meters = units.inchesToMeters(17.0)
    kWheelBase: units.meters = units.inchesToMeters(27.0)

    kTranslationSpeedMax: units.meters_per_second = 4.46
    kRotationSpeedMax: units.degrees_per_second = 360.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(4.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 80,
      drivingMotorReduction = 8.46
    )

    kDifferentialModuleConfigs: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 2, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 4, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Left, 3, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.Right, 5, 4, False, _differentialModuleConstants)
    )

    kDriveKinematics = DifferentialDriveKinematics(kTrackWidth)

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPLTVController(0.02)

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
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

class Services:
  class Localization:
    kVisionMaxTargetDistance: units.meters = 4.0
    kVisionMaxPoseAmbiguity: units.percent = 0.2
    kRobotPoseMaxGroundPlaneDelta: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kMXP_SPI

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   "Front",
      #   Transform3d(
      #     Translation3d(units.inchesToMeters(9.0), units.inchesToMeters(1.5), units.inchesToMeters(18.5)),
      #     Rotation3d(units.degreesToRadians(0), units.degreesToRadians(11.0), units.degreesToRadians(0))
      #   ), _poseSensorConstants
      # )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Driver": "http://10.28.81.6:1184/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game: 
  class Commands:
    pass

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.Default, APRIL_TAG_FIELD_LAYOUT.getTagPose(1))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.Default, APRIL_TAG_FIELD_LAYOUT.getTagPose(2))
        }
      }
