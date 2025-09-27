from typing import Callable
import math
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform3d
from wpimath.estimator import DifferentialDrivePoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import DifferentialDriveModulePositions
from lib.sensors.pose import PoseSensor
from core.classes import Target
import core.constants as constants

class Localization():
  def __init__(
      self,
      getGyroRotation: Callable[[], Rotation2d],
      getDriveModulePositions: Callable[[], DifferentialDriveModulePositions],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
    super().__init__()
    self._getGyroRotation = getGyroRotation
    self._getDriveModulePositions = getDriveModulePositions
    self._poseSensors = poseSensors

    self._poseEstimator = DifferentialDrivePoseEstimator(
      constants.Subsystems.Drive.kDriveKinematics,
      self._getGyroRotation(),
      *self._getDriveModulePositions(),
      Pose2d()
    )

    self._alliance = None
    self._robotPose = Pose2d()
    self._targets: dict[int, Target] = {}
    self._targetPoses: list[Pose2d] = []
    self._hasValidVisionTarget: bool = False
    self._validVisionTargetBufferTimer = Timer()
    
    self._robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d).publish()
    SmartDashboard.putNumber("Game/Field/Length", constants.Game.Field.kLength)
    SmartDashboard.putNumber("Game/Field/Width", constants.Game.Field.kWidth)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateRobotPose()
    self._updateTargets()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), *self._getDriveModulePositions())
    hasVisionTarget = False
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        if self._isValidRobotPose(estimatedRobotPose.estimatedPose):
          totalTargets = 0
          totalDistance = 0
          for target in estimatedRobotPose.targetsUsed:
            distance = target.getBestCameraToTarget().translation().norm()
            if self._isValidTarget(distance, target.getPoseAmbiguity(), estimatedRobotPose.strategy):
              totalTargets += 1
              totalDistance += distance
          if totalTargets > 0:
            hasVisionTarget = True
            avgDistance = totalDistance / totalTargets
            stdDevTranslation = 0.02 * avgDistance * avgDistance if totalTargets > 1 else 0.5 * avgDistance / 4.0
            stdDevRotation = 0.3 * avgDistance * avgDistance if totalTargets > 1 else 1.0
            self._poseEstimator.addVisionMeasurement(
              estimatedRobotPose.estimatedPose.toPose2d(), 
              estimatedRobotPose.timestampSeconds,
              (stdDevTranslation, stdDevTranslation, stdDevRotation)
            )
    self._robotPose = self._poseEstimator.getEstimatedPosition()
    if hasVisionTarget:
      self._hasValidVisionTarget = True
      self._validVisionTargetBufferTimer.restart()
    else:
      if self._hasValidVisionTarget and self._validVisionTargetBufferTimer.hasElapsed(0.1):
        self._hasValidVisionTarget = False

  def _isValidTarget(self, distance: units.meters, ambiguity: units.percent, strategy: PoseStrategy) -> bool:
    return (
      distance <= constants.Services.Localization.kVisionMaxTargetDistance and (
        strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR or 
        utils.isValueInRange(ambiguity, 0, constants.Services.Localization.kVisionMaxPoseAmbiguity)
      )
    )

  def _isValidRobotPose(self, pose: Pose3d) -> bool:
    return (
      utils.isPoseInBounds(pose.toPose2d(), constants.Game.Field.kBounds) and 
      math.fabs(pose.Z()) <= constants.Services.Localization.kRobotPoseMaxGroundPlaneDelta
    )

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.kTargets[self._alliance]
      self._targetPoses = [t.pose.toPose2d() for t in self._targets.values()]

  def getTargetPose(self) -> Pose3d:
    target = self._targets.get(utils.getTargetHash(self._robotPose.nearest(self._targetPoses)))
    if target is not None:
      return target.pose.transformBy(Transform3d())
    return Pose3d(self._robotPose)

  def hasValidVisionTarget(self) -> bool:
    return self._hasValidVisionTarget

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Localization/HasValidVisionTarget", self._hasValidVisionTarget)
    self._robotPosePublisher.set(self._robotPose)
