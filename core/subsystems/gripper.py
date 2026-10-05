from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.modules.speed_control import SpeedControlModule
from lib.modules.follower_control import FollowerControlModule
import core.constants as constants

class Gripper(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Gripper

    self._telemetryName = "Robot/Subsystems/Gripper"

    self._gripperLeader = SpeedControlModule(self._constants.GRIPPER_LEADER_CONFIG)
    self._gripperFollower = FollowerControlModule(self._constants.GRIPPER_FOLLOWER_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def intake(self) -> Command:
    return self.startEnd(
      lambda: self._gripperLeader.setSpeed(-self._constants.GRIPPER_INTAKE_SPEED),
      lambda: self.reset()
    )
  
  def score(self) -> Command:
    return self.startEnd(
      lambda: self._gripperLeader.setSpeed(self._constants.GRIPPER_SCORE_SPEED),
      lambda: self.reset()
    )

  def reset(self) -> None:
    self._gripperLeader.reset()

  def _updateTelemetry(self) -> None:
    pass