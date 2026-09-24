from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.components.speed_module import SpeedModule
from lib.components.follower_module import FollowerModule
import core.constants as constants

class Gripper(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Gripper

    self._gripperLeader = SpeedModule(self._constants.GRIPPER_LEADER_CONFIG)
    self._gripperFollower = FollowerModule(self._constants.GRIPPER_FOLLOWER_CONFIG)

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