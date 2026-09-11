from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from lib import logger, utils
from lib.components.limit_position_control_module import LimitPositionControlModule
from lib.components.follower_module import FollowerModule
import core.constants as constants

class Arm(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Arm

    self._armLeader = LimitPositionControlModule(self._constants.ARM_LEADER_CONFIG)
    self._armFollower = FollowerModule(self._constants.ARM_FOLLOWER_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: float) -> Command:
    return self.startEnd(
      lambda: self._armLeader.setPosition(position),
      lambda: self._armLeader.setPosition(0)
    ).withName("Arm:SetPosition")
  
  def getPosition(self) -> float:
    return self._armLeader.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._armLeader.isAtTargetPosition()

  def reset(self) -> None:
    self._armLeader.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Arm/IsAtTargetPosition", self.isAtTargetPosition())
