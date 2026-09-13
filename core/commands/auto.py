from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  CUSTOM = auto()

class Auto:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

  def get(self) -> Command:
    return cmd.none()
