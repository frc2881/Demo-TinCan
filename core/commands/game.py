from typing import TYPE_CHECKING
from wpilib import RobotBase
from commands2 import Command, cmd
from lib import logger, telemetry, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.BOTH, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.SHORT
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.OPERATOR),
      # self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.DRIVER)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
