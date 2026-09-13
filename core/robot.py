from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.controllers.xbox import XboxController
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive 
from core.subsystems.arm import Arm
from core.subsystems.gripper import Gripper
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initCommands()
    self._initControllers()
    self._initTriggers()
    self._initTelemetry()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    pass
    
  def _initSubsystems(self) -> None:
    self.drive = Drive()
    self.arm = Arm()
    self.gripper = Gripper()
    
  def _initServices(self) -> None:
    pass

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())
    self.driver = XboxController(constants.Controllers.DRIVER_CONTROLLER_PORT, constants.Controllers.INPUT_DEADBAND)

  def _initTriggers(self) -> None:
    self._setupDriver()

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getRightX))
    # self.driver.leftStick().whileTrue(cmd.none())
    # self.driver.rightStick().whileTrue(cmd.none())
    self.driver.leftTrigger().whileTrue(self.gripper.intake())
    self.driver.rightTrigger().whileTrue(self.gripper.score())
    self.driver.leftBumper().whileTrue(self.arm.setBackward())
    self.driver.rightBumper().whileTrue(self.arm.setForward())
    # self.driver.a().whileTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    # self.driver.x().whileTrue(cmd.none())
    # self.driver.y().whileTrue(cmd.none())
    # self.driver.povUp().whileTrue(cmd.none())
    # self.driver.povDown().whileTrue(cmd.none())
    # self.driver.povLeft().whileTrue(cmd.none())
    # self.driver.povRight().whileTrue(cmd.none())
    # self.driver.start().onTrue(cmd.none())
    # self.driver.back().whileTrue(cmd.none())

  def _initTelemetry(self) -> None:
    SmartDashboard.putString("Game/Robot/Type", constants.Game.Robot.TYPE.name)
    SmartDashboard.putString("Game/Robot/Name", constants.Game.Robot.NAME)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    pass

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()

  def isHoming(self) -> bool:
    return False

  def isHomed(self) -> bool:
    return True

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/IsHoming", self.isHoming())
    SmartDashboard.putBoolean("Robot/Status/IsHomed", self.isHomed())
