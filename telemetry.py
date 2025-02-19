import math
from wpilib import Timer, DriverStation, RobotController, SmartDashboard
from . import logger, utils

def start() -> None:
  utils.addRobotPeriodic(_updateTimingInfo, 0.3, 0.25)
  utils.addRobotPeriodic(_updateRobotInfo, 1.0, 0.50)
  utils.addRobotPeriodic(_updateGameInfo, 3.0, 0.75)

def _updateTimingInfo() -> None:
  SmartDashboard.putNumber("Robot/Status/Time", Timer.getFPGATimestamp())
  SmartDashboard.putNumber("Game/MatchTime",  math.floor(DriverStation.getMatchTime()))

def _updateRobotInfo() -> None:
  SmartDashboard.putString("Robot/Status/Mode", utils.getRobotMode().name)
  SmartDashboard.putString("Robot/Status/State", utils.getRobotState().name)
  SmartDashboard.putNumber("Robot/Power/Battery/Voltage", RobotController.getBatteryVoltage())
  SmartDashboard.putNumber("Robot/Power/Battery/IsBrownedOut", RobotController.isBrownedOut())

def _updateGameInfo() -> None:
  SmartDashboard.putString("Game/Alliance", utils.getAlliance().name)
  SmartDashboard.putNumber("Game/Team", RobotController.getTeamNumber())
  SmartDashboard.putNumber("Game/Station", DriverStation.getLocation() or 0)
