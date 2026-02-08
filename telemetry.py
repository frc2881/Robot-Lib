import math
from wpilib import Timer, DriverStation, RobotController, SmartDashboard
from . import logger, utils

def start() -> None:
  _updateTimingInfo()
  _updateRobotInfo()
  _updateMatchInfo()
  utils.addRobotPeriodic(_updateTimingInfo, 0.2, 0.25)
  utils.addRobotPeriodic(_updateRobotInfo, 1.0, 0.50)
  utils.addRobotPeriodic(_updateMatchInfo, 3.0, 0.75)

def _updateTimingInfo() -> None:
  SmartDashboard.putNumber("Robot/Status/Time", Timer.getFPGATimestamp())
  SmartDashboard.putNumber("Match/Time",  math.floor(DriverStation.getMatchTime()))

def _updateRobotInfo() -> None:
  SmartDashboard.putString("Robot/Status/Mode", utils.getRobotMode().name)
  SmartDashboard.putString("Robot/Status/State", utils.getRobotState().name)
  SmartDashboard.putNumber("Robot/Power/Battery/Voltage", RobotController.getBatteryVoltage())
  SmartDashboard.putNumber("Robot/Power/IsBrownedOut", RobotController.isBrownedOut())

def _updateMatchInfo() -> None:
  SmartDashboard.putString("Match/Alliance", utils.getAlliance().name)
  SmartDashboard.putNumber("Match/Team", RobotController.getTeamNumber())
  SmartDashboard.putNumber("Match/Station", DriverStation.getLocation() or 0)
