from typing import Any
import math
from wpilib import Timer, DriverStation, RobotController, SmartDashboard
from . import logger, utils

def start() -> None:
  _updateTimingInfo()
  _updateRobotInfo()
  _updateGameMatchInfo()
  utils.addRobotPeriodic(_updateTimingInfo, 0.1, 0.25)
  utils.addRobotPeriodic(_updateRobotInfo, 0.2, 0.50)
  utils.addRobotPeriodic(_updateGameMatchInfo, 1.0, 0.75)

def _updateTimingInfo() -> None:
  log("Robot/Status/Time", Timer.getFPGATimestamp())
  log("Match/Time",  math.floor(DriverStation.getMatchTime()))

def _updateRobotInfo() -> None:
  log("Robot/Status/Mode", utils.getRobotMode().name)
  log("Robot/Status/State", utils.getRobotState().name)
  log("Robot/Power/Battery/Voltage", RobotController.getBatteryVoltage())

def _updateGameMatchInfo() -> None:
  log("Game/Team", RobotController.getTeamNumber())
  log("Match/Alliance", utils.getAlliance().name)
  log("Match/Station", DriverStation.getLocation() or 0)
  log("Match/IsCompetitionMode", utils.isCompetitionMode())

def log(name: str, value: Any, element_type: type[Any] | None = None) -> None:
  match value:
    case str():
      SmartDashboard.putString(name, value)
    case bool():
      SmartDashboard.putBoolean(name, value)
    case int() | float():
      SmartDashboard.putNumber(name, value)
    case list():
      if element_type is str:
        SmartDashboard.putStringArray(name, value)
      elif element_type is bool:
        SmartDashboard.putBooleanArray(name, value)
      elif element_type is int or element_type is float:
        SmartDashboard.putNumberArray(name, value)
      else:
        pass
    case _:
      pass
