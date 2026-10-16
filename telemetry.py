from typing import Any
from wpilib import RobotController, MatchState
import telemetry
from . import logger, utils

def start() -> None:
  _updateRobotInfo()
  _updateGameMatchInfo()
  utils.addRobotPeriodic(_updateRobotInfo, 0.1, 0.25)
  utils.addRobotPeriodic(_updateGameMatchInfo, 2.0, 0.50)

def _updateRobotInfo() -> None:
  log("Robot/Status/Time", utils.getRobotTime())
  log("Robot/Status/Mode", utils.getRobotMode().name)
  log("Robot/Status/State", utils.getRobotState().name)
  log("Robot/Power/Battery/Voltage", RobotController.get_battery_voltage())

def _updateGameMatchInfo() -> None:
  log("Game/Team", RobotController.get_team_number())
  log("Match/Alliance", utils.getAlliance().name)
  log("Match/Station", MatchState.get_location() or 0)
  log("Match/IsCompetitionMode", utils.isCompetitionMode())

def log(name: str, value: Any, element_type: type[Any] | None = None) -> None:
  telemetry.log(name, value, element_type = element_type)
