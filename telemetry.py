from typing import Any
import math
from wpilib import Timer, DriverStation, RobotController
from ntcore import NetworkTableInstance, StructPublisher, StructArrayPublisher
from . import logger, utils

_publishers: dict[str, StructPublisher | StructArrayPublisher] = {}

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
  name = f'/Telemetry/{ name }'
  if hasattr(value, "WPIStruct") or hasattr(element_type, "WPIStruct"):
    topic = (
      NetworkTableInstance.getDefault().getStructArrayTopic(name, element_type)
      if element_type is not None else
      NetworkTableInstance.getDefault().getStructTopic(name, type(value))
    )
    if not topic.exists(): 
      _publishers[name] = topic.publish()
    _publishers[name].set(value)
  else:
    entry = NetworkTableInstance.getDefault().getEntry(name)
    if isinstance(value, list):
      if element_type is str: entry.setStringArray(value)
      elif element_type is bool: entry.setBooleanArray(value)
      elif element_type is int: entry.setIntegerArray(value)
      elif element_type is float: entry.setFloatArray(value)
      else: pass
    else:
      entry.setValue(value)
