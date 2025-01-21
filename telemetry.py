import math
from wpilib import Timer, DriverStation, RobotController, SmartDashboard, LiveWindow
from . import logger, utils

def start() -> None:
  utils.addRobotPeriodic(_updateTimingInfo, 0.3, 0.25)
  utils.addRobotPeriodic(_updateRobotInfo, 1.0, 0.5)
  utils.addRobotPeriodic(_updateTelemetrySetting, 3.0, 0.75)

def _updateTimingInfo() -> None:
  SmartDashboard.putNumber("Robot/FPGATimestamp", Timer.getFPGATimestamp())
  SmartDashboard.putNumber("Robot/Game/MatchTime",  math.floor(DriverStation.getMatchTime()))

def _updateRobotInfo() -> None:
  SmartDashboard.putString("Robot/Mode", utils.getRobotMode().name)
  SmartDashboard.putString("Robot/State", utils.getRobotState().name)
  SmartDashboard.putString("Robot/Game/Alliance", utils.getAlliance().name)
  SmartDashboard.putNumber("Robot/Game/Team", RobotController.getTeamNumber())
  SmartDashboard.putNumber("Robot/Game/Station", DriverStation.getLocation() or 0)
  SmartDashboard.putNumber("Robot/Power/Battery/Voltage", RobotController.getBatteryVoltage())

def _updateTelemetrySetting() -> None:
  if DriverStation.isFMSAttached():
    SmartDashboard.putBoolean("Robot/IsAllTelemetryEnabled", False)
    if LiveWindow.isEnabled():
      LiveWindow.setEnabled(False)
  else:
    isAllTelemetryEnabled: bool = SmartDashboard.getBoolean("Robot/IsAllTelemetryEnabled", False)
    if isAllTelemetryEnabled != LiveWindow.isEnabled():
      LiveWindow.setEnabled(isAllTelemetryEnabled)
    