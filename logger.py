import traceback
from wpilib import DataLogManager, DriverStation
from commands2 import CommandScheduler, Command, cmd
from . import telemetry, utils
from .classes import RobotMode

def start() -> None:
  DataLogManager.start()
  DriverStation.startDataLog(DataLogManager.getLog())

  CommandScheduler.getInstance().onCommandInitialize(
    lambda command: log(f'---> Command Start: {command.getName()}')
  )
  CommandScheduler.getInstance().onCommandInterrupt(
    lambda command: log(f'---X Command Interrupt: {command.getName()}')
  )
  CommandScheduler.getInstance().onCommandFinish(
    lambda command: log(f'---< Command End: {command.getName()}')
  )

  log("+++++ Robot Started +++++")

  telemetry.log("Robot/Status/HasError", False)
  telemetry.log("Robot/Status/LastError", "")

def log(message: str) -> None:
  DataLogManager.log(f'[{"%.6f" % utils.getRobotTime()}] {message}')

def log_(message: str) -> Command:
  return cmd.runOnce(lambda: log(message))

def mode(mode: RobotMode) -> None:
  log(f'>>>>> Robot Mode Changed: {mode.name} <<<<<')

def debug(message: str) -> None:
  log(f'@@@@@ DEBUG: {message} @@@@@')

def debug_(message: str) -> Command:
  return cmd.runOnce(lambda: debug(message))

def error(message: str) -> None:
  log(f'!!!!! ERROR: {message} !!!!!')
  telemetry.log("Robot/Status/HasError", True)
  telemetry.log("Robot/Status/LastError", message)

def exception() -> None:
  error(traceback.format_exc())
