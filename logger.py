import traceback
from commands2 import CommandScheduler
from wpilib import DataLogManager, DriverStation, Timer, SmartDashboard
from .classes import RobotMode

def start() -> None:
  DataLogManager.start()
  DriverStation.startDataLog(DataLogManager.getLog())

  CommandScheduler.getInstance().onCommandInitialize(
    lambda command: log(f'----> Command Start: {command.getName()}')
  )
  CommandScheduler.getInstance().onCommandInterrupt(
    lambda command: log(f'----X Command Interrupt: {command.getName()}')
  )
  CommandScheduler.getInstance().onCommandFinish(
    lambda command: log(f'----< Command End: {command.getName()}')
  )

  SmartDashboard.putBoolean("Robot/Error/HasError", False)
  SmartDashboard.putString("Robot/Error/LastError", "")

  log("********** Robot Started **********")

def log(message: str) -> None:
  DataLogManager.log(f'[{"%.6f" % Timer.getFPGATimestamp()}] {message}')

def mode(mode: RobotMode) -> None:
  log(f'>>>>>>>>>> Robot Mode Changed: {mode.name} <<<<<<<<<<')

def debug(message: str) -> None:
  log(f'@@@@@@@@@@ DEBUG: {message} @@@@@@@@@@')

def error(message: str) -> None:
  log(f'!!!!!!!!!! ERROR: {message} !!!!!!!!!!')
  SmartDashboard.putBoolean("Robot/Error/HasError", True)
  SmartDashboard.putString("Robot/Error/LastError", message)

def exception() -> None:
  error(traceback.format_exc())
