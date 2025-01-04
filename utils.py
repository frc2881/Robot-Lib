from typing import Any, Callable, Tuple, TypeVar
import math
import numpy
import json
from commands2 import TimedCommandRobot
import wpilib
import wpimath
from wpimath import units
from wpimath.geometry import Pose2d, Translation2d
from wpilib import DriverStation
from rev import SparkBase, SparkBaseConfig, REVLibError
from . import logger
from .classes import Alliance, RobotMode, RobotState

T = TypeVar("T")

robot: TimedCommandRobot = None

def setRobotInstance(instance: TimedCommandRobot) -> None:
  global robot
  robot = instance

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
  robot.addPeriodic(callback, period, offset)

def getRobotState() -> RobotState:
  if wpilib.RobotState.isEnabled():
    return RobotState.Enabled
  elif wpilib.RobotState.isEStopped():
    return RobotState.EStopped
  else:
    return RobotState.Disabled

def getRobotMode() -> RobotMode:
  if wpilib.RobotState.isTeleop():
    return RobotMode.Teleop
  elif wpilib.RobotState.isAutonomous():
    return RobotMode.Auto
  elif wpilib.RobotState.isTest():
    return RobotMode.Test
  else:
    return RobotMode.Disabled

def getValueForRobotMode(autoValue: T, teleopValue: T) -> T:
  return autoValue if getRobotMode() == RobotMode.Auto else teleopValue 

def isAutonomousMode() -> bool:
  return getRobotMode() == RobotMode.Auto

def isCompetitionMode() -> bool:
  return DriverStation.isFMSAttached()

def getAlliance() -> Alliance:
  return Alliance(DriverStation.getAlliance() or Alliance.Blue)

def getValueForAlliance(blueValue: T, redValue: T) -> T:
  return blueValue if getAlliance() == Alliance.Blue else redValue

def getMatchTime() -> units.seconds:
  return DriverStation.getMatchTime()

def isValueInRange(value: float, minValue: float, maxValue: float) -> bool:
  return value >= minValue and value <= maxValue

def squareControllerInput(input: units.percent, deadband: units.percent) -> units.percent:
  deadbandInput: units.percent = wpimath.applyDeadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def wrapAngle(angle: units.degrees) -> units.degrees:
  return wpimath.inputModulus(angle, -180, 180)

def isPoseInBounds(pose: Pose2d, bounds: Tuple[Translation2d, Translation2d]) -> bool:
  return isValueInRange(pose.X(), bounds[0].X(), bounds[1].X()) and isValueInRange(pose.Y(), bounds[0].Y(), bounds[1].Y())

def getInterpolatedValue(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  try:
    return numpy.interp([x], xs, ys)[0]
  except:
    return math.nan

def setSparkSoftLimitsEnabled(motor: SparkBase, enabled: bool) -> None:
  config = SparkBaseConfig()
  config.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled)
  setSparkConfig(
    motor.configure(
      config, 
      SparkBase.ResetMode.kNoResetSafeParameters, 
      SparkBase.PersistMode.kNoPersistParameters
    )
  )

def setSparkConfig(error: REVLibError) -> None:
  if error != REVLibError.kOk:
    logger.error(f'REVLibError: {error}')

def toJson(value: Any) -> str:
  try:
    return json.dumps(value, default=lambda o: o.__dict__)
  except:
    return "{}"