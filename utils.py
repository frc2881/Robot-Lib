from typing import Any, Callable, Tuple, TypeVar
import math
import numpy
import json
from commands2 import TimedCommandRobot
import wpilib
import wpimath
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Rotation2d
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

def clampValue(value: float, minValue: float, maxValue: float) -> float:
  return max(min(value, maxValue), minValue)

def squareControllerInput(input: units.percent, deadband: units.percent) -> units.percent:
  deadbandInput: units.percent = wpimath.applyDeadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def wrapAngle(angle: units.degrees) -> units.degrees:
  return wpimath.inputModulus(angle, -180, 180)

def isPoseInBounds(pose: Pose2d, bounds: Tuple[Translation2d, Translation2d]) -> bool:
  return isValueInRange(pose.X(), bounds[0].X(), bounds[1].X()) and isValueInRange(pose.Y(), bounds[0].Y(), bounds[1].Y())

def getTargetHash(pose: Pose2d) -> int:
  return hash((pose.X(), pose.Y(), pose.rotation().radians()))

def getTargetHeading(robotPose: Pose2d, targetPose: Pose3d) -> units.degrees:
  translation = targetPose.toPose2d().relativeTo(robotPose).translation()
  rotation = Rotation2d(translation.X(), translation.Y()).rotateBy(robotPose.rotation())
  return wrapAngle(rotation.degrees())

def getTargetDistance(robotPose: Pose2d, targetPose: Pose3d) -> units.meters:
  return robotPose.translation().distance(targetPose.toPose2d().translation())

def getTargetPitch(robotPose: Pose2d, targetPose: Pose3d) -> units.degrees:
  return math.degrees(math.atan2((targetPose - Pose3d(robotPose)).Z(), getTargetDistance(robotPose, targetPose)))

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