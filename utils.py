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
from wpimath.kinematics import ChassisSpeeds
from rev import SparkBase, SparkBaseConfig, REVLibError, ResetMode, PersistMode
from . import logger
from .classes import Alliance, RobotMode, RobotState, MotorIdleMode, Value

T = TypeVar("T")

__robot__: TimedCommandRobot = None

def setRobotInstance(instance: TimedCommandRobot) -> None:
  global __robot__
  __robot__ = instance

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
  __robot__.addPeriodic(callback, period, offset)

def getRobotState() -> RobotState:
  if wpilib.RobotState.isEnabled(): return RobotState.Enabled
  elif wpilib.RobotState.isEStopped(): return RobotState.EStopped
  else: return RobotState.Disabled

def getRobotMode() -> RobotMode:
  if wpilib.RobotState.isTeleop(): return RobotMode.Teleop
  elif wpilib.RobotState.isAutonomous(): return RobotMode.Auto
  elif wpilib.RobotState.isTest(): return RobotMode.Test
  else: return RobotMode.Disabled

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

def wrapAngle(angle: units.degrees) -> units.degrees:
  return wpimath.inputModulus(angle, -180, 180)

def getInterpolatedValue(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  try: return numpy.interp([x], xs, ys)[0]
  except: return Value.none

def isPoseInBounds(pose: Pose2d, bounds: Tuple[Translation2d, Translation2d]) -> bool:
  return isValueInRange(pose.X(), bounds[0].X(), bounds[1].X()) and isValueInRange(pose.Y(), bounds[0].Y(), bounds[1].Y())

def isPoseAlignedToTarget(sourcePose: Pose2d, targetPose: Pose3d, translationTolerance: units.meters, rotationTolerance: units.degrees) -> bool:
  transform = sourcePose - targetPose.toPose2d()
  return (
    isValueInRange(transform.translation().X(), -translationTolerance, translationTolerance) and 
    isValueInRange(transform.translation().Y(), -translationTolerance, translationTolerance) and
    isValueInRange(transform.rotation().degrees(), -rotationTolerance, rotationTolerance)
  )

def getTargetHeading(sourcePose: Pose2d, targetPose: Pose2d) -> units.degrees:
  translation = targetPose.relativeTo(sourcePose).translation()
  return wrapAngle(Rotation2d(translation.X(), translation.Y()).rotateBy(sourcePose.rotation()).degrees())

def getTargetDistance(sourcePose: Pose3d, targetPose: Pose3d) -> units.meters:
  return sourcePose.translation().distance(targetPose.translation())

def getTargetPitch(sourcePose: Pose3d, targetPose: Pose3d) -> units.degrees:
  return units.radiansToDegrees(math.atan2((targetPose - sourcePose).Z(), getTargetDistance(sourcePose, targetPose)))

def getTargetHash(pose: Pose2d) -> int:
  return hash((pose.X(), pose.Y(), pose.rotation().radians()))

def squareControllerInput(input: units.percent, deadband: units.percent) -> units.percent:
  deadbandInput: units.percent = wpimath.applyDeadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def clampTranslationVelocity(chassisSpeeds: ChassisSpeeds, translationMaxVelocity: units.meters_per_second) -> ChassisSpeeds:
  if not isValueInRange(chassisSpeeds.vx, -translationMaxVelocity, translationMaxVelocity) or not isValueInRange(chassisSpeeds.vy, -translationMaxVelocity, translationMaxVelocity):
    dv = translationMaxVelocity / abs(max(chassisSpeeds.vx, chassisSpeeds.vy, key = abs))
    chassisSpeeds = ChassisSpeeds(chassisSpeeds.vx * dv, chassisSpeeds.vy * dv, chassisSpeeds.omega)
  return chassisSpeeds

def setSoftLimitsEnabled(motor: SparkBase, enabled: bool) -> None:
  config = SparkBaseConfig()
  config.softLimit.reverseSoftLimitEnabled(enabled).forwardSoftLimitEnabled(enabled)
  setSparkConfig(motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters))

def setMotorIdleMode(motor: SparkBase, motorIdleMode: MotorIdleMode) -> None:
  setSparkConfig(motor.configure(SparkBaseConfig().setIdleMode(
    SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
  ), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters))

def setSparkConfig(error: REVLibError) -> None:
  if error != REVLibError.kOk: logger.error(f'REVLibError: {error}')

def toJson(value: Any) -> str:
  try: return json.dumps(value, default=lambda o: o.__dict__)
  except: return "{}"
