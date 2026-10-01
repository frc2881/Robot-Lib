from typing import Any, Callable, TypeVar
import math, numpy, json
import wpilib, wpimath
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Rectangle2d
from wpimath.kinematics import ChassisSpeeds
from commands2 import TimedCommandRobot
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkFlex, SparkMax, REVLibError, ResetMode, PersistMode
from . import logger
from .classes import Alliance, RobotMode, RobotState, IdleMode, Value, Range

T = TypeVar("T")

_robot: TimedCommandRobot

def initRobot(robot: TimedCommandRobot) -> None:
  global _robot
  _robot = robot

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
  _robot.addPeriodic(callback, period, offset)

def getRobotTime() -> units.seconds:
  return wpilib.Timer.getTimestamp()

def getRobotState() -> RobotState:
  if wpilib.RobotState.isEnabled(): return RobotState.ENABLED
  elif wpilib.RobotState.isEStopped(): return RobotState.ESTOPPED
  else: return RobotState.DISABLED

def getRobotMode() -> RobotMode:
  if wpilib.RobotState.isTeleop(): return RobotMode.TELEOP
  elif wpilib.RobotState.isAutonomous(): return RobotMode.AUTO
  elif wpilib.RobotState.isTest(): return RobotMode.TEST
  else: return RobotMode.DISABLED

def getValueForRobotMode(autoValue: T, teleopValue: T) -> T:
  return autoValue if getRobotMode() == RobotMode.AUTO else teleopValue 

def isAutonomousMode() -> bool:
  return getRobotMode() == RobotMode.AUTO

def isCompetitionMode() -> bool:
  return wpilib.DriverStation.isFMSAttached()

def getAlliance() -> Alliance:
  return Alliance.RED if wpilib.DriverStation.getAlliance() == 1 else Alliance.BLUE

def getValueForAlliance(blueValue: T, redValue: T) -> T:
  return blueValue if getAlliance() == Alliance.BLUE else redValue

def getMatchTime() -> units.seconds:
  return math.floor(wpilib.DriverStation.getMatchTime())

def isValueWithinRange(value: float, minValue: float, maxValue: float) -> bool:
  return value >= minValue and value <= maxValue

def isValueWithinTolerance(value: float, targetValue: float, tolerance: float) -> bool:
  return math.isclose(value, targetValue, abs_tol = tolerance)

def clampValue(value: float, minValue: float, maxValue: float) -> float:
  return max(min(value, maxValue), minValue)

def wrapAngle(angle: units.degrees, inputRange = Range(-180, 180)) -> units.degrees:
  return wpimath.inputModulus(angle, inputRange.min, inputRange.max)

def getInterpolatedValue(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  try: return numpy.interp([x], xs, ys)[0]
  except: return Value.none

def isPoseWithinBounds(pose: Pose2d, bounds: Rectangle2d) -> bool:
  return bounds.contains(pose.translation())

def isPoseAlignedToTarget(sourcePose: Pose2d, targetPose: Pose3d, translationTolerance: units.meters, rotationTolerance: units.degrees) -> bool:
  transform = sourcePose - targetPose.toPose2d()
  return (
    isValueWithinTolerance(transform.translation().X(), 0, translationTolerance) and
    isValueWithinTolerance(transform.translation().Y(), 0, translationTolerance) and
    isValueWithinTolerance(transform.rotation().degrees(), 0, rotationTolerance)
  )

def getTargetDistance(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d) -> units.meters:
  return math.dist(
    (sourcePose.X(), sourcePose.Y(), sourcePose.Z() if isinstance(sourcePose, Pose3d) else 0), 
    (targetPose.X(), targetPose.Y(), targetPose.Z() if isinstance(targetPose, Pose3d) else 0)
  )

def getTargetHeading(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d, isRobotRelative: bool = False) -> units.degrees:
  if isinstance(sourcePose, Pose3d): sourcePose = sourcePose.toPose2d()
  if isinstance(targetPose, Pose3d): targetPose = targetPose.toPose2d()
  return units.radiansToDegrees(math.atan2(targetPose.Y() - sourcePose.Y(), targetPose.X() - sourcePose.X()) - (sourcePose.rotation().radians() if isRobotRelative else 0))

def getTargetPitch(sourcePose: Pose3d, targetPose: Pose3d) -> units.degrees:
  return units.radiansToDegrees(math.atan2((targetPose - sourcePose).Z(), getTargetDistance(sourcePose, targetPose)))

def getTargetHash(pose: Pose2d) -> int:
  return hash((pose.X(), pose.Y(), pose.rotation().radians()))

def squareControllerInput(input: units.percent, deadband: units.percent) -> units.percent:
  deadbandInput: units.percent = wpimath.applyDeadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def clampTranslationVelocity(chassisSpeeds: ChassisSpeeds, translationMaxVelocity: units.meters_per_second) -> ChassisSpeeds:
  if not isValueWithinRange(chassisSpeeds.vx, -translationMaxVelocity, translationMaxVelocity) or not isValueWithinRange(chassisSpeeds.vy, -translationMaxVelocity, translationMaxVelocity):
    dv = translationMaxVelocity / abs(max(chassisSpeeds.vx, chassisSpeeds.vy, key = abs))
    chassisSpeeds = ChassisSpeeds(chassisSpeeds.vx * dv, chassisSpeeds.vy * dv, chassisSpeeds.omega)
  return chassisSpeeds

def getSparkController(id: int, controllerType: SparkLowLevel.SparkModel, motorType: SparkLowLevel.MotorType) -> SparkBase:
  return SparkFlex(id, motorType) if controllerType == SparkLowLevel.SparkModel.kSparkFlex else SparkMax(id, motorType)

def configureSparkController(controller: SparkBase, sparkConfig: SparkBaseConfig, isPersisted: bool = True) -> None:
  error = controller.configure(
    sparkConfig, 
    ResetMode.kResetSafeParameters if isPersisted else ResetMode.kNoResetSafeParameters,
    PersistMode.kPersistParameters if isPersisted else PersistMode.kNoPersistParameters
  )
  if error != REVLibError.kOk: 
    logger.error(f'REVLibError: {error}')

def setSoftLimitsEnabled(controller: SparkBase, enabled: bool) -> None:
  sparkConfig = SparkBaseConfig()
  sparkConfig.softLimit.reverseSoftLimitEnabled(enabled).forwardSoftLimitEnabled(enabled)
  configureSparkController(controller, sparkConfig, isPersisted = False)

def setIdleMode(controller: SparkBase, idleMode: IdleMode) -> None:
  sparkConfig = SparkBaseConfig()
  sparkConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast if idleMode == IdleMode.COAST else SparkBaseConfig.IdleMode.kBrake)
  configureSparkController(controller, sparkConfig, isPersisted = False)

def toJson(value: Any) -> str:
  try: return json.dumps(value, default=lambda o: o.__dict__)
  except: return "{}"
