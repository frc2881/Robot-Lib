from typing import Any, Callable, TypeVar
import math, numpy, json
import wpilib, wpimath
from wpimath import units, Pose2d, Pose3d, Rectangle2d, ChassisVelocities
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
  _robot.add_periodic(callback, period, offset)

def getRobotTime() -> units.seconds:
  return wpilib.Timer.get_timestamp()

def getRobotState() -> RobotState:
  if wpilib.RobotState.is_enabled(): return RobotState.ENABLED
  elif wpilib.RobotState.is_e_stopped(): return RobotState.ESTOPPED
  else: return RobotState.DISABLED

def getRobotMode() -> RobotMode:
  if wpilib.RobotState.is_teleop(): return RobotMode.TELEOP
  elif wpilib.RobotState.is_autonomous(): return RobotMode.AUTO
  elif wpilib.RobotState.is_utility(): return RobotMode.UTILITY
  else: return RobotMode.DISABLED

def getValueForRobotMode(autoValue: T, teleopValue: T) -> T:
  return autoValue if getRobotMode() == RobotMode.AUTO else teleopValue 

def isAutonomousMode() -> bool:
  return getRobotMode() == RobotMode.AUTO

def isCompetitionMode() -> bool:
  return wpilib.RobotState.is_fms_attached()

def getAlliance() -> Alliance:
  return Alliance.RED if wpilib.MatchState.get_alliance() == 0 else Alliance.BLUE

def getValueForAlliance(blueValue: T, redValue: T) -> T:
  return blueValue if getAlliance() == Alliance.BLUE else redValue

def getMatchTime() -> units.seconds:
  return math.floor(wpilib.MatchState.get_match_time())

def isValueWithinRange(value: float, minValue: float, maxValue: float) -> bool:
  return value >= minValue and value <= maxValue

def isValueWithinTolerance(value: float, targetValue: float, tolerance: float) -> bool:
  return math.isclose(value, targetValue, abs_tol = tolerance)

def clampValue(value: float, minValue: float, maxValue: float) -> float:
  return max(min(value, maxValue), minValue)

def wrapAngle(angle: units.degrees, inputRange = Range(-180, 180)) -> units.degrees:
  return wpimath.input_modulus(angle, inputRange.min, inputRange.max)

def getInterpolatedValue(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
  try: return numpy.interp([x], xs, ys)[0]
  except: return Value.none

def isPoseWithinBounds(pose: Pose2d, bounds: Rectangle2d) -> bool:
  return bounds.contains(pose.translation())

def isPoseAlignedToTarget(sourcePose: Pose2d, targetPose: Pose3d, translationTolerance: units.meters, rotationTolerance: units.degrees) -> bool:
  transform = sourcePose - targetPose.to_pose2d()
  return (
    isValueWithinTolerance(transform.translation().x, 0, translationTolerance) and
    isValueWithinTolerance(transform.translation().y, 0, translationTolerance) and
    isValueWithinTolerance(transform.rotation().degrees(), 0, rotationTolerance)
  )

def getTargetDistance(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d) -> units.meters:
  return math.dist(
    (sourcePose.x, sourcePose.y, sourcePose.z if isinstance(sourcePose, Pose3d) else 0), 
    (targetPose.x, targetPose.y, targetPose.z if isinstance(targetPose, Pose3d) else 0)
  )

def getTargetHeading(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d, isRobotRelative: bool = False) -> units.degrees:
  if isinstance(sourcePose, Pose3d): sourcePose = sourcePose.to_pose2d()
  if isinstance(targetPose, Pose3d): targetPose = targetPose.to_pose2d()
  return units.radians_to_degrees(math.atan2(targetPose.y - sourcePose.y, targetPose.x - sourcePose.x) - (sourcePose.rotation().radians() if isRobotRelative else 0))

def getTargetPitch(sourcePose: Pose3d, targetPose: Pose3d) -> units.degrees:
  return units.radians_to_degrees(math.atan2((targetPose - sourcePose).z, getTargetDistance(sourcePose, targetPose)))

def getTargetHash(pose: Pose2d) -> int:
  return hash((pose.x, pose.y, pose.rotation().radians()))

def squareControllerInput(input: units.percent, deadband: units.percent) -> units.percent:
  deadbandInput: units.percent = wpimath.apply_deadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def clampTranslationVelocity(chassisSpeeds: ChassisVelocities, translationMaxVelocity: units.meters_per_second) -> ChassisVelocities:
  if not isValueWithinRange(chassisSpeeds.vx, -translationMaxVelocity, translationMaxVelocity) or not isValueWithinRange(chassisSpeeds.vy, -translationMaxVelocity, translationMaxVelocity):
    dv = translationMaxVelocity / abs(max(chassisSpeeds.vx, chassisSpeeds.vy, key = abs))
    chassisSpeeds = ChassisVelocities(chassisSpeeds.vx * dv, chassisSpeeds.vy * dv, chassisSpeeds.omega)
  return chassisSpeeds

def getSparkController(port: wpilib.CANPort, id: int, controllerType: SparkLowLevel.SparkModel, motorType: SparkLowLevel.MotorType) -> SparkBase:
  return SparkFlex(port, id, motorType) if controllerType == SparkLowLevel.SparkModel.SPARK_FLEX else SparkMax(port, id, motorType)

def configureSparkController(controller: SparkBase, sparkConfig: SparkBaseConfig, isPersisted: bool = True) -> None:
  error = controller.configure(
    sparkConfig, 
    ResetMode.RESET_SAFE_PARAMETERS if isPersisted else ResetMode.NO_RESET_SAFE_PARAMETERS,
    PersistMode.PERSIST_PARAMETERS if isPersisted else PersistMode.NO_PERSIST_PARAMETERS
  )
  if error != REVLibError.OK: 
    logger.error(f'REVLibError: {error}')

def setSoftLimitsEnabled(controller: SparkBase, enabled: bool) -> None:
  sparkConfig = SparkBaseConfig()
  sparkConfig.soft_limit.reverse_soft_limit_enabled(enabled).forward_soft_limit_enabled(enabled)
  configureSparkController(controller, sparkConfig, isPersisted = False)

def setIdleMode(controller: SparkBase, idleMode: IdleMode) -> None:
  sparkConfig = SparkBaseConfig()
  sparkConfig.set_idle_mode(SparkBaseConfig.IdleMode.COAST if idleMode == IdleMode.COAST else SparkBaseConfig.IdleMode.BRAKE)
  configureSparkController(controller, sparkConfig, isPersisted = False)

def toJson(value: Any) -> str:
  try: return json.dumps(value, default=lambda o: o.__dict__)
  except: return "{}"
