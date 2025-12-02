from typing import NamedTuple
import sys
import math
from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout
from rev import SparkLowLevel
from photonlibpy.photonPoseEstimator import PoseStrategy

class Alliance(IntEnum):
  Red = 0
  Blue = 1

class RobotMode(Enum):
  Disabled = auto()
  Auto = auto()
  Teleop = auto()
  Test = auto()

class RobotState(Enum):
  Disabled = auto()
  Enabled = auto()
  EStopped = auto()

class State(Enum):
  Disabled = auto()
  Enabled = auto()
  Stopped = auto()
  Running = auto()
  Completed = auto()

class Position(Enum):
  Unknown = auto()
  Default = auto()
  Up = auto()
  Down = auto()
  Left = auto()
  Right = auto()
  Front = auto()
  Rear = auto()
  Top = auto()
  Bottom = auto()
  Center = auto()
  Open = auto()
  Closed = auto()
  In = auto()
  Out = auto()
  Unlocked = auto()
  Locked = auto()

class MotorDirection(Enum):
  Forward = auto()
  Reverse = auto()
  Stop = auto()

class MotorIdleMode(Enum):
  Brake = auto()
  Coast = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

class TargetAlignmentMode(Enum):
  Heading = auto()
  Translation = auto()

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

class PID(NamedTuple):
  P: float
  I: float
  D: float

class Range(NamedTuple):
  min: float
  max: float

class Tolerance(NamedTuple):
  error: float
  errorDerivative: float

@dataclass(frozen=True, slots=True)
class Value(float):
  none = math.nan
  min = sys.float_info.min
  max = sys.float_info.max

@dataclass(frozen=True, slots=True)
class DriftCorrectionConstants:
  rotationPID: PID
  rotationTolerance: Tolerance

@dataclass(frozen=True, slots=True)
class TargetAlignmentConstants:
  translationPID: PID
  translationMaxVelocity: units.meters_per_second
  translationMaxAcceleration: units.meters_per_second_squared
  translationTolerance: Tolerance
  rotationPID: PID
  rotationMaxVelocity: units.degrees_per_second
  rotationMaxAcceleration: units.degrees_per_second_squared
  rotationTolerance: Tolerance
  rotationHeadingModeOffset: units.degrees
  rotationTranslationModeOffset: units.degrees

@dataclass(frozen=True, slots=True)
class SwerveModuleConstants:
  wheelDiameter: units.meters
  wheelBevelGearTeeth: int
  wheelSpurGearTeeth: int
  wheelBevelPinionTeeth: int
  drivingMotorPinionTeeth: int
  drivingMotorFreeSpeed: units.revolutions_per_minute
  drivingMotorControllerType: SparkLowLevel.SparkModel
  drivingMotorType: SparkLowLevel.MotorType
  drivingMotorCurrentLimit: int
  drivingMotorPID: PID
  turningMotorCurrentLimit: int
  turningMotorPID: PID

class SwerveModuleLocation(IntEnum):
  FrontLeft = 0,
  FrontRight = 1,
  RearLeft = 2,
  RearRight = 3

@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
  location: SwerveModuleLocation
  drivingMotorCANId: int
  turningMotorCANId: int
  turningOffset: units.degrees
  translation: Translation2d
  constants: SwerveModuleConstants

@dataclass(frozen=True, slots=True)
class DifferentialModuleConstants:
  wheelDiameter: units.meters
  drivingMotorControllerType: SparkLowLevel.SparkModel
  drivingMotorType: SparkLowLevel.MotorType
  drivingMotorCurrentLimit: int
  drivingMotorReduction: float

class DifferentialModuleLocation(IntEnum):
  Left = 0,
  Right = 1

@dataclass(frozen=True, slots=True)
class DifferentialModuleConfig:
  location: DifferentialModuleLocation
  drivingMotorCANId: int
  leaderMotorCANId: int | None
  isInverted: bool
  constants: DifferentialModuleConstants

class DifferentialDriveModulePositions(NamedTuple):
  left: units.meters
  right: units.meters

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int
  motorReduction: float
  motorPID: PID
  motorOutputRange: Range
  motorMotionMaxVelocity: units.units_per_second
  motorMotionMaxAcceleration: units.units_per_second_squared
  motorMotionAllowedClosedLoopError: float
  motorSoftLimitReverse: float
  motorSoftLimitForward: float
  motorResetSpeed: units.percent
  distancePerRotation: float

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConfig:
  moduleBaseKey: str
  motorCANId: int
  isInverted: bool
  constants: RelativePositionControlModuleConstants

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int
  motorReduction: float
  motorPID: PID
  motorOutputRange: Range
  motorMotionMaxVelocity: units.units_per_second
  motorMotionMaxAcceleration: units.units_per_second_squared
  motorMotionAllowedClosedLoopError: float
  motorSoftLimitReverse: units.inches
  motorSoftLimitForward: units.inches
  motorResetSpeed: units.percent

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConfig:
  moduleBaseKey: str
  motorCANId: int
  isInverted: bool
  constants: AbsolutePositionControlModuleConstants

@dataclass(frozen=True, slots=True)
class FollowerModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int

@dataclass(frozen=True, slots=True)
class FollowerModuleConfig:
  moduleBaseKey: str
  motorCANId: int
  leaderMotorCANId: int
  isInverted: bool
  constants: FollowerModuleConstants

@dataclass(frozen=True, slots=True)
class PoseSensorConstants:
  aprilTagFieldLayout: AprilTagFieldLayout
  poseStrategy: PoseStrategy
  fallbackPoseStrategy: PoseStrategy

@dataclass(frozen=True, slots=True)
class PoseSensorConfig:
  cameraName: str
  cameraTransform: Transform3d
  stream: str
  constants: PoseSensorConstants

@dataclass(frozen=True, slots=True)
class BinarySensorConfig:
  sensorName: str
  digitalInputChannel: int

@dataclass(frozen=True, slots=True)
class DistanceSensorConfig:
  sensorName: str
  digitalInputChannel: int
  pulseWidthConversionFactor: float
  minTargetDistance: units.millimeters
  maxTargetDistance: units.millimeters

@dataclass(frozen=True, slots=True)
class ObjectSensorConfig:
  cameraName: str
  cameraTransform: Transform3d
  stream: str
