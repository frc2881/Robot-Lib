from typing import NamedTuple
import sys
import math
from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout
from rev import SparkLowLevel, AbsoluteEncoderConfig

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

class RobotType(Enum):
  Other = auto()
  Competition = auto()
  Practice = auto()
  Demo = auto()

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

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

class PID(NamedTuple):
  P: float = 0
  I: float = 0
  D: float = 0

class FeedForwardGains(NamedTuple):
  static: units.volts = 0
  velocity: units.volts = 0
  acceleration: units.volts = 0
  gravity: units.volts = 0

class Range(NamedTuple):
  min: float
  max: float

@dataclass(frozen=True, slots=True)
class Value(float):
  none = math.nan
  min = sys.float_info.min
  max = sys.float_info.max

class MotorModel(Enum):
  NEO = auto()
  NEOVortex = auto()
  NEO550 = auto()

class SwerveModuleGearKit(Enum):
  Low = auto()
  Medium = auto()
  High = auto()
  ExtraHigh1 = auto()
  ExtraHigh2 = auto()
  ExtraHigh3 = auto()
  ExtraHigh4 = auto()
  ExtraHigh5 = auto()

class SwerveModuleLocation(IntEnum):
  FrontLeft = 0,
  FrontRight = 1,
  RearLeft = 2,
  RearRight = 3

@dataclass(frozen=True, slots=True)
class SwerveModuleConstants:
  wheelDiameter: units.meters
  drivingMotorControllerType: SparkLowLevel.SparkModel
  drivingMotorType: SparkLowLevel.MotorType
  drivingMotorFreeSpeed: units.revolutions_per_minute
  drivingMotorReduction: float
  drivingMotorCurrentLimit: int
  drivingMotorPID: PID
  turningMotorCurrentLimit: int
  turningMotorPID: PID
  turningMotorAbsoluteEncoderConfig: AbsoluteEncoderConfig

@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
  location: SwerveModuleLocation
  drivingMotorCANId: int
  turningMotorCANId: int
  turningOffset: units.degrees
  translation: Translation2d
  constants: SwerveModuleConstants

@dataclass(frozen=True, slots=True)
class RotationAlignmentConstants:
  rotationPID: PID
  rotationPositionTolerance: units.degrees

@dataclass(frozen=True, slots=True)
class TargetAlignmentConstants:
  translationPID: PID
  translationMaxVelocity: units.meters_per_second
  translationMaxAcceleration: units.meters_per_second_squared
  translationPositionTolerance: units.meters
  rotationPID: PID
  rotationMaxVelocity: units.degrees_per_second
  rotationMaxAcceleration: units.degrees_per_second_squared
  rotationPositionTolerance: units.degrees

class DifferentialModuleLocation(IntEnum):
  Left = 0,
  Right = 1

class DifferentialModulePositions(NamedTuple):
  left: units.meters
  right: units.meters

@dataclass(frozen=True, slots=True)
class DifferentialModuleConstants:
  wheelDiameter: units.meters
  drivingMotorControllerType: SparkLowLevel.SparkModel
  drivingMotorType: SparkLowLevel.MotorType
  drivingMotorReduction: float
  drivingMotorCurrentLimit: int

@dataclass(frozen=True, slots=True)
class DifferentialModuleConfig:
  location: DifferentialModuleLocation
  drivingMotorCANId: int
  leaderMotorCANId: int | None
  isInverted: bool
  constants: DifferentialModuleConstants

@dataclass(frozen=True, slots=True)
class FollowerModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int

@dataclass(frozen=True, slots=True)
class FollowerModuleConfig:
  baseKey: str
  motorCANId: int
  leaderMotorCANId: int
  isInverted: bool
  constants: FollowerModuleConstants

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int
  motorPID: PID
  motorOutputRange: Range
  motorFeedForwardGains: FeedForwardGains
  motorMotionCruiseVelocity: units.units_per_second
  motorMotionMaxAcceleration: units.units_per_second_squared
  motorMotionAllowedProfileError: float
  motorRelativeEncoderPositionConversionFactor: float
  motorSoftLimitReverse: float
  motorSoftLimitForward: float
  motorHomedPosition: float
  motorHomingSpeed: units.percent

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConfig:
  baseKey: str
  motorCANId: int
  isInverted: bool
  constants: RelativePositionControlModuleConstants

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConstants:
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int
  motorPID: PID
  motorOutputRange: Range
  motorFeedForwardGains: FeedForwardGains
  motorMotionCruiseVelocity: units.units_per_second
  motorMotionMaxAcceleration: units.units_per_second_squared
  motorMotionAllowedProfileError: float
  motorRelativeEncoderPositionConversionFactor: float
  motorAbsoluteEncoderPositionConversionFactor: float
  motorSoftLimitReverse: float
  motorSoftLimitForward: float

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConfig:
  baseKey: str
  motorCANId: int
  isInverted: bool
  constants: AbsolutePositionControlModuleConstants

@dataclass(frozen=True, slots=True)
class PoseSensorConfig:
  name: str
  transform: Transform3d
  stream: str
  aprilTagFieldLayout: AprilTagFieldLayout

@dataclass(frozen=True, slots=True)
class ObjectSensorConfig:
  name: str
  transform: Transform3d
  stream: str

@dataclass(frozen=True, slots=True)
class BinarySensorConfig:
  name: str
  channel: int

@dataclass(frozen=True, slots=True)
class DistanceSensorConfig:
  name: str
  channel: int
  pulseWidthConversionFactor: float
  minTargetDistance: units.millimeters
  maxTargetDistance: units.millimeters

@dataclass(frozen=True, slots=True)
class ButtonControllerConfig:
  name: str
  channel: int