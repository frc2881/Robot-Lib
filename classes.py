from typing import NamedTuple, Optional
import sys, math
from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform2d, Transform3d, Pose3d
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
  Forward = auto()
  Backward = auto()

class MotorDirection(Enum):
  Forward = auto()
  Reverse = auto()
  Stop = auto()

class IdleMode(Enum):
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

@dataclass(frozen=True, slots=True)
class Value(float):
  none = math.nan
  min = sys.float_info.min
  max = sys.float_info.max

class Range(NamedTuple):
  min: float
  max: float

class PID(NamedTuple):
  P: float = 0
  I: float = 0
  D: float = 0

class FeedForwardGains(NamedTuple):
  static: units.volts = 0
  velocity: units.volts = 0
  acceleration: units.volts = 0
  gravity: units.volts = 0

class MotorModel(Enum):
  NEO = auto()
  NEOVortex = auto()
  NEO550 = auto()

class SwerveDriveModuleGearKit(Enum):
  Low = auto()
  Medium = auto()
  High = auto()
  ExtraHigh1 = auto()
  ExtraHigh2 = auto()
  ExtraHigh3 = auto()
  ExtraHigh4 = auto()
  ExtraHigh5 = auto()

class SwerveDriveModuleLocation(IntEnum):
  FrontLeft = 0
  FrontRight = 1
  RearLeft = 2
  RearRight = 3

@dataclass(frozen=True, slots=True)
class SwerveDriveModuleConfig:
  location: SwerveDriveModuleLocation
  drivingId: int
  turningId: int
  turningOffset: units.degrees
  chassisTranslation: Translation2d
  constants: SwerveDriveModuleConfigConstants

@dataclass(frozen=True, slots=True)
class SwerveDriveModuleConfigConstants:
  drivingControllerType: SparkLowLevel.SparkModel
  drivingMotorType: SparkLowLevel.MotorType
  drivingFreeSpeed: units.revolutions_per_minute
  drivingGearReduction: float
  drivingCurrentLimit: int
  drivingControlPID: PID
  turningCurrentLimit: int
  turningControlPID: PID
  turningEncoderConfig: AbsoluteEncoderConfig
  wheelDiameter: units.meters
  telemetryName: str

class DifferentialDriveModuleLocation(IntEnum):
  Left = 0
  Right = 1

class DifferentialDriveModulePositions(NamedTuple):
  left: units.meters
  right: units.meters

@dataclass(frozen=True, slots=True)
class DifferentialDriveModuleConfig:
  location: DifferentialDriveModuleLocation
  id: int
  leaderId: Optional[int]
  isInverted: bool
  constants: DifferentialDriveModuleConfigConstants

@dataclass(frozen=True, slots=True)
class DifferentialDriveModuleConfigConstants:
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  gearReduction: float
  currentLimit: int
  wheelDiameter: units.meters
  telemetryName: str

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConfig:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  softLimitReverse: float
  softLimitForward: float
  controlPID: PID
  outputRange: Range
  feedForwardGains: FeedForwardGains
  cruiseVelocity: units.revolutions_per_minute
  maxAcceleration: units.units_per_second
  allowedProfileError: float
  homingPosition: float
  homingSpeed: units.percent
  positionConversionFactor: float
  telemetryName: str

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConfig:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  softLimitReverse: float
  softLimitForward: float
  controlPID: PID
  outputRange: Range
  feedForwardGains: FeedForwardGains
  cruiseVelocity: units.revolutions_per_minute
  maxAcceleration: units.units_per_second
  allowedProfileError: float
  relativePositionConversionFactor: float
  absolutePositionConversionFactor: float
  telemetryName: str

@dataclass(frozen=True, slots=True)
class LimitPositionControlModuleConfig:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  outputRange: Range
  telemetryName: str

@dataclass(frozen=True, slots=True)
class VelocityControlModuleConfig:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  controlPID: PID
  outputRange: Range
  feedForwardGains: FeedForwardGains
  cruiseVelocity: units.revolutions_per_minute
  maxAcceleration: units.units_per_second
  allowedProfileError: float
  telemetryName: str

@dataclass(frozen=True, slots=True)
class SpeedControlModuleConfig:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  telemetryName: str

@dataclass(frozen=True, slots=True)
class FollowerControlModuleConfig:
  id: int
  leaderId: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  telemetryName: str

@dataclass(frozen=True, slots=True)
class XboxControllerConfig:
  port: int
  inputDeadband: units.percent
  telemetryName: str

@dataclass(frozen=True, slots=True)
class ButtonControllerConfig:
  channel: int
  telemetryName: str

@dataclass(frozen=True, slots=True)
class BinarySensorConfig:
  channel: int
  telemetryName: str

@dataclass(frozen=True, slots=True)
class DistanceSensorConfig:
  channel: int
  pulseWidthConversionFactor: float
  minTargetDistance: units.millimeters
  maxTargetDistance: units.millimeters
  telemetryName: str

@dataclass(frozen=True, slots=True)
class PoseSensorConfig:
  cameraName: str
  transform: Transform3d
  stream: str
  aprilTagFieldLayout: AprilTagFieldLayout
  telemetryName: str

class PoseSensorResultType(Enum):
  SINGLE_TAG = auto()
  MULTI_TAG = auto()

@dataclass(frozen=True, slots=True)
class PoseSensorResult:
  timestamp: units.seconds
  estimatedPose: Pose3d
  resultType: PoseSensorResultType
  bestTargetReprojectionError: float
  bestTargetAmbiguity: units.percent
  bestTargetDistance: units.meters

@dataclass(frozen=True, slots=True)
class ObjectSensorConfig:
  cameraName: str
  transform: Transform3d
  stream: str
  objectHeight: units.meters
  telemetryName: str

@dataclass(frozen=True, slots=True)
class Objects:
  transform: Transform2d
  count: int

@dataclass(frozen=True, slots=True)
class HeadingAlignmentConstants:
  rotationControlPID: PID
  rotationPositionTolerance: units.degrees

@dataclass(frozen=True, slots=True)
class PoseAlignmentConstants:
  translationControlPID: PID
  translationMaxVelocity: units.meters_per_second
  translationPositionTolerance: units.meters
  rotationControlPID: PID
  rotationMaxVelocity: units.degrees_per_second
  rotationPositionTolerance: units.degrees