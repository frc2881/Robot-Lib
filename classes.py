from typing import NamedTuple, Optional
import sys, math
from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform2d, Transform3d, Pose3d
from robotpy_apriltag import AprilTagFieldLayout
from rev import SparkLowLevel, AbsoluteEncoderConfig

class Alliance(IntEnum):
  RED = 0
  BLUE = 1

class RobotMode(Enum):
  DISABLED = auto()
  AUTO = auto()
  TELEOP = auto()
  TEST = auto()

class RobotState(Enum):
  DISABLED = auto()
  ENABLED = auto()
  ESTOPPED = auto()

class RobotType(Enum):
  OTHER = auto()
  COMPETITION = auto()
  PRACTICE = auto()
  DEMO = auto()

class State(Enum):
  DISABLED = auto()
  ENABLED = auto()
  STOPPED = auto()
  RUNNING = auto()
  COMPLETED = auto()

class Position(Enum):
  UNKNOWN = auto()
  DEFAULT = auto()
  UP = auto()
  DOWN = auto()
  LEFT = auto()
  RIGHT = auto()
  FRONT = auto()
  REAR = auto()
  TOP = auto()
  BOTTOM = auto()
  CENTER = auto()
  OPEN = auto()
  CLOSED = auto()
  IN = auto()
  OUT = auto()
  UNLOCKED = auto()
  LOCKED = auto()
  FORWARD = auto()
  BACKWARD = auto()

class MotorDirection(Enum):
  FORWARD = auto()
  REVERSE = auto()
  STOP = auto()

class IdleMode(Enum):
  BRAKE = auto()
  COAST = auto()

class DriveOrientation(Enum):
  FIELD = auto()
  ROBOT = auto()

class SpeedMode(Enum):
  COMPETITION = auto()
  DEMO = auto()

class ControllerRumbleMode(Enum):
  BOTH = auto()
  DRIVER = auto()
  OPERATOR = auto()

class ControllerRumblePattern(Enum):
  SHORT = auto()
  LONG = auto()

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
  NEO_VORTEX = auto()
  NEO_550 = auto()

class SwerveDriveModuleGearKit(Enum):
  LOW = auto()
  MEDIUM = auto()
  HIGH = auto()
  EXTRA_HIGH_1 = auto()
  EXTRA_HIGH_2 = auto()
  EXTRA_HIGH_3 = auto()
  EXTRA_HIGH_4 = auto()
  EXTRA_HIGH_5 = auto()

class SwerveDriveModuleLocation(IntEnum):
  FRONT_LEFT = 0
  FRONT_RIGHT = 1
  REAR_LEFT = 2
  REAR_RIGHT = 3

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
  LEFT = 0
  RIGHT = 1

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
class ControlModuleConfigBase:
  id: int
  controllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  currentLimit: int
  isInverted: bool
  telemetryName: str

@dataclass(frozen=True, slots=True)
class RelativePositionControlModuleConfig(ControlModuleConfigBase):
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

@dataclass(frozen=True, slots=True)
class AbsolutePositionControlModuleConfig(ControlModuleConfigBase):
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

@dataclass(frozen=True, slots=True)
class LimitPositionControlModuleConfig(ControlModuleConfigBase):
  outputRange: Range

@dataclass(frozen=True, slots=True)
class VelocityControlModuleConfig(ControlModuleConfigBase):
  controlPID: PID
  outputRange: Range
  feedForwardGains: FeedForwardGains
  cruiseVelocity: units.revolutions_per_minute
  maxAcceleration: units.units_per_second
  allowedProfileError: float

@dataclass(frozen=True, slots=True)
class SpeedControlModuleConfig(ControlModuleConfigBase):
  pass

@dataclass(frozen=True, slots=True)
class FollowerControlModuleConfig(ControlModuleConfigBase):
  leaderId: int

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