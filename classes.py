from typing import NamedTuple
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

class OptionState(Enum):
  Enabled = auto()
  Disabled = auto()

class MotorDirection(Enum):
  Forward = auto()
  Reverse = auto()
  Stopped = auto()

class MotorIdleMode(Enum):
  Brake = auto()
  Coast = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

class LockState(Enum):
  Unlocked = auto()
  Locked = auto()

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

class Tolerance(NamedTuple):
  error: float
  errorDerivative: float

@dataclass(frozen=True, slots=True)
class DriftCorrectionConstants:
  rotationPID: PID
  rotationTolerance: Tolerance

@dataclass(frozen=True, slots=True)
class TargetAlignmentConstants:
  rotationPID: PID
  rotationTolerance: Tolerance
  rotationSpeedMax: units.radians_per_second
  rotationHeadingModeOffset: units.degrees
  rotationTranslationModeOffset: units.degrees
  translationPID: PID
  translationTolerance: Tolerance
  translationSpeedMax: units.meters_per_second

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
  turningOffset: units.radians
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
  left: float
  right: float

@dataclass(frozen=True, slots=True)
class PositionControlModuleConstants:
  motorTravelDistance: units.inches
  motorControllerType: SparkLowLevel.SparkModel
  motorType: SparkLowLevel.MotorType
  motorCurrentLimit: int
  motorReduction: float
  motorPID: PID
  motorMotionMaxVelocityRate: units.percent
  motorMotionMaxAccelerationRate: units.percent
  allowedClosedLoopError: float
  motorSoftLimitForward: units.inches
  motorSoftLimitReverse: units.inches
  motorResetSpeed: units.percent

@dataclass(frozen=True, slots=True)
class PositionControlModuleConfig:
  moduleBaseKey: str
  motorCANId: int
  leaderMotorCANId: int | None
  constants: PositionControlModuleConstants

@dataclass(frozen=True, slots=True)
class PoseSensorConfig:
  cameraName: str
  cameraTransform: Transform3d
  poseStrategy: PoseStrategy
  fallbackPoseStrategy: PoseStrategy
  aprilTagFieldLayout: AprilTagFieldLayout

@dataclass(frozen=True, slots=True)
class ObjectSensorConfig:
  cameraName: str
  cameraTransform: Transform3d
