from typing import NamedTuple
from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout
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

class MotorControllerType(Enum):
  SparkMax = auto()
  SparkFlex = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

class LockState(Enum):
  Unlocked = auto()
  Locked = auto()

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

class SwerveModuleLocation(IntEnum):
  FrontLeft = 0,
  FrontRight = 1,
  RearLeft = 2,
  RearRight = 3

@dataclass(frozen=True, slots=True)
class SwerveModuleConstants:
  wheelDiameter: units.meters
  wheelBevelGearTeeth: int
  wheelSpurGearTeeth: int
  wheelBevelPinionTeeth: int
  drivingMotorPinionTeeth: int
  drivingMotorFreeSpeed: units.revolutions_per_minute
  drivingMotorControllerType: MotorControllerType
  drivingMotorCurrentLimit: int
  drivingMotorPID: PID
  turningMotorCurrentLimit: int
  turningMotorPID: PID

@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
  location: SwerveModuleLocation
  drivingMotorCANId: int
  turningMotorCANId: int
  turningOffset: units.radians
  translation: Translation2d
  constants: SwerveModuleConstants

class DifferentialModuleLocation(IntEnum):
  LeftFront = 0,
  LeftCenter = 1,
  LeftRear = 2,
  RightFront = 3,
  RightCenter = 4,
  RightRear = 5

@dataclass(frozen=True, slots=True)
class DifferentialModuleConstants:
  wheelDiameter: units.meters
  drivingMotorControllerType: MotorControllerType
  drivingMotorCurrentLimit: int
  drivingMotorReduction: float

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

class PoseSensorLocation(Enum):
  Front = auto(),
  Rear = auto(),
  Left = auto(),
  Right = auto()

@dataclass(frozen=True, slots=True)
class PoseSensorConfig:
  location: PoseSensorLocation
  cameraTransform: Transform3d
  poseStrategy: PoseStrategy
  fallbackPoseStrategy: PoseStrategy
  aprilTagFieldLayout: AprilTagFieldLayout

@dataclass(frozen=True, slots=True)
class LeadscrewModuleConstants:
  leadscrewTravelDistance: units.inches
  motorControllerType: MotorControllerType
  motorCurrentLimit: int
  motorReduction: float
  motorPID: PID
  motorMotionMaxVelocityRate: units.percent
  motorMotionMaxAccelerationRate: units.percent
  allowedClosedLoopError: float
  motorSoftLimitForward: float
  motorSoftLimitReverse: float
  motorResetSpeed: units.percent

@dataclass(frozen=True, slots=True)
class LeadscrewModuleConfig:
  moduleBaseKey: str
  motorCANId: int
  constants: LeadscrewModuleConstants

@dataclass(frozen=False, slots=True)
class TargetInfo:
  distance: units.meters
  heading: units.degrees
  pitch: units.degrees
