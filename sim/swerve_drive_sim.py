import math
import random
import wpimath
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim
from wpimath.system.plant import DCMotor
from wpimath.filter import SlewRateLimiter
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import RobotController
from ntcore import NetworkTableInstance

from lib.components.swerve_module import SwerveModule

# Simulation constants
# 5G acceleration
kdrivingMotorSimMaxAcceleration: float = 9.8 * 5
# The P value for the turning PID controller. rad/s
kturningMotorSimP: float = 31.0
# The D value for the turning PID controller.
kturningMotorSimD: float = 0.0

def clamp(val: float, a: float, b: float) -> float:
  return max(a, min(b, val))


def clampPose(pose: Pose2d) -> Pose2d:
  translation: Translation2d = pose.translation()
  translation = Translation2d(
    clamp(translation.X(), 0, 16.49),
    clamp(translation.Y(), 0, 8.10)
  )
  return Pose2d(translation, pose.rotation())

def _createSim(motor: SparkFlex | SparkMax) -> SparkFlexSim | SparkMaxSim:
  match motor:
    case SparkFlex():
      return SparkFlexSim(motor, DCMotor.NEO(1))
    case SparkMax():
      return SparkMaxSim(motor, DCMotor.NEO(1))

class SwerveDriveSim:
  class SwerveModuleSim:
    def __init__(self, module: SwerveModule):
      self._driveSparkSim = _createSim(module._drivingMotor)
      self._turnSparkSim = _createSim(module._turningMotor)
      self._angularOffset = module._config.turningOffset

      # Calculates the velocity of the drive motor, simulating inertia and friction
      self._driveRateLimiter = SlewRateLimiter(kdrivingMotorSimMaxAcceleration)

      # Calculates the velocity of the turn motor
      self._turnController = PIDController(
        kturningMotorSimP,
        0,
        kturningMotorSimD
      )
      self._turnController.enableContinuousInput(-math.pi, math.pi)

      # Randomize starting rotation
      self._turnSparkSim.setPosition(random.uniform(-math.pi, math.pi))

    def simulationPeriodic(self, vbus: float, dt: float):
      # m/s
      targetVelocity = self._driveSparkSim.getSetpoint()
      driveVelocity = self._driveRateLimiter.calculate(targetVelocity)
      self._driveSparkSim.iterate(
        driveVelocity,
        vbus,
        dt
      )

      # rad
      targetAngle = self._turnSparkSim.getSetpoint()
      curAngle = wpimath.angleModulus(self._turnSparkSim.getPosition())
      self._turnController.setSetpoint(targetAngle)
      turnVelocity = self._turnController.calculate(curAngle)
      self._turnSparkSim.iterate(
        turnVelocity,
        vbus,
        dt
      )

    def getState(self) -> SwerveModuleState:
      return SwerveModuleState(
        self._driveSparkSim.getVelocity(),
        Rotation2d(self._turnSparkSim.getPosition() - self._angularOffset)
      )

  def __init__(self, kinematics: SwerveDrive4Kinematics, modules: tuple[SwerveModule, ...]):
    self._kinematics = kinematics
    self._modules = tuple(
      SwerveDriveSim.SwerveModuleSim(module) for module in modules
    )
    self._statePublish = NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Drive/Modules/State", SwerveModuleState).publish()

  def simulationPeriodic(self, dt: float) -> ChassisSpeeds:
    vBus = RobotController.getBatteryVoltage()
    for module in self._modules:
      module.simulationPeriodic(vBus, dt)
    moduleStates = tuple(module.getState() for module in self._modules)
    self._statePublish.set(moduleStates)
    chassisSpeeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(moduleStates)
    return chassisSpeeds
