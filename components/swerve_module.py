import math
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ClosedLoopConfig
from ..classes import SwerveModuleConfig, MotorIdleMode
from .. import logger, utils

class SwerveModule:
  def __init__(
    self,
    config: SwerveModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/Drive/Modules/{self._config.location.name}'

    drivingMotorReduction: float = (self._config.constants.wheelBevelGearTeeth * self._config.constants.wheelSpurGearTeeth) / (self._config.constants.drivingMotorPinionTeeth * self._config.constants.wheelBevelPinionTeeth)
    driveWheelFreeSpeedRps: float = ((self._config.constants.drivingMotorFreeSpeed / 60) * (self._config.constants.wheelDiameter * math.pi)) / drivingMotorReduction
    drivingEncoderPositionConversionFactor: float = (self._config.constants.wheelDiameter * math.pi) / drivingMotorReduction
    turningEncoderPositionConversionFactor: float = 2 * math.pi

    if self._config.constants.drivingMotorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._drivingMotor = SparkFlex(self._config.drivingMotorCANId, self._config.constants.drivingMotorType)
    else: 
      self._drivingMotor = SparkMax(self._config.drivingMotorCANId, self._config.constants.drivingMotorType)
    self._drivingMotorConfig = SparkBaseConfig()
    (self._drivingMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.drivingMotorCurrentLimit))
    (self._drivingMotorConfig.encoder
      .positionConversionFactor(drivingEncoderPositionConversionFactor)
      .velocityConversionFactor(drivingEncoderPositionConversionFactor / 60.0))
    (self._drivingMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.constants.drivingMotorPID)
      .velocityFF(1 / driveWheelFreeSpeedRps)
      .outputRange(-1.0, 1.0))
    utils.setSparkConfig(
      self._drivingMotor.configure(
        self._drivingMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._drivingClosedLoopController = self._drivingMotor.getClosedLoopController()
    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingEncoder.setPosition(0)

    self._turningMotor = SparkMax(self._config.turningMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._turningMotorConfig = SparkBaseConfig()
    (self._turningMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.turningMotorCurrentLimit))
    (self._turningMotorConfig.absoluteEncoder
      .inverted(True)
      .positionConversionFactor(turningEncoderPositionConversionFactor)
      .velocityConversionFactor(turningEncoderPositionConversionFactor / 60.0))
    (self._turningMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
      .pid(*self._config.constants.turningMotorPID)
      .outputRange(-1.0, 1.0)
      .positionWrappingEnabled(True)
      .positionWrappingInputRange(0, turningEncoderPositionConversionFactor))
    utils.setSparkConfig(
      self._turningMotor.configure(
        self._turningMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._turningClosedLoopController = self._turningMotor.getClosedLoopController()
    self._turningEncoder = self._turningMotor.getAbsoluteEncoder()

    self._turningOffset = units.degreesToRadians(self._config.turningOffset)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setTargetState(self, targetState: SwerveModuleState) -> None:
    currentAngle = Rotation2d(self._turningEncoder.getPosition())
    targetState.angle += Rotation2d(self._turningOffset)
    targetState.optimize(currentAngle)
    targetState.cosineScale(currentAngle)
    self._drivingClosedLoopController.setReference(targetState.speed, SparkBase.ControlType.kVelocity)
    self._turningClosedLoopController.setReference(targetState.angle.radians(), SparkBase.ControlType.kPosition)

  def getState(self) -> SwerveModuleState:
    return SwerveModuleState(self._drivingEncoder.getVelocity(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))

  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(self._drivingEncoder.getPosition(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    idleMode = SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
    utils.setSparkConfig(self._drivingMotor.configure(SparkBaseConfig().setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    utils.setSparkConfig(self._turningMotor.configure(SparkBaseConfig().setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    
  def _updateTelemetry(self) -> None:
    # SmartDashboard.putNumber(f'{self._baseKey}/Driving/Current', self._drivingMotor.getOutputCurrent())
    pass
