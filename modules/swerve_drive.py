import math
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import SwerveDriveModuleConfig, IdleMode

class SwerveDriveModule:
  def __init__(
    self,
    config: SwerveDriveModuleConfig
  ) -> None:
    self._config = config

    drivingWheelFreeSpeedRps: float = ((config.constants.drivingFreeSpeed / 60) * (config.constants.wheelDiameter * math.pi)) / config.constants.drivingGearReduction
    drivingFeedForwardVelocityGain: float = 12.0 / drivingWheelFreeSpeedRps
    drivingPositionConversionFactor: float = (config.constants.wheelDiameter * math.pi) / config.constants.drivingGearReduction
    turningPositionConversionFactor: float = 2 * math.pi

    self._drivingController = utils.getSparkController(config.drivingId, config.constants.drivingControllerType, config.constants.drivingMotorType)
    drivingSparkConfig = SparkBaseConfig()
    (drivingSparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(config.constants.drivingCurrentLimit)
    )
    (drivingSparkConfig.encoder
      .positionConversionFactor(drivingPositionConversionFactor)
      .velocityConversionFactor(drivingPositionConversionFactor / 60.0)
    )
    (drivingSparkConfig.closedLoop
      .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(*config.constants.drivingControlPID)
      .outputRange(-1.0, 1.0)
      .feedForward.kV(drivingFeedForwardVelocityGain)
    )
    utils.configureSparkController(self._drivingController, drivingSparkConfig)
    self._drivingClosedLoopController = self._drivingController.getClosedLoopController()
    self._drivingEncoder = self._drivingController.getEncoder()
    self._drivingEncoder.setPosition(0)

    self._turningController = utils.getSparkController(config.turningId, SparkLowLevel.SparkModel.kSparkMax, SparkLowLevel.MotorType.kBrushless)
    turningSparkConfig = SparkBaseConfig()
    (turningSparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(config.constants.turningCurrentLimit)
    )
    (turningSparkConfig.absoluteEncoder
      .inverted(True)
      .positionConversionFactor(turningPositionConversionFactor)
      .velocityConversionFactor(turningPositionConversionFactor / 60.0)
      .apply(config.constants.turningEncoderConfig)
    )
    (turningSparkConfig.closedLoop
      .setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(*config.constants.turningControlPID)
      .outputRange(-1.0, 1.0)
      .positionWrappingEnabled(True)
      .positionWrappingInputRange(0, turningPositionConversionFactor)
    )
    utils.configureSparkController(self._turningController, turningSparkConfig)
    self._turningClosedLoopController = self._turningController.getClosedLoopController()
    self._turningEncoder = self._turningController.getAbsoluteEncoder()
    self._turningOffset = units.degreesToRadians(config.turningOffset)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setTargetState(self, targetState: SwerveModuleState) -> None:
    currentAngle = Rotation2d(self._turningEncoder.getPosition())
    targetState.angle += Rotation2d(self._turningOffset)
    targetState.optimize(currentAngle)
    targetState.cosineScale(currentAngle)
    self._drivingClosedLoopController.setSetpoint(targetState.speed, SparkBase.ControlType.kVelocity)
    self._turningClosedLoopController.setSetpoint(targetState.angle.radians(), SparkBase.ControlType.kPosition)

  def getState(self) -> SwerveModuleState:
    return SwerveModuleState(self._drivingEncoder.getVelocity(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))

  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(self._drivingEncoder.getPosition(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._drivingController, idleMode)
    utils.setIdleMode(self._turningController, idleMode)
    
  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Velocity', self._drivingEncoder.getVelocity())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Turning/Velocity', self._turningEncoder.getVelocity())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Current', self._drivingController.getOutputCurrent())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Turning/Current', self._turningController.getOutputCurrent())
