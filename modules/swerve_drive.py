import math
from wpimath import units, Rotation2d, SwerveModulePosition, SwerveModuleVelocity
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

    self._drivingController = utils.getSparkController(config.port, config.drivingId, config.constants.drivingControllerType, config.constants.drivingMotorType)
    drivingSparkConfig = SparkBaseConfig()
    (drivingSparkConfig
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .smart_current_limit(config.constants.drivingCurrentLimit)
    )
    (drivingSparkConfig.soft_limit
      .reverse_soft_limit_enabled(False)
      .forward_soft_limit_enabled(False)
    )
    (drivingSparkConfig.closed_loop
      .set_feedback_sensor(FeedbackSensor.PRIMARY_ENCODER)
      .pid(*config.constants.drivingControlPID)
      .output_range(-1.0, 1.0)
      .feed_forward.v(drivingFeedForwardVelocityGain)
    )
    utils.configureSparkController(self._drivingController, drivingSparkConfig)
    self._drivingClosedLoopController = self._drivingController.get_closed_loop_controller()
    self._drivingEncoder = self._drivingController.get_encoder()
    self._drivingEncoder.set_position(0)

    self._turningController = utils.getSparkController(config.port, config.turningId, SparkLowLevel.SparkModel.SPARK_MAX, SparkLowLevel.MotorType.BRUSHLESS)
    turningSparkConfig = SparkBaseConfig()
    (turningSparkConfig
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .smart_current_limit(config.constants.turningCurrentLimit)
    )
    (turningSparkConfig.absolute_encoder
      .inverted(True)
      .apply(config.constants.turningEncoderConfig)
    )
    (turningSparkConfig.soft_limit
      .reverse_soft_limit_enabled(False)
      .forward_soft_limit_enabled(False)
    )
    (turningSparkConfig.closed_loop
      .set_feedback_sensor(FeedbackSensor.ABSOLUTE_ENCODER)
      .pid(*config.constants.turningControlPID)
      .output_range(-1.0, 1.0)
      .position_wrapping_enabled(True)
    )
    utils.configureSparkController(self._turningController, turningSparkConfig)
    self._turningClosedLoopController = self._turningController.get_closed_loop_controller()
    self._turningEncoder = self._turningController.get_absolute_encoder()
    self._turningOffset = units.degrees_to_radians(config.turningOffset)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setTargetVelocity(self, targetVelocity: SwerveModuleVelocity) -> None:
    currentAngle = Rotation2d(self._turningEncoder.get_position().get())
    targetVelocity.angle += Rotation2d(self._turningOffset)
    targetVelocity.optimize(currentAngle)
    targetVelocity.cosine_scale(currentAngle)
    self._drivingClosedLoopController.set_setpoint(targetVelocity.velocity, SparkBase.ControlType.VELOCITY)
    self._turningClosedLoopController.set_setpoint(targetVelocity.angle.radians(), SparkBase.ControlType.POSITION)

  def getVelocity(self) -> SwerveModuleVelocity:
    return SwerveModuleVelocity(self._drivingEncoder.get_velocity().get(), Rotation2d(self._turningEncoder.get_position().get() - self._turningOffset))

  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(self._drivingEncoder.get_position().get(), Rotation2d(self._turningEncoder.get_position().get() - self._turningOffset))

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._drivingController, idleMode)
    utils.setIdleMode(self._turningController, idleMode)
    
  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Velocity', self._drivingEncoder.get_velocity().get())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Turning/Velocity', self._turningEncoder.get_velocity().get())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Current', self._drivingController.get_output_current().get())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Turning/Current', self._turningController.get_output_current().get())
