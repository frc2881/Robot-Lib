from wpimath import units
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import VelocityControlModuleConfig, IdleMode

class VelocityControlModule:
  def __init__(
    self,
    config: VelocityControlModuleConfig
  ) -> None:
    self._config = config

    self._targetSpeed: float = 0

    self._controller = utils.getSparkController(config.port, config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smart_current_limit(self._config.currentLimit)
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .inverted(self._config.isInverted)
    )
    (sparkConfig.soft_limit
      .reverse_soft_limit_enabled(False)
      .forward_soft_limit_enabled(False)
    )
    (sparkConfig.closed_loop
      .set_feedback_sensor(FeedbackSensor.PRIMARY_ENCODER)
      .pid(*self._config.controlPID)
      .output_range(-1.0, 1.0)
      .feed_forward
        .s(self._config.feedForwardGains.static)
        .v(self._config.feedForwardGains.velocity)
        .a(self._config.feedForwardGains.acceleration)
        .g(self._config.feedForwardGains.gravity)
    )
    (sparkConfig.closed_loop.max_motion
      .cruise_velocity(self._config.cruiseVelocity)
      .max_acceleration(self._config.maxAcceleration)
      .allowed_profile_error(self._config.allowedProfileError)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._closedLoopController = self._controller.get_closed_loop_controller()
    self._encoder = self._controller.get_encoder()
    self._encoder.set_position(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._targetSpeed = speed
    self._closedLoopController.set_setpoint(self._config.cruiseVelocity * speed, SparkBase.ControlType.MAX_MOTION_VELOCITY_CONTROL)

  def getSpeed(self) -> units.percent:
    return self._encoder.get_velocity().get() / self._config.cruiseVelocity

  def getTargetSpeed(self) -> units.percent:
    return self._targetSpeed

  def isAtTargetSpeed(self) -> bool:
    return self._targetSpeed != 0 and utils.isValueWithinTolerance(self.getSpeed(), self._targetSpeed, 0.05)
  
  def _resetTargetSpeed(self) -> None:
    self._targetSpeed = 0

  def getOutputCurrent(self) -> units.amperes:
    return self._controller.get_output_current().get()

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stop_motor()
    self._resetTargetSpeed()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Speed', self.getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.get_velocity().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())

