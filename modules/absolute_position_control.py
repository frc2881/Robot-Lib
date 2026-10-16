from wpimath import units
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import AbsolutePositionControlModuleConfig, MotorDirection, IdleMode, Value

class AbsolutePositionControlModule:
  def __init__(
    self,
    config: AbsolutePositionControlModuleConfig
  ) -> None:
    self._config = config

    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    self._controller = utils.getSparkController(config.port, config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smart_current_limit(config.currentLimit)
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .inverted(config.isInverted)
    )
    (sparkConfig.absolute_encoder
      .inverted(config.isInverted)
    )
    (sparkConfig.soft_limit
      .reverse_soft_limit(config.softLimitReverse)
      .reverse_soft_limit_enabled(True)
      .forward_soft_limit(config.softLimitForward)
      .forward_soft_limit_enabled(True)
    )
    (sparkConfig.closed_loop
      .set_feedback_sensor(FeedbackSensor.PRIMARY_ENCODER)
      .pid(*config.controlPID)
      .output_range(*config.outputRange)
      .feed_forward
        .s(config.feedForwardGains.static)
        .v(config.feedForwardGains.velocity)
        .a(config.feedForwardGains.acceleration)
        .g(config.feedForwardGains.gravity)
    )
    (sparkConfig.closed_loop.max_motion
      .cruise_velocity(config.cruiseVelocity)
      .max_acceleration(config.maxAcceleration)
      .allowed_profile_error(config.allowedProfileError)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._closedLoopController = self._controller.get_closed_loop_controller()
    self._absoluteEncoder = self._controller.get_absolute_encoder()
    self._relativeEncoder = self._controller.get_encoder()
    self._relativeEncoder.set_position(self._absoluteEncoder.get_position().get())

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set_throttle(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.softLimitReverse
    if position == Value.max: position = self._config.softLimitForward
    if position != self._targetPosition:
      self._resetPosition()
      self._targetPosition = position
    self._closedLoopController.set_setpoint(self._targetPosition, SparkBase.ControlType.MAX_MOTION_POSITION_CONTROL)
    self._isAtTargetPosition = utils.isValueWithinTolerance(self.getPosition(), self._targetPosition, self._config.allowedProfileError)

  def getPosition(self) -> float:
    return self._relativeEncoder.get_position().get()

  def getTargetPosition(self) -> float:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def _resetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtSoftLimit(self, direction: MotorDirection) -> bool:
    return (
      self._controller.get_reverse_soft_limit().is_reached()
      if direction == MotorDirection.REVERSE else
      self._controller.get_forward_soft_limit().is_reached()
    )
  
  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSoftLimitsEnabled(self._controller, isEnabled)

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stop_motor()
    self._relativeEncoder.set_position(self._absoluteEncoder.get_position().get())
    self._resetPosition()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._absoluteEncoder.get_position().get())
    telemetry.log(f'{self._config.telemetryName}/RelativePosition', self._relativeEncoder.get_position().get())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._relativeEncoder.get_velocity().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())
