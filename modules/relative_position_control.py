from wpimath import units
from commands2 import Command, cmd, Subsystem
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import RelativePositionControlModuleConfig, MotorDirection, IdleMode, RobotState, Value

class RelativePositionControlModule:
  def __init__(
    self,
    config: RelativePositionControlModuleConfig
  ) -> None:
    self._config = config

    self._isHoming: bool = False
    self._isHomed: bool = False
    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    self._controller = utils.getSparkController(config.port, config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smart_current_limit(config.currentLimit)
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
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
    self._encoder = self._controller.get_encoder()
    self._encoder.set_position(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set_throttle(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetTargetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.softLimitReverse
    if position == Value.max: position = self._config.softLimitForward
    if position != self._targetPosition:
      self._resetTargetPosition()
      self._targetPosition = position
    self._closedLoopController.set_setpoint(self._targetPosition, SparkBase.ControlType.MAX_MOTION_POSITION_CONTROL)
    self._isAtTargetPosition = utils.isValueWithinTolerance(self.getPosition(), self._targetPosition, self._config.allowedProfileError)

  def getPosition(self) -> float:
    return self._encoder.get_position().get()

  def getTargetPosition(self) -> float:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def _resetTargetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtSoftLimit(self, direction: MotorDirection, tolerance: float) -> bool:
    return utils.isValueWithinTolerance(
      self.getPosition(),
      self._config.softLimitReverse if direction == MotorDirection.REVERSE else self._config.softLimitForward, 
      tolerance
    )

  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSoftLimitsEnabled(self._controller, isEnabled)

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def resetToHome(self, subsystem: Subsystem, defaultIdleMode: IdleMode = IdleMode.BRAKE) -> Command:
    return cmd.start_end(
      lambda: self._startHoming(),
      lambda: self._endHoming(defaultIdleMode),
      subsystem
    )
  
  def _startHoming(self) -> None:
    self._isHomed = False
    self._isHoming = True
    utils.setSoftLimitsEnabled(self._controller, False)
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.set_throttle(-self._config.homingSpeed)
    else:
      self.setIdleMode(IdleMode.COAST)

  def _endHoming(self, defaultIdleMode: IdleMode) -> None:
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.stop_motor()
    else:
      self.setIdleMode(defaultIdleMode)
    self._encoder.set_position(self._config.homingPosition)
    utils.setSoftLimitsEnabled(self._controller, True)
    self._isHomed = True
    self._isHoming = False
  
  def isHoming(self) -> bool:
    return self._isHoming

  def isHomed(self) -> bool:
    return self._isHomed

  def reset(self) -> None:
    self._controller.stop_motor()
    self._resetTargetPosition()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._encoder.get_position().get())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.get_velocity().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())
