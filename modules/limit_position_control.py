from rev import SparkBaseConfig, LimitSwitchConfig
from .. import logger, telemetry, utils
from ..classes import LimitPositionControlModuleConfig, IdleMode, Position

class LimitPositionControlModule:
  def __init__(
    self,
    config: LimitPositionControlModuleConfig
  ) -> None:
    self._config = config

    self._targetPosition: Position = Position.UNKNOWN

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
    (sparkConfig.limit_switch
      .reverse_limit_switch_type(LimitSwitchConfig.Type.NORMALLY_CLOSED)
      .reverse_limit_switch_trigger_behavior(LimitSwitchConfig.Behavior.STOP_MOVING_MOTOR)
      .forward_limit_switch_type(LimitSwitchConfig.Type.NORMALLY_CLOSED)
      .forward_limit_switch_trigger_behavior(LimitSwitchConfig.Behavior.STOP_MOVING_MOTOR)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._forwardLimitSwitch = self._controller.get_forward_limit_switch()
    self._reverseLimitSwitch = self._controller.get_reverse_limit_switch()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: Position) -> None:
    self._targetPosition = position
    match self._targetPosition:
      case Position.FORWARD:
        self._controller.set_throttle(self._config.outputRange.max)
      case Position.BACKWARD:
        self._controller.set_throttle(self._config.outputRange.min)
      case _:
        self._controller.stop_motor()
    
  def getPosition(self) -> Position:
    if self._forwardLimitSwitch.get(): return Position.FORWARD
    if self._reverseLimitSwitch.get(): return Position.BACKWARD
    return Position.UNKNOWN

  def getTargetPosition(self) -> Position:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    position = self.getPosition()
    return position != Position.UNKNOWN and position == self._targetPosition

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stop_motor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self.getPosition().name)
    telemetry.log(f'{self._config.telemetryName}/LimitSwitch/Forward', self._forwardLimitSwitch.get().get())
    telemetry.log(f'{self._config.telemetryName}/LimitSwitch/Reverse', self._reverseLimitSwitch.get().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())
