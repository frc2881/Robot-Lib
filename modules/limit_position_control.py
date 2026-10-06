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

    self._controller = utils.getSparkController(config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smartCurrentLimit(self._config.currentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._config.isInverted)
    )
    (sparkConfig.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0)
    )
    (sparkConfig.softLimit
      .reverseSoftLimitEnabled(False)
      .forwardSoftLimitEnabled(False)
    )
    (sparkConfig.limitSwitch
      .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
      .reverseLimitSwitchEnabled(True)
      .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
      .forwardLimitSwitchEnabled(True)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._forwardLimitSwitch = self._controller.getForwardLimitSwitch()
    self._reverseLimitSwitch = self._controller.getReverseLimitSwitch()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: Position) -> None:
    self._targetPosition = position
    match self._targetPosition:
      case Position.FORWARD:
        self._controller.set(self._config.outputRange.max)
      case Position.BACKWARD:
        self._controller.set(self._config.outputRange.min)
      case _:
        self._controller.stopMotor()
    
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
    self._controller.stopMotor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self.getPosition().name)
    telemetry.log(f'{self._config.telemetryName}/LimitSwitchForward', self._forwardLimitSwitch.get())
    telemetry.log(f'{self._config.telemetryName}/LimitSwitchReverse', self._reverseLimitSwitch.get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())
