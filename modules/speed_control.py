from wpimath import units
from rev import SparkBaseConfig
from .. import logger, telemetry, utils
from ..classes import SpeedControlModuleConfig, IdleMode

class SpeedControlModule:
  def __init__(
    self,
    config: SpeedControlModuleConfig
  ) -> None:
    self._config = config

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
    utils.configureSparkController(self._controller, sparkConfig)
    self._encoder = self._controller.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set(speed)

  def getSpeed(self) -> units.percent:
    return self._controller.get()

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stopMotor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Speed', self.getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())
