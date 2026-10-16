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
    utils.configureSparkController(self._controller, sparkConfig)
    self._encoder = self._controller.get_encoder()
    self._encoder.set_position(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set_throttle(speed)

  def getSpeed(self) -> units.percent:
    return self._controller.get_throttle()

  def getOutputCurrent(self) -> units.amperes:
    return self._controller.get_output_current().get()

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stop_motor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Speed', self.getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.get_velocity().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())
