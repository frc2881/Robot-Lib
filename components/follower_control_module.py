from rev import SparkBaseConfig
from .. import logger, telemetry, utils
from ..classes import FollowerControlModuleConfig, IdleMode

class FollowerControlModule:
  def __init__(
    self,
    config: FollowerControlModuleConfig
  ) -> None:
    self._config = config

    self._controller = utils.getSparkController(config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smartCurrentLimit(self._config.currentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake))
    sparkConfig.follow(self._config.leaderId, self._config.isInverted)
    utils.configureSparkController(self._controller, sparkConfig)
    self._encoder = self._controller.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())
