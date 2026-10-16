import math
from rev import SparkBase, SparkBaseConfig
from .. import logger, telemetry, utils
from ..classes import DifferentialDriveModuleConfig, IdleMode

class DifferentialDriveModule:
  def __init__(
    self,
    config: DifferentialDriveModuleConfig
  ) -> None:
    self._config = config

    drivingPositionConversionFactor: float = (config.constants.wheelDiameter * math.pi) / config.constants.gearReduction

    self._controller = utils.getSparkController(config.port, config.id, config.constants.controllerType, config.constants.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smart_current_limit(config.constants.currentLimit)
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .inverted(config.isInverted)
    )
    (sparkConfig.soft_limit
      .reverse_soft_limit_enabled(False)
      .forward_soft_limit_enabled(False)
    )
    if config.leaderId is not None:
      sparkConfig.follow(config.leaderId)
    utils.configureSparkController(self._controller, sparkConfig)
    self._encoder = self._controller.get_encoder()
    self._encoder.set_position(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getController(self) -> SparkBase:
    return self._controller

  def getPosition(self) -> float:
    return self._encoder.get_position().get()
  
  def getVelocity(self) -> float:
    return self._encoder.get_velocity().get()
  
  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Velocity', self._encoder.get_velocity().get())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Current', self._controller.get_output_current().get())
