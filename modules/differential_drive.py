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

    self._controller = utils.getSparkController(config.id, config.constants.controllerType, config.constants.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smartCurrentLimit(config.constants.currentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(config.isInverted)
    )
    (sparkConfig.encoder
      .positionConversionFactor(drivingPositionConversionFactor)
      .velocityConversionFactor(drivingPositionConversionFactor / 60.0)
    )
    if config.leaderId is not None:
      sparkConfig.follow(config.leaderId)
    utils.configureSparkController(self._controller, sparkConfig)
    self._encoder = self._controller.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getController(self) -> SparkBase:
    return self._controller

  def getPosition(self) -> float:
    return self._encoder.getPosition()
  
  def getVelocity(self) -> float:
    return self._encoder.getVelocity()
  
  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.constants.telemetryName}/{self._config.location.name}/Driving/Current', self._controller.getOutputCurrent())
