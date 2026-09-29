from wpilib import DigitalInput
from .. import logger, telemetry, utils
from ..classes import BinarySensorConfig

class BinarySensor:
  def __init__(
      self, 
      config: BinarySensorConfig
    ) -> None:
    self._config = config

    self._digitalInput = DigitalInput(config.channel)

    self._isTriggered: bool = False
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def hasTarget(self) -> bool:
    hasTarget = not self._digitalInput.get()
    if hasTarget and not self._isTriggered:
      self._isTriggered = True
    return hasTarget
  
  def isTriggered(self) -> bool:
    return self._isTriggered

  def resetTrigger(self) -> None:
    self._isTriggered = False

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/HasTarget', self.hasTarget())
    telemetry.log(f'{self._config.telemetryName}/IsTriggered', self.isTriggered())
