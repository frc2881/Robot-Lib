from wpilib import DigitalInput, SmartDashboard
from ..classes import BinarySensorConfig
from .. import logger, utils

class BinarySensor:
  def __init__(
      self, 
      config: BinarySensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Binary/{self._config.sensorName}'

    self._digitalInput = DigitalInput(self._config.digitalInputChannel)

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
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
    SmartDashboard.putBoolean(f'{self._baseKey}/IsTriggered', self.isTriggered())
