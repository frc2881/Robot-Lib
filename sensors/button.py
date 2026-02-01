from wpilib import DigitalInput, SmartDashboard
from ..classes import ButtonSensorConfig
from .. import logger, utils

class ButtonSensor:
  def __init__(
      self, 
      config: ButtonSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Button/{self._config.name}'

    self._digitalInput = DigitalInput(self._config.digitalInputChannel)
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
  
  def isPressed(self) -> bool:
    return not self._digitalInput.get() if self._config.isInverted else self._digitalInput.get()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsPressed', self.isPressed())
