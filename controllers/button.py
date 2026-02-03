from commands2.button import Trigger
from wpilib import DigitalInput, SmartDashboard
from ..classes import ButtonControllerConfig, RobotState
from .. import logger, utils

class ButtonController():
  def __init__(
      self, 
      config: ButtonControllerConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Controllers/Button/{self._config.name}'

    self._digitalInput = DigitalInput(self._config.channel)
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def _isPressed(self) -> bool:
    return False if utils.getRobotState() != RobotState.Disabled else not self._digitalInput.get()
  
  def pressed(self) -> Trigger:
    return Trigger(lambda: self._isPressed())
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsPressed', self._isPressed())
