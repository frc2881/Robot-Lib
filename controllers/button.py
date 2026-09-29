from wpilib import DigitalInput
from commands2.button import Trigger
from .. import logger, telemetry, utils
from ..classes import ButtonControllerConfig, RobotState

class ButtonController():
  def __init__(
      self, 
      config: ButtonControllerConfig
    ) -> None:
    self._config = config

    self._digitalInput = DigitalInput(config.channel)
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def _isPressed(self) -> bool:
    return False if utils.getRobotState() != RobotState.Disabled else not self._digitalInput.get()
  
  def pressed(self) -> Trigger:
    return Trigger(lambda: self._isPressed())
  
  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/IsPressed', self._isPressed())
