from wpilib import DigitalInput, SmartDashboard
from .. import logger, utils

class BeamBreakSensor:
  def __init__(
      self, 
      sensorName: str,
      channel: int
    ) -> None:
    self._sensorName = sensorName
    self._baseKey = f'Robot/Sensors/BeamBreak/{self._sensorName}'

    self._digitalInput = DigitalInput(channel)

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
