import math
from wpilib import DigitalInput, DutyCycle, SmartDashboard
from wpimath import units
from ..classes import DistanceSensorConfig
from .. import logger, utils

class DistanceSensor:
  def __init__(
      self, 
      config: DistanceSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Distance/{self._config.name}'

    self._dutycycle = DutyCycle(DigitalInput(self._config.digitalInputChannel))

    self._isTriggered: bool = False

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getDistance(self) -> units.millimeters:
    time = self._dutycycle.getHighTime() * 1000000
    distance = -1
    if time > 0 and time < 1850:
      distance = (time - 1000) * self._config.pulseWidthConversionFactor
      if distance < 0: distance = 0
    return math.trunc(distance)

  def hasTarget(self) -> bool:
    hasTarget = utils.isValueInRange(self.getDistance(), self._config.minTargetDistance, self._config.maxTargetDistance)
    if hasTarget and not self._isTriggered:
      self._isTriggered = True
    return hasTarget
  
  def isTriggered(self) -> bool:
    return self._isTriggered

  def resetTrigger(self) -> None:
    self._isTriggered = False

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Value', self.getDistance())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
    SmartDashboard.putBoolean(f'{self._baseKey}/IsTriggered', self.isTriggered())
