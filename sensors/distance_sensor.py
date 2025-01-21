from ntcore import NetworkTableInstance, PubSubOptions
from wpilib import SmartDashboard
from wpimath import units
from .. import logger, utils

class DistanceSensor:
  def __init__(
      self, 
      sensorName: str,
      minTargetDistance: units.millimeters,
      maxTargetDistance: units.millimeters
    ) -> None:
    self._sensorName = sensorName
    self._minTargetDistance = minTargetDistance
    self._maxTargetDistance = maxTargetDistance
    self._baseKey = f'Robot/Sensors/Distance/{self._sensorName}'

    self._subscriber = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(f'{self._baseKey}/Value').subscribe(-1.0, PubSubOptions(periodic=0.01))

    self._isTriggered: bool = False

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getDistance(self) -> units.millimeters:
    return self._subscriber.get()

  def hasTarget(self) -> bool:
    hasTarget = utils.isValueInRange(self.getDistance(), self._minTargetDistance, self._maxTargetDistance)
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
