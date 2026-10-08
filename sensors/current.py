from typing import Callable
from wpimath import units
from wpimath.filter import Debouncer
from .. import logger, telemetry, utils
from ..classes import CurrentSensorConfig

class CurrentSensor:
  def __init__(
      self,
      config: CurrentSensorConfig,
      getCurrent: Callable[[], units.amperes]
    ) -> None:
    self._config = config
    self._getCurrent = getCurrent

    self._debouncer = Debouncer(self._config.changeTime, Debouncer.DebounceType.kRising)

    self._hasTarget: bool = False
    self._isTriggered: bool = False

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._hasTarget = self._debouncer.calculate(self._getCurrent() >= self._config.targetCurrent)
    self._updateTelemetry()

  def hasTarget(self) -> bool:
    if self._hasTarget and not self._isTriggered:
      self._isTriggered = True
    return self._hasTarget
  
  def isTriggered(self) -> bool:
    return self._isTriggered

  def resetTrigger(self) -> None:
    self._isTriggered = False

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Value', self._getCurrent())
    telemetry.log(f'{self._config.telemetryName}/HasTarget', self.hasTarget())
    telemetry.log(f'{self._config.telemetryName}/IsTriggered', self.isTriggered())
