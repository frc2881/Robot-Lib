from enum import Enum, auto
from .. import logger, telemetry, utils

class LightsMode(Enum):
  Default = auto()

class LightsController:
  def __init__(self) -> None:
    self.setMode(LightsMode.Default)

  def setMode(self, mode: Enum) -> None:
    telemetry.log("Robot/Controllers/Lights/Mode", mode.name)
