from enum import Enum, auto
from wpilib import SmartDashboard

class LightsMode(Enum):
  Default = auto()

class LightsController:
  def __init__(self) -> None:
    self.setMode(LightsMode.Default)

  def setMode(self, mode: Enum) -> None:
    SmartDashboard.putString("Robot/Controllers/Lights/Mode", mode.name)
