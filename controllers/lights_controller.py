from wpilib import SmartDashboard
from ..classes import LightsMode

class LightsController:
  def __init__(self) -> None:
    self.setLightsMode(LightsMode.Default)

  def setLightsMode(self, mode: LightsMode) -> None:
    SmartDashboard.putString("Robot/Lights/Mode", mode.name)