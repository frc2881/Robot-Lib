from wpilib import SmartDashboard

class LightsController:
  def __init__(self) -> None:
    self.setLightsMode("Default")

  def setLightsMode(self, mode: str) -> None:
    SmartDashboard.putString("Robot/Lights/Mode", mode)
