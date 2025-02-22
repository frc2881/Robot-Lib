from wpilib import SmartDashboard

class Lights:
  def __init__(self) -> None:
    self.setMode("Default")

  def setMode(self, mode: str) -> None:
    SmartDashboard.putString("Robot/Lights/Mode", mode)
