import math
from commands2 import Command, cmd
from commands2.button import CommandXboxController, Trigger
from wpimath import units
from wpilib import XboxController
from .. import logger, utils
from ..classes import ControllerRumblePattern

class Xbox(CommandXboxController):
  def __init__(
      self, 
      port: int, 
      inputDeadband: units.percent
    ) -> None:
    super().__init__(port)
    self._inputDeadband = inputDeadband

  def getLeftY(self) -> units.percent:
    return utils.squareControllerInput(-super().getLeftY(), self._inputDeadband)
  
  def leftY(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getLeftY()) > self._inputDeadband)
  
  def getLeftX(self) -> units.percent:
    return utils.squareControllerInput(-super().getLeftX(), self._inputDeadband)
  
  def leftX(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getLeftX()) > self._inputDeadband)
  
  def getRightY(self) -> units.percent:
    return utils.squareControllerInput(-super().getRightY(), self._inputDeadband)
  
  def rightY(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getRightY()) > self._inputDeadband)
  
  def getRightX(self) -> units.percent:
    return utils.squareControllerInput(-super().getRightX(), self._inputDeadband)
  
  def rightX(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getRightX()) > self._inputDeadband)
  
  def rumble(self, pattern: ControllerRumblePattern) -> Command:
    return cmd.select(
      {
        ControllerRumblePattern.Short: cmd.startEnd(
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 1),
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 0)
        ).withTimeout(0.5),
        ControllerRumblePattern.Long: cmd.startEnd(
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 1),
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 0)
        ).withTimeout(1.0),
      }, 
      lambda: pattern
    )
