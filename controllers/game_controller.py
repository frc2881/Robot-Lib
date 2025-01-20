import math
from commands2 import Command, cmd
from commands2.button import CommandXboxController, Trigger
from wpimath import units
from wpilib import XboxController
from .. import logger, utils
from ..classes import ControllerRumblePattern

class GameController(CommandXboxController):
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
  
  def rumbleCommand(self, pattern: ControllerRumblePattern) -> Command:
    match pattern:
      case ControllerRumblePattern.Short:
        return cmd.run(
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ).withTimeout(
          0.5
        ).finallyDo(
          lambda end: self.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      case ControllerRumblePattern.Long:
        return cmd.run(
          lambda: self.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ).withTimeout(
          1.5
        ).finallyDo(
          lambda end: self.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      case _:
        return cmd.none()
