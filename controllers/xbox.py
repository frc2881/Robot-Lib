import math
from wpilib import GenericHID
from wpimath import units
from commands2 import Command, cmd
from commands2.button import CommandXboxController, Trigger
from .. import logger, telemetry, utils
from ..classes import XboxControllerConfig, ControllerRumblePattern

class XboxController(CommandXboxController):
  def __init__(
      self, 
      config: XboxControllerConfig
    ) -> None:
    super().__init__(config.port)
    self._config = config

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getLeftY(self) -> units.percent:
    return utils.squareControllerInput(-super().get_left_y(), self._config.inputDeadband)
  
  def leftY(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getLeftY()) > self._config.inputDeadband)
  
  def getLeftX(self) -> units.percent:
    return utils.squareControllerInput(-super().get_left_x(), self._config.inputDeadband)
  
  def leftX(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getLeftX()) > self._config.inputDeadband)
  
  def getRightY(self) -> units.percent:
    return utils.squareControllerInput(-super().get_right_y(), self._config.inputDeadband)
  
  def rightY(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getRightY()) > self._config.inputDeadband)
  
  def getRightX(self) -> units.percent:
    return utils.squareControllerInput(-super().get_right_x(), self._config.inputDeadband)
  
  def rightX(self) -> Trigger:
    return Trigger(lambda: math.fabs(self.getRightX()) > self._config.inputDeadband)
  
  def rumble(self, pattern: ControllerRumblePattern) -> Command:
    return cmd.select(
      {
        ControllerRumblePattern.SHORT: cmd.start_end(
          lambda: self.get_hid().set_rumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1),
          lambda: self.get_hid().set_rumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0)
        ).with_timeout(0.5),
        ControllerRumblePattern.LONG: cmd.start_end(
          lambda: self.get_hid().set_rumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1),
          lambda: self.get_hid().set_rumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0)
        ).with_timeout(1.0)
      }, 
      lambda: pattern
    )

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/IsConnected', self.get_hid().is_connected())