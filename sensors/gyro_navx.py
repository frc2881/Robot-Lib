from wpimath import units, Pose2d
from commands2 import Command, cmd
from navx import Navx
from .. import logger, telemetry, utils

class Gyro():
  def __init__(
      self
    ) -> None:
    self._telemetryName = "Robot/Sensors/Gyro"

    self._gyro = Navx.usb()

    self._angleAdjustment: units.degrees = 0

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
  
  def getHeading(self) -> units.degrees:
    return -utils.wrapAngle(self._gyro.getAngle() or 0 + self._angleAdjustment)
  
  def getPitch(self) -> units.degrees:
    return self._gyro.getPitch() or 0
  
  def getRoll(self) -> units.degrees:
    return self._gyro.getRoll() or 0
  
  def _reset(self, heading: units.degrees = 0) -> None:
    self._angleAdjustment = -heading if heading != 0 else 0
    self._gyro.resetYaw()

  def resetRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))

  def reset(self) -> Command:
    return cmd.run_once(self._reset).with_name("Gyro:Reset")
  
  def isConnected(self) -> bool:
    return self._gyro.isConnected()
  
  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/IsConnected', self.isConnected())
    telemetry.log(f'{self._telemetryName}/Heading', self.getHeading())
    telemetry.log(f'{self._telemetryName}/Pitch', self.getPitch())
    telemetry.log(f'{self._telemetryName}/Roll', self.getRoll())
