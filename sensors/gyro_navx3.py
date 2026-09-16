from commands2 import Command, cmd
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d
from navx import Navx
from .. import logger, utils

class Gyro_NAVX3():
  def __init__(
      self,
      port: Navx.Port
    ) -> None:
    self._baseKey = f'Robot/Sensors/Gyro'
    
    self._gyro = Navx(port)

    self._angleAdjustment: units.degrees = 0

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
  
  def getHeading(self) -> units.degrees:
    return -utils.wrapAngle(self._gyro.getAngle() + self._angleAdjustment)
  
  def getPitch(self) -> units.degrees:
    return self._gyro.getPitch()
  
  def getRoll(self) -> units.degrees:
    return self._gyro.getRoll()
  
  def _reset(self, heading: units.degrees = 0) -> None:
    self._angleAdjustment = -heading if heading != 0 else 0
    self._gyro.resetYaw()

  def resetRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))

  def reset(self) -> Command:
    return cmd.runOnce(self._reset).withName("GyroSensor:Reset")
  
  def isConnected(self) -> bool:
    return self._gyro.getTemperature() > 0
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self.isConnected())
    SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
    SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
    SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
