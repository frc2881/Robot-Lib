from commands2 import Command, cmd
from wpilib import SmartDashboard, RobotBase
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d
from navx import AHRS
from .. import logger, utils

class GyroSensor_NAVX2():
  def __init__(
      self,
      comType: AHRS.NavXComType
    ) -> None:
    self._baseKey = f'Robot/Sensors/Gyro'
    
    self._gyro = AHRS(comType)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
  
  def getHeading(self) -> units.degrees:
    return -utils.wrapAngle(self._gyro.getAngle())
  
  def getRotation(self) -> Rotation2d:
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getPitch(self) -> units.degrees:
    return self._gyro.getPitch()
  
  def getRoll(self) -> units.degrees:
    return self._gyro.getRoll()
  
  def getTurnRate(self) -> units.degrees_per_second:
    return self._gyro.getRate()
  
  def _reset(self, heading: units.degrees = 0) -> None:
    self._gyro.setAngleAdjustment(-heading if heading != 0 else 0)
    self._gyro.reset()

  def resetRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))

  def resetCommand(self) -> Command:
    return cmd.runOnce(self._reset).ignoringDisable(True).withName("GyroSensor:Reset")

  def _calibrate(self) -> None:
    if RobotBase.isReal():
      pass # NO-OP as navX2 currently does automatic calibration

  def calibrateCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(
        lambda: [
          SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', True),
          self._calibrate()
        ]
      ),
      cmd.waitSeconds(0.2),
      cmd.runOnce(
        lambda: SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', False)
      )
    ).ignoringDisable(True).withName("GyroSensor:Calibrate")
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
    SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
    SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
    SmartDashboard.putNumber(f'{self._baseKey}/TurnRate', self.getTurnRate())
