from commands2 import Command, cmd
from wpilib import ADIS16470_IMU, SPI, SmartDashboard, RobotBase
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d
from .. import logger, utils

class GyroSensor_ADIS16470():
  def __init__(
      self, 
      spiPort: SPI.Port,
      imuAxisYaw: ADIS16470_IMU.IMUAxis,
      imuAxisPitch: ADIS16470_IMU.IMUAxis,
      imuAxisRoll: ADIS16470_IMU.IMUAxis,
      initCalibrationTime: ADIS16470_IMU.CalibrationTime,
      commandCalibrationTime: ADIS16470_IMU.CalibrationTime,
      commandCalibrationDelay: units.seconds
    ) -> None:
    self._commandCalibrationTime = commandCalibrationTime
    self._commandCalibrationDelay = commandCalibrationDelay
    self._baseKey = f'Robot/Sensors/Gyro'
    
    self._gyro = ADIS16470_IMU(
      imuAxisYaw,
      imuAxisPitch,
      imuAxisRoll,
      spiPort,
      initCalibrationTime
    )
    
    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getHeading(self) -> units.degrees:
    return utils.wrapAngle(self._gyro.getAngle())
  
  def getRotation(self) -> Rotation2d:
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getPitch(self) -> units.degrees:
    return utils.wrapAngle(self._gyro.getAngle(self._gyro.getPitchAxis()))
  
  def getRoll(self) -> units.degrees:
    return utils.wrapAngle(self._gyro.getAngle(self._gyro.getRollAxis()))
  
  def getTurnRate(self) -> units.degrees_per_second:
    return self._gyro.getRate()
  
  def _reset(self, heading: units.degrees) -> None:
    self._gyro.setGyroAngle(self._gyro.getYawAxis(), heading)
    self._gyro.reset()

  def resetRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))

  def resetCommand(self) -> Command:
    return cmd.runOnce(self._reset).ignoringDisable(True).withName("GyroSensor:Reset")
  
  def _calibrate(self) -> None:
    if RobotBase.isReal():
      self._gyro.configCalTime(self._commandCalibrationTime)
      self._gyro.calibrate()

  def calibrateCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(
        lambda: [
          SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', True),
          self._calibrate()
        ]
      ),
      cmd.waitSeconds(self._commandCalibrationDelay),
      cmd.runOnce(
        lambda: SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', False)
      )
    ).ignoringDisable(True).withName("GyroSensor:Calibrate")
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
    SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
    SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
    SmartDashboard.putNumber(f'{self._baseKey}/TurnRate', self.getTurnRate())
