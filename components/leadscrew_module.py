from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ClosedLoopConfig
from ..classes import LeadscrewModuleConfig, MotorControllerType
from .. import logger, utils

class LeadscrewModule:
  def __init__(
    self,
    config: LeadscrewModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    _encoderPositionConversionFactor: float = self._config.constants.leadscrewTravelDistance / self._config.constants.motorReduction
    _encoderVelocityConversionFactor: float = _encoderPositionConversionFactor / 60.0
    _motorSmartMotionMaxVelocity: float = (self._config.constants.motorSmartMotionMaxVelocityRate / _encoderPositionConversionFactor) * 60
    _motorSmartMotionMaxAcceleration: float = self._config.constants.motorSmartMotionMaxAccelerationRate / _encoderVelocityConversionFactor 

    if self._config.constants.motorControllerType == MotorControllerType.SparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, SparkLowLevel.MotorType.kBrushless)
    else: 
      self._motor = SparkMax(self._config.motorCANId, SparkLowLevel.MotorType.kBrushless)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .secondaryCurrentLimit(self._config.constants.motorCurrentLimit))
    (self._motorConfig.encoder
      .positionConversionFactor(_encoderPositionConversionFactor)
      .velocityConversionFactor(_encoderVelocityConversionFactor))
    (self._motorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.constants.motorPID)
      .outputRange(-1.0, 1.0)
      .smartMotion
        .maxVelocity(_motorSmartMotionMaxVelocity)
        .maxAcceleration(_motorSmartMotionMaxAcceleration))
    (self._motorConfig.softLimit
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._config.constants.motorSoftLimitForward)
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._config.constants.motorSoftLimitReverse))
    utils.setSparkConfig(
      self._motor.configure(
        self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._closedLoopController = self._motor.getClosedLoopController()
    self._encoder = self._motor.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._updateTelemetry)
    
  def setSpeed(self, speed: units.percent) -> None:
    self._motor.set(speed)

  def setPosition(self, position: float) -> None:
    self._closedLoopController.setReference(position, SparkBase.ControlType.kSmartMotion)

  def getPosition(self) -> float:
    return self._encoder.getPosition()
  
  def startZeroReset(self) -> None:
    utils.setSparkSoftLimitsEnabled(self._motor, False)
    self._motor.set(-self._config.constants.motorResetSpeed)

  def endZeroReset(self) -> None:
    self._motor.stopMotor()
    self._encoder.setPosition(0)
    utils.setSparkSoftLimitsEnabled(self._motor, True)

  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Position', self._encoder.getPosition())
