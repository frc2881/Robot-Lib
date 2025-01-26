from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ClosedLoopConfig
from ..classes import PositionControlModuleConfig, MotorControllerType
from .. import logger, utils

class PositionControlModule:
  def __init__(
    self,
    config: PositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    encoderPositionConversionFactor: float = self._config.constants.motorTravelDistance / self._config.constants.motorReduction
    encoderVelocityConversionFactor: float = encoderPositionConversionFactor / 60.0
    motorMotionMaxVelocity: float = (self._config.constants.motorMotionMaxVelocityRate / encoderPositionConversionFactor) * 60
    motorMotionMaxAcceleration: float = self._config.constants.motorMotionMaxAccelerationRate / encoderVelocityConversionFactor 

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
      .positionConversionFactor(encoderPositionConversionFactor)
      .velocityConversionFactor(encoderVelocityConversionFactor))
    (self._motorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.constants.motorPID)
      .outputRange(-1.0, 1.0)
      .maxMotion
        .maxVelocity(motorMotionMaxVelocity)
        .maxAcceleration(motorMotionMaxAcceleration)
        .allowedClosedLoopError(self._config.constants.allowedClosedLoopError))
    (self._motorConfig.softLimit
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._config.constants.motorSoftLimitForward)
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._config.constants.motorSoftLimitReverse))
    if self._config.leaderMotorCANId is not None:
      self._motorConfig.follow(self._config.leaderMotorCANId)
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

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._motor.set(speed)

  def setPosition(self, position: float) -> None:
    self._closedLoopController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl)

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
