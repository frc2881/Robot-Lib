from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, FeedbackSensor, ResetMode, PersistMode
from ..classes import VelocityControlModuleConfig, MotorIdleMode
from .. import logger, utils

class VelocityControlModule:
  def __init__(
    self,
    config: VelocityControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.baseKey}'

    self._targetSpeed: float = 0

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._config.isInverted))
    (self._motorConfig.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(self._config.constants.motorVelocityConversionFactor))
    (self._motorConfig.closedLoop
      .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.constants.motorPID)
      .outputRange(*self._config.constants.motorOutputRange)
      .feedForward
        .kS(self._config.constants.motorFeedForwardGains.static)
        .kV(self._config.constants.motorFeedForwardGains.velocity)
        .kA(self._config.constants.motorFeedForwardGains.acceleration)
        .kG(self._config.constants.motorFeedForwardGains.gravity))
    (self._motorConfig.closedLoop.maxMotion
      .maxAcceleration(self._config.constants.motorMotionMaxAcceleration)
      .allowedProfileError(0.1))
    utils.setSparkConfig(self._motor.configure(self._motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._closedLoopController = self._motor.getClosedLoopController()
    self._relativeEncoder = self._motor.getEncoder()
    self._relativeEncoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._targetSpeed = speed
    self._closedLoopController.setSetpoint(self._config.constants.motorMotionMaxVelocity * speed, SparkBase.ControlType.kMAXMotionVelocityControl)

  def getSpeed(self) -> units.percent:
    return self._relativeEncoder.getVelocity() / self._config.constants.motorMotionMaxVelocity

  def getTargetSpeed(self) -> float:
    return self._targetSpeed

  def isAtTargetSpeed(self) -> bool:
    return self._targetSpeed != 0 and utils.isValueWithinTolerance(self.getSpeed(), self._targetSpeed, 0.1)
  
  def _resetTargetSpeed(self) -> None:
    self._targetSpeed = 0

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetTargetSpeed()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Speed', self.getSpeed())
    SmartDashboard.putNumber(f'{self._baseKey}/Velocity', self._relativeEncoder.getVelocity())
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())

