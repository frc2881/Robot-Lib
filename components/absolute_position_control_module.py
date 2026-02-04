import math
from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, FeedbackSensor, ResetMode, PersistMode
from ..classes import AbsolutePositionControlModuleConfig, MotorDirection, MotorIdleMode, Value
from .. import logger, utils

class AbsolutePositionControlModule:
  def __init__(
    self,
    config: AbsolutePositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.baseKey}'

    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .inverted(self._config.isInverted))
    (self._motorConfig.encoder
      .positionConversionFactor(self._config.constants.motorRelativeEncoderPositionConversionFactor)
      .velocityConversionFactor(self._config.constants.motorRelativeEncoderPositionConversionFactor / 60.0))
    (self._motorConfig.absoluteEncoder
      .positionConversionFactor(self._config.constants.motorAbsoluteEncoderPositionConversionFactor)
      .inverted(self._config.isInverted))
    (self._motorConfig.softLimit
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._config.constants.motorSoftLimitReverse)
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._config.constants.motorSoftLimitForward))
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
      .cruiseVelocity(self._config.constants.motorMotionCruiseVelocity)
      .maxAcceleration(self._config.constants.motorMotionMaxAcceleration)
      .allowedProfileError(self._config.constants.motorMotionAllowedProfileError))
    utils.setSparkConfig(self._motor.configure(self._motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._closedLoopController = self._motor.getClosedLoopController()
    self._relativeEncoder = self._motor.getEncoder()
    self._absoluteEncoder = self._motor.getAbsoluteEncoder()
    self._relativeEncoder.setPosition(self._absoluteEncoder.getPosition())

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._motor.set(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.constants.motorSoftLimitReverse
    if position == Value.max: position = self._config.constants.motorSoftLimitForward
    if position != self._targetPosition:
      self._resetPosition()
      self._targetPosition = position
    self._closedLoopController.setSetpoint(self._targetPosition, SparkBase.ControlType.kMAXMotionPositionControl)
    self._isAtTargetPosition = math.isclose(self.getPosition(), self._targetPosition, abs_tol = self._config.constants.motorMotionAllowedProfileError)

  def getPosition(self) -> float:
    return self._relativeEncoder.getPosition()

  def _resetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def isAtSoftLimit(self, direction: MotorDirection, tolerance: float) -> bool:
    return math.isclose(
      self.getPosition(),
      self._config.constants.motorSoftLimitReverse if direction == MotorDirection.Reverse else self._config.constants.motorSoftLimitForward, 
      abs_tol = tolerance
    )
  
  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSoftLimitsEnabled(self._motor, isEnabled)

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetPosition()
    self._relativeEncoder.setPosition(self._absoluteEncoder.getPosition())

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsAtTargetPosition', self._isAtTargetPosition)
    SmartDashboard.putNumber(f'{self._baseKey}/RelativePosition', self._relativeEncoder.getPosition())
    SmartDashboard.putNumber(f'{self._baseKey}/AbsolutePosition', self._absoluteEncoder.getPosition())
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
    SmartDashboard.putNumber(f'{self._baseKey}/Velocity', self._relativeEncoder.getVelocity())
