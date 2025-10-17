import math
from commands2 import Command, cmd, Subsystem
from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ClosedLoopConfig
from ..classes import RelativePositionControlModuleConfig, MotorDirection, Value
from .. import logger, utils

class PositionControlModule:
  def __init__(
    self,
    config: RelativePositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    self._hasZeroReset: bool = False
    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    encoderPositionConversionFactor: float = self._config.constants.distancePerRotation / self._config.constants.motorReduction

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
      .positionConversionFactor(encoderPositionConversionFactor)
      .velocityConversionFactor(encoderPositionConversionFactor / 60.0))
    if self._config.leaderMotorCANId is not None:
      self._motorConfig.follow(self._config.leaderMotorCANId, self._config.isInverted)
    else:
      (self._motorConfig.closedLoop
        .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(*self._config.constants.motorPID)
        .outputRange(*self._config.constants.motorOutputRange)
        .maxMotion
          .maxVelocity(self._config.constants.motorMotionMaxVelocity)
          .maxAcceleration(self._config.constants.motorMotionMaxAcceleration)
          .allowedClosedLoopError(self._config.constants.motorMotionAllowedClosedLoopError))
      (self._motorConfig.softLimit
        .reverseSoftLimitEnabled(True)
        .reverseSoftLimit(self._config.constants.motorSoftLimitReverse)
        .forwardSoftLimitEnabled(True)
        .forwardSoftLimit(self._config.constants.motorSoftLimitForward))
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
    self._motor.set(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.constants.motorSoftLimitReverse
    if position == Value.max: position = self._config.constants.motorSoftLimitForward
    if position != self._targetPosition:
      self._resetPosition()
      self._targetPosition = position
    self._closedLoopController.setReference(self._targetPosition, SparkBase.ControlType.kMAXMotionPositionControl)
    self._isAtTargetPosition = math.isclose(self.getPosition(), self._targetPosition, abs_tol = self._config.constants.motorMotionAllowedClosedLoopError)

  def getPosition(self) -> float:
    return self._encoder.getPosition()

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
    utils.setSparkSoftLimitsEnabled(self._motor, isEnabled)

  def resetToZero(self, subsystem: Subsystem) -> Command:
    return cmd.startEnd(
      lambda: [ 
        utils.setSparkSoftLimitsEnabled(self._motor, False),
        self._motor.set(-self._config.constants.motorResetSpeed) 
      ],
      lambda: [
        self._motor.stopMotor(),
        self._encoder.setPosition(0),
        utils.setSparkSoftLimitsEnabled(self._motor, True),
        setattr(self, "_hasZeroReset", True)
      ],
      subsystem
    )
  
  def hasZeroReset(self) -> bool:
    return self._hasZeroReset

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetPosition()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsAtTargetPosition', self._isAtTargetPosition)
    SmartDashboard.putNumber(f'{self._baseKey}/Position', self._encoder.getPosition())
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
    SmartDashboard.putNumber(f'{self._baseKey}/Velocity', self._encoder.getVelocity())
