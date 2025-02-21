import math
from commands2 import Command, cmd, Subsystem
from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ClosedLoopConfig
from ..classes import PositionControlModuleConfig, MotorDirection, Value
from .. import logger, utils

class PositionControlModule:
  def __init__(
    self,
    config: PositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    self._hasInitialZeroReset: bool = False
    self._targetPosition: float = Value.none
    self._isAlignedToPosition: bool = False

    encoderPositionConversionFactor: float = self._config.constants.distancePerRotation / self._config.constants.motorReduction
    encoderVelocityConversionFactor: float = encoderPositionConversionFactor / 60.0
    motorMotionMaxVelocity: float = (self._config.constants.motorMotionMaxVelocity / encoderPositionConversionFactor) * 60
    motorMotionMaxAcceleration: float = self._config.constants.motorMotionMaxAcceleration / encoderVelocityConversionFactor 

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
      .velocityConversionFactor(encoderVelocityConversionFactor))
    (self._motorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.constants.motorPID)
      .outputRange(*self._config.constants.motorOutputRange)
      .maxMotion
        .maxVelocity(motorMotionMaxVelocity)
        .maxAcceleration(motorMotionMaxAcceleration)
        .allowedClosedLoopError(self._config.constants.motorMotionAllowedClosedLoopError))
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
    self.resetPositionAlignment()
    self._motor.set(-speed if self._config.isInverted else speed)
    
  def setPosition(self, position: float) -> None:
    self._targetPosition = utils.clampValue(position, self._config.constants.motorSoftLimitReverse, self._config.constants.motorSoftLimitForward)
    self._closedLoopController.setReference(self._targetPosition, SparkBase.ControlType.kMAXMotionPositionControl)
    self._setIsAlignedToPosition()
    
  def getPosition(self) -> float:
    return self._encoder.getPosition()

  def _setIsAlignedToPosition(self) -> None:
    self._isAlignedToPosition = math.isclose(self.getPosition(), self._targetPosition, abs_tol = self._config.constants.motorMotionAllowedClosedLoopError)

  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition

  def isAtSoftLimit(self, direction: MotorDirection, tolerance: float) -> bool:
    return math.isclose(
      self.getPosition(),
      self._config.constants.motorSoftLimitReverse if direction == MotorDirection.Reverse else self._config.constants.motorSoftLimitForward, 
      abs_tol = tolerance
    )

  def suspendSoftLimitsCommand(self) -> Command:
    return cmd.startEnd(
      lambda: utils.setSparkSoftLimitsEnabled(self._motor, False),
      lambda: utils.setSparkSoftLimitsEnabled(self._motor, True)
    )

  def resetToZeroCommand(self, subsystem: Subsystem) -> Command:
    return cmd.startEnd(
      lambda: [
        utils.setSparkSoftLimitsEnabled(self._motor, False),
        self._motor.set(-self._config.constants.motorResetSpeed)
      ],
      lambda: [
        self._motor.stopMotor(),
        self._encoder.setPosition(0),
        utils.setSparkSoftLimitsEnabled(self._motor, True),
        setattr(self, "_hasInitialZeroReset", True)
      ],
      subsystem
    )
    
  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset
  
  def resetPositionAlignment(self) -> None:
    self._targetPosition = Value.none
    self._isAlignedToPosition = False

  def reset(self) -> None:
    self._motor.stopMotor()
    self.resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Position/Current', self._encoder.getPosition())
