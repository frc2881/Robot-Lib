from wpimath import units
from commands2 import Command, cmd, Subsystem
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import RelativePositionControlModuleConfig, MotorDirection, IdleMode, RobotState, Value

class RelativePositionControlModule:
  def __init__(
    self,
    config: RelativePositionControlModuleConfig
  ) -> None:
    self._config = config

    self._isHoming: bool = False
    self._isHomed: bool = False
    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

    self._controller = utils.getSparkController(config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smartCurrentLimit(config.currentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(config.isInverted)
    )
    (sparkConfig.encoder
      .positionConversionFactor(config.positionConversionFactor)
      .velocityConversionFactor(config.positionConversionFactor / 60.0)
    )
    (sparkConfig.softLimit
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(config.softLimitReverse)
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(config.softLimitForward)
    )
    (sparkConfig.closedLoop
      .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(*config.controlPID)
      .outputRange(*config.outputRange)
      .feedForward
        .kS(config.feedForwardGains.static)
        .kV(config.feedForwardGains.velocity)
        .kA(config.feedForwardGains.acceleration)
        .kG(config.feedForwardGains.gravity)
    )
    (sparkConfig.closedLoop.maxMotion
      .cruiseVelocity(config.cruiseVelocity)
      .maxAcceleration(config.maxAcceleration)
      .allowedProfileError(config.allowedProfileError)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._closedLoopController = self._controller.getClosedLoopController()
    self._encoder = self._controller.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetTargetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.softLimitReverse
    if position == Value.max: position = self._config.softLimitForward
    if position != self._targetPosition:
      self._resetTargetPosition()
      self._targetPosition = position
    self._closedLoopController.setSetpoint(self._targetPosition, SparkBase.ControlType.kMAXMotionPositionControl)
    self._isAtTargetPosition = utils.isValueWithinTolerance(self.getPosition(), self._targetPosition, self._config.allowedProfileError)

  def getPosition(self) -> float:
    return self._encoder.getPosition()

  def getTargetPosition(self) -> float:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def _resetTargetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtSoftLimit(self, direction: MotorDirection, tolerance: float) -> bool:
    return utils.isValueWithinTolerance(
      self.getPosition(),
      self._config.softLimitReverse if direction == MotorDirection.Reverse else self._config.softLimitForward, 
      tolerance
    )

  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSoftLimitsEnabled(self._controller, isEnabled)

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def resetToHome(self, subsystem: Subsystem) -> Command:
    return cmd.startEnd(
      lambda: self._startHoming(),
      lambda: self._endHoming(),
      subsystem
    )
  
  def _startHoming(self) -> None:
    self._isHomed = False
    self._isHoming = True
    utils.setSoftLimitsEnabled(self._controller, False)
    if utils.getRobotState() == RobotState.Enabled:
      self._controller.set(-self._config.homingSpeed)
    else:
      self.setIdleMode(IdleMode.Coast)

  def _endHoming(self) -> None:
    if utils.getRobotState() == RobotState.Enabled:
      self._controller.stopMotor()
    else:
      self.setIdleMode(IdleMode.Brake)
    self._encoder.setPosition(self._config.homingPosition)
    utils.setSoftLimitsEnabled(self._controller, True)
    self._isHomed = True
    self._isHoming = False
  
  def isHoming(self) -> bool:
    return self._isHoming

  def isHomed(self) -> bool:
    return self._isHomed

  def reset(self) -> None:
    self._controller.stopMotor()
    self._resetTargetPosition()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._encoder.getPosition())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())
