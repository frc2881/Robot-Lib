from wpimath import units
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import AbsolutePositionControlModuleConfig, MotorDirection, IdleMode, Value

class AbsolutePositionControlModule:
  def __init__(
    self,
    config: AbsolutePositionControlModuleConfig
  ) -> None:
    self._config = config

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
      .positionConversionFactor(config.relativePositionConversionFactor)
      .velocityConversionFactor(config.relativePositionConversionFactor / 60.0)
    )
    (sparkConfig.absoluteEncoder
      .positionConversionFactor(config.absolutePositionConversionFactor)
      .inverted(config.isInverted)
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
    self._absoluteEncoder = self._controller.getAbsoluteEncoder()
    self._relativeEncoder = self._controller.getEncoder()
    self._relativeEncoder.setPosition(self._absoluteEncoder.getPosition())

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._controller.set(-speed if self._config.isInverted else speed)
    if speed != 0:
      self._resetPosition()
    
  def setPosition(self, position: float) -> None:
    if position == Value.min: position = self._config.softLimitReverse
    if position == Value.max: position = self._config.softLimitForward
    if position != self._targetPosition:
      self._resetPosition()
      self._targetPosition = position
    self._closedLoopController.setSetpoint(self._targetPosition, SparkBase.ControlType.kMAXMotionPositionControl)
    self._isAtTargetPosition = utils.isValueWithinTolerance(self.getPosition(), self._targetPosition, self._config.allowedProfileError)

  def getPosition(self) -> float:
    return self._relativeEncoder.getPosition()

  def getTargetPosition(self) -> float:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def _resetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def isAtSoftLimit(self, direction: MotorDirection, tolerance: float) -> bool:
    return utils.isValueWithinTolerance(
      self.getPosition(),
      self._config.softLimitReverse if direction == MotorDirection.REVERSE else self._config.softLimitForward, 
      tolerance
    )
  
  def setSoftLimitsEnabled(self, isEnabled: bool) -> None:
    utils.setSoftLimitsEnabled(self._controller, isEnabled)

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stopMotor()
    self._relativeEncoder.setPosition(self._absoluteEncoder.getPosition())
    self._resetPosition()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._absoluteEncoder.getPosition())
    telemetry.log(f'{self._config.telemetryName}/RelativePosition', self._relativeEncoder.getPosition())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._relativeEncoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())
