from wpimath import units
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import VelocityControlModuleConfig, IdleMode

class VelocityControlModule:
  def __init__(
    self,
    config: VelocityControlModuleConfig
  ) -> None:
    self._config = config

    self._targetSpeed: float = 0

    self._controller = utils.getSparkController(config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smartCurrentLimit(self._config.currentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._config.isInverted)
    )
    (sparkConfig.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0)
    )
    (sparkConfig.softLimit
      .reverseSoftLimitEnabled(False)
      .forwardSoftLimitEnabled(False)
    )
    (sparkConfig.closedLoop
      .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(*self._config.controlPID)
      .outputRange(-1.0, 1.0)
      .feedForward
        .kS(self._config.feedForwardGains.static)
        .kV(self._config.feedForwardGains.velocity)
        .kA(self._config.feedForwardGains.acceleration)
        .kG(self._config.feedForwardGains.gravity)
    )
    (sparkConfig.closedLoop.maxMotion
      .cruiseVelocity(self._config.cruiseVelocity)
      .maxAcceleration(self._config.maxAcceleration)
      .allowedProfileError(self._config.allowedProfileError)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._closedLoopController = self._controller.getClosedLoopController()
    self._encoder = self._controller.getEncoder()
    self._encoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()
    
  def setSpeed(self, speed: units.percent) -> None:
    self._targetSpeed = speed
    self._closedLoopController.setSetpoint(self._config.cruiseVelocity * speed, SparkBase.ControlType.kMAXMotionVelocityControl)

  def getSpeed(self) -> units.percent:
    return self._encoder.getVelocity() / self._config.cruiseVelocity

  def getTargetSpeed(self) -> units.percent:
    return self._targetSpeed

  def isAtTargetSpeed(self) -> bool:
    return self._targetSpeed != 0 and utils.isValueWithinTolerance(self.getSpeed(), self._targetSpeed, 0.05)
  
  def _resetTargetSpeed(self) -> None:
    self._targetSpeed = 0

  def getOutputCurrent(self) -> units.amperes:
    return self._controller.getOutputCurrent()

  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def reset(self) -> None:
    self._controller.stopMotor()
    self._resetTargetSpeed()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Speed', self.getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())

