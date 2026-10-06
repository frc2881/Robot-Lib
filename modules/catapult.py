from wpimath import units
from commands2 import Command, cmd, Subsystem
from rev import SparkBase, SparkBaseConfig, FeedbackSensor
from .. import logger, telemetry, utils
from ..classes import CatapultModuleConfig, IdleMode, RobotState

class CatapultModule:
  def __init__(
    self,
    config: CatapultModuleConfig
  ) -> None:
    self._config = config

    self._isHoming: bool = False
    self._isHomed: bool = False

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
      .reverseSoftLimit(0)
      .reverseSoftLimitEnabled(True)
      .forwardSoftLimit(self._config.launchPosition)
      .forwardSoftLimitEnabled(True)
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

  def launch(self, speed: units.percent, subsystem: Subsystem) -> Command:
    return (
      cmd.run(lambda: self._setSpeed(speed), subsystem)
      .until(lambda: self._controller.getForwardSoftLimit().isReached())
      .andThen(
        cmd.run(lambda: self._setSpeed(-self._config.resetSpeed), subsystem)
        .until(lambda: self.isReset())
      )
      .finallyDo(lambda end: self._setSpeed(-self._config.holdSpeed))
    )

  def _setSpeed(self, speed: units.percent) -> None:
    self._closedLoopController.setSetpoint(self._config.cruiseVelocity * speed, SparkBase.ControlType.kMAXMotionVelocityControl)

  def _getSpeed(self) -> units.percent:
    return self._encoder.getVelocity() / self._config.cruiseVelocity

  def isReset(self) -> bool:
    return self._controller.getReverseSoftLimit().isReached()

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
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.set(-self._config.homingSpeed)
    else:
      self.setIdleMode(IdleMode.COAST)

  def _endHoming(self) -> None:
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.stopMotor()
    else:
      self.setIdleMode(IdleMode.BRAKE)
    self._encoder.setPosition(0)
    utils.setSoftLimitsEnabled(self._controller, True)
    self._isHomed = True
    self._isHoming = False
  
  def isHoming(self) -> bool:
    return self._isHoming

  def isHomed(self) -> bool:
    return self._isHomed

  def reset(self) -> None:
    self._controller.stopMotor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._encoder.getPosition())
    telemetry.log(f'{self._config.telemetryName}/Speed', self._getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.getVelocity())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.getOutputCurrent())

