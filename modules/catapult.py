from typing import Callable
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

    self._controller = utils.getSparkController(config.port, config.id, config.controllerType, config.motorType)
    sparkConfig = SparkBaseConfig()
    (sparkConfig
      .smart_current_limit(self._config.currentLimit)
      .set_idle_mode(SparkBaseConfig.IdleMode.BRAKE)
      .inverted(self._config.isInverted)
    )
    (sparkConfig.soft_limit
      .reverse_soft_limit(config.softLimitReverse)
      .reverse_soft_limit_enabled(True)
      .forward_soft_limit(config.softLimitForward)
      .forward_soft_limit_enabled(True)
    )
    (sparkConfig.closed_loop
      .set_feedback_sensor(FeedbackSensor.PRIMARY_ENCODER)
      .pid(*self._config.controlPID)
      .output_range(-1.0, 1.0)
      .feed_forward
        .s(self._config.feedForwardGains.static)
        .v(self._config.feedForwardGains.velocity)
        .a(self._config.feedForwardGains.acceleration)
        .g(self._config.feedForwardGains.gravity)
    )
    (sparkConfig.closed_loop.max_motion
      .cruise_velocity(self._config.cruiseVelocity)
      .max_acceleration(self._config.maxAcceleration)
      .allowed_profile_error(self._config.allowedProfileError)
    )
    utils.configureSparkController(self._controller, sparkConfig)
    self._closedLoopController = self._controller.get_closed_loop_controller()
    self._encoder = self._controller.get_encoder()
    self._encoder.set_position(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def launch(self, getSpeed: Callable[[], units.percent], subsystem: Subsystem) -> Command:
    return (
      cmd.run(lambda: self._setSpeed(getSpeed()), subsystem)
      .until(lambda: self.isAtLaunchPosition())
    )

  def retract(self, subsystem: Subsystem) -> Command:
    return (
      cmd.run(lambda: self._controller.set_throttle(-self._config.retractSpeed if not self.isRetracted() else 0), subsystem)
    )

  def _setSpeed(self, speed: units.percent) -> None:
    self._closedLoopController.set_setpoint(self._config.cruiseVelocity * speed, SparkBase.ControlType.MAX_MOTION_VELOCITY_CONTROL)

  def _getSpeed(self) -> units.percent:
    return self._encoder.get_velocity().get() / self._config.cruiseVelocity

  def isAtLaunchPosition(self) -> bool:
    return utils.isValueWithinTolerance(self._encoder.get_position().get(), self._config.launchPosition, 0.05)

  def isRetracted(self) -> bool:
    return utils.isValueWithinTolerance(self._encoder.get_position().get(), 0, self._config.allowedProfileError)


  def setIdleMode(self, idleMode: IdleMode) -> None:
    utils.setIdleMode(self._controller, idleMode)

  def resetToHome(self, subsystem: Subsystem) -> Command:
    return cmd.start_end(
      lambda: self._startHoming(),
      lambda: self._endHoming(),
      subsystem
    )
  
  def _startHoming(self) -> None:
    self._isHomed = False
    self._isHoming = True
    utils.setSoftLimitsEnabled(self._controller, False)
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.set_throttle(-self._config.homingSpeed)
    else:
      self.setIdleMode(IdleMode.COAST)

  def _endHoming(self) -> None:
    if utils.getRobotState() == RobotState.ENABLED:
      self._controller.stop_motor()
    else:
      self.setIdleMode(IdleMode.BRAKE)
    self._encoder.set_position(0)
    utils.setSoftLimitsEnabled(self._controller, True)
    self._isHomed = True
    self._isHoming = False
  
  def isHoming(self) -> bool:
    return self._isHoming

  def isHomed(self) -> bool:
    return self._isHomed

  def reset(self) -> None:
    self._controller.stop_motor()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._config.telemetryName}/Position', self._encoder.get_position().get())
    telemetry.log(f'{self._config.telemetryName}/Speed', self._getSpeed())
    telemetry.log(f'{self._config.telemetryName}/Velocity', self._encoder.get_velocity().get())
    telemetry.log(f'{self._config.telemetryName}/Current', self._controller.get_output_current().get())

