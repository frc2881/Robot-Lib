from wpilib import SmartDashboard
from rev import SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ResetMode, PersistMode
from ..classes import FollowerModuleConfig, MotorIdleMode
from .. import logger, utils

class FollowerModule:
  def __init__(
    self,
    config: FollowerModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.baseKey}'

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake))
    self._motorConfig.follow(self._config.leaderMotorCANId, self._config.isInverted)
    utils.setSparkConfig(self._motor.configure(self._motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._relativeEncoder = self._motor.getEncoder()
    self._relativeEncoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
    SmartDashboard.putNumber(f'{self._baseKey}/Velocity', self._relativeEncoder.getVelocity())
