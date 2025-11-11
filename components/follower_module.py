from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex
from ..classes import FollowerModuleConfig
from .. import logger, utils

class FollowerModule:
  def __init__(
    self,
    config: FollowerModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.moduleBaseKey}'

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.motorCurrentLimit))
    self._motorConfig.follow(self._config.leaderMotorCANId, self._config.isInverted)
    utils.setSparkConfig(
      self._motor.configure(
        self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
