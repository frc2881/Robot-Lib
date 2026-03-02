from wpilib import SmartDashboard
from wpimath import units
from rev import SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ResetMode, PersistMode
from ..classes import SpeedModuleConfig, MotorIdleMode
from .. import logger, utils

class SpeedModule:
  def __init__(
    self,
    config: SpeedModuleConfig
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
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._config.isInverted))
    (self._motorConfig.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(self._config.constants.motorVelocityConversionFactor))
    utils.setSparkConfig(self._motor.configure(self._motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._relativeEncoder = self._motor.getEncoder()
    self._relativeEncoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, speed: units.percent) -> None:
    self._motor.set(speed)

  def getSpeed(self) -> units.percent:
    return self._motor.get()

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Speed', self.getSpeed())
    SmartDashboard.putNumber(f'{self._baseKey}/Velocity', self._relativeEncoder.getVelocity())
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
