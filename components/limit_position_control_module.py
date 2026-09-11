from wpilib import SmartDashboard
from rev import SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ResetMode, PersistMode, LimitSwitchConfig
from ..classes import LimitPositionControlModuleConfig, MotorIdleMode, Position
from .. import logger, utils

class LimitPositionControlModule:
  def __init__(
    self,
    config: LimitPositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.baseKey}'

    self._targetPosition: Position = Position.Unknown

    if self._config.constants.motorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._motor = SparkFlex(self._config.motorCANId, self._config.constants.motorType)
    else: 
      self._motor = SparkMax(self._config.motorCANId, self._config.constants.motorType)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .smartCurrentLimit(self._config.constants.motorCurrentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._config.isInverted))
    (self._motorConfig.softLimit
      .reverseSoftLimitEnabled(False)
      .forwardSoftLimitEnabled(False))
    (self._motorConfig.limitSwitch
      .forwardLimitSwitchEnabled(True)
      .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
      .reverseLimitSwitchEnabled(True)
      .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
     )
    utils.setSparkConfig(self._motor.configure(self._motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._forwardLimitSwitch = self._motor.getForwardLimitSwitch()
    self._reverseLimitSwitch = self._motor.getReverseLimitSwitch()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: Position) -> None:
    self._targetPosition = position
    match self._targetPosition:
      case Position.Forward:
        self._motor.set(self._config.constants.motorOutputRange.max)
      case Position.Backward:
        self._motor.set(self._config.constants.motorOutputRange.min)
      case _:
        self._motor.stopMotor()
    
  def getPosition(self) -> Position:
    if self._forwardLimitSwitch.get(): return Position.Forward
    if self._reverseLimitSwitch.get(): return Position.Backward
    return Position.Unknown

  def getTargetPosition(self) -> Position:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    position = self.getPosition()
    return position != Position.Unknown and position == self._targetPosition

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString(f'{self._baseKey}/Position', self.getPosition().name)
    SmartDashboard.putBoolean(f'{self._baseKey}/LimitSwitchForward', self._forwardLimitSwitch.get())
    SmartDashboard.putBoolean(f'{self._baseKey}/LimitSwitchReverse', self._reverseLimitSwitch.get())
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
