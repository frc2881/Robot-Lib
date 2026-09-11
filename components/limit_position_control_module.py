from wpilib import SmartDashboard
from rev import SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ResetMode, PersistMode, LimitSwitchConfig
from ..classes import LimitPositionControlModuleConfig, MotorIdleMode, Value
from .. import logger, utils

class LimitPositionControlModule:
  def __init__(
    self,
    config: LimitPositionControlModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/{self._config.baseKey}'

    self._targetPosition: float = Value.none
    self._isAtTargetPosition: bool = False

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
    self._forwardLimit = self._motor.getForwardLimitSwitch()
    self._reverseLimit = self._motor.getReverseLimitSwitch()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: float) -> None:
    if position < 0:
       position = -1.0
    elif position > 0:
       position = 1.0
    else:
      position = 0
    self._targetPosition = position
    self._motor.set(position * self._config.constants.motorMaxSpeed)
    self._isAtTargetPosition = utils.isValueWithinTolerance(self.getPosition(), self._targetPosition, 0)

  def getPosition(self) -> float:
    if self._forwardLimit.get():
      return 1.0
    if self._reverseLimit.get():
      return -1.0
    return 0

  def getTargetPosition(self) -> float:
    return self._targetPosition

  def isAtTargetPosition(self) -> bool:
    return self._isAtTargetPosition

  def _resetTargetPosition(self) -> None:
    self._targetPosition = Value.none
    self._isAtTargetPosition = False

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    utils.setMotorIdleMode(self._motor, motorIdleMode)

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetTargetPosition()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsAtTargetPosition', self._isAtTargetPosition)
    SmartDashboard.putNumber(f'{self._baseKey}/Current', self._motor.getOutputCurrent())
    SmartDashboard.putNumber(f'{self._baseKey}/Position', self.getPosition())
    SmartDashboard.putBoolean(f'{self._baseKey}/ForwardLimit', self._forwardLimit.get())
    SmartDashboard.putBoolean(f'{self._baseKey}/ReverseLimit', self._reverseLimit.get())
