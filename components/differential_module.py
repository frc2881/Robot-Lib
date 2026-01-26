import math
from wpilib import SmartDashboard
from rev import SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, ResetMode, PersistMode
from ..classes import DifferentialModuleConfig, MotorIdleMode
from .. import utils, logger

class DifferentialModule:
  def __init__(
    self,
    config: DifferentialModuleConfig
  ) -> None:
    self._config = config

    self._baseKey = f'Robot/Drive/Modules/{self._config.location.name}'

    drivingEncoderPositionConversionFactor: float = (self._config.constants.wheelDiameter * math.pi) / self._config.constants.drivingMotorReduction
    
    if self._config.constants.drivingMotorControllerType == SparkLowLevel.SparkModel.kSparkFlex:
      self._drivingMotor = SparkFlex(self._config.drivingMotorCANId, self._config.constants.drivingMotorType)
    else: 
      self._drivingMotor = SparkMax(self._config.drivingMotorCANId, self._config.constants.drivingMotorType)
    self._drivingMotorConfig = SparkBaseConfig()
    (self._drivingMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._config.constants.drivingMotorCurrentLimit)
      .inverted(self._config.isInverted))
    (self._drivingMotorConfig.encoder
      .positionConversionFactor(drivingEncoderPositionConversionFactor)
      .velocityConversionFactor(drivingEncoderPositionConversionFactor / 60.0))
    if self._config.leaderMotorCANId is not None:
      self._drivingMotorConfig.follow(self._config.leaderMotorCANId)
    utils.setSparkConfig(self._drivingMotor.configure(self._drivingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingEncoder.setPosition(0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getMotorController(self) -> SparkMax:
    return self._drivingMotor

  def getPosition(self) -> float:
    return self._drivingEncoder.getPosition()
  
  def getVelocity(self) -> float:
    return self._drivingEncoder.getVelocity()
  
  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    idleMode = SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
    utils.setSparkConfig(self._drivingMotor.configure(SparkBaseConfig().setIdleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters))
    
  def reset(self) -> None:
    self._drivingEncoder.setPosition(0)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Velocity', self._drivingEncoder.getVelocity())
