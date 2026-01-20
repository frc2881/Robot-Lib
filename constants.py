from wpimath import units
from .classes import MotorModel, SwerveModuleGearKit

class Motors:
  MOTOR_FREE_SPEEDS: dict[MotorModel, units.revolutions_per_minute] = {
    MotorModel.NEO: 5676.0,
    MotorModel.NEOVortex: 6784.0,
    MotorModel.NEO550: 11000.0
  }

class Drive:
  SWERVE_MODULE_GEAR_RATIOS: dict[SwerveModuleGearKit, float] = {
    SwerveModuleGearKit.Low: 5.50,
    SwerveModuleGearKit.Medium: 5.08,
    SwerveModuleGearKit.High: 4.71,
    SwerveModuleGearKit.ExtraHigh1: 4.50,
    SwerveModuleGearKit.ExtraHigh2: 4.29,
    SwerveModuleGearKit.ExtraHigh3: 4.00,
    SwerveModuleGearKit.ExtraHigh4: 3.75,
    SwerveModuleGearKit.ExtraHigh5: 3.56
  }
