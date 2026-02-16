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
    SwerveModuleGearKit.Low: 5.50,        # 12 / 22 
    SwerveModuleGearKit.Medium: 5.08,     # 13 / 22
    SwerveModuleGearKit.High: 4.71,       # 14 / 22
    SwerveModuleGearKit.ExtraHigh1: 4.50, # 14 / 21
    SwerveModuleGearKit.ExtraHigh2: 4.29, # 14 / 20
    SwerveModuleGearKit.ExtraHigh3: 4.00, # 15 / 20
    SwerveModuleGearKit.ExtraHigh4: 3.75, # 16 / 20
    SwerveModuleGearKit.ExtraHigh5: 3.56  # 16 / 19
  }

  SWERVE_MODULE_FREE_SPEEDS: dict[MotorModel, dict[SwerveModuleGearKit, units.meters_per_second]] = {
    MotorModel.NEOVortex: {
      SwerveModuleGearKit.Low: 4.92,
      SwerveModuleGearKit.Medium: 5.33,
      SwerveModuleGearKit.High: 5.74,
      SwerveModuleGearKit.ExtraHigh1: 6.01,
      SwerveModuleGearKit.ExtraHigh2: 6.32,
      SwerveModuleGearKit.ExtraHigh3: 6.77,
      SwerveModuleGearKit.ExtraHigh4: 7.22,
      SwerveModuleGearKit.ExtraHigh5: 7.60
    },
    MotorModel.NEO: {
      SwerveModuleGearKit.Low: 4.12,
      SwerveModuleGearKit.Medium: 4.46,
      SwerveModuleGearKit.High: 4.80,
      SwerveModuleGearKit.ExtraHigh1: 5.03,
      SwerveModuleGearKit.ExtraHigh2: 5.28,
      SwerveModuleGearKit.ExtraHigh3: 5.66,
      SwerveModuleGearKit.ExtraHigh4: 6.04,
      SwerveModuleGearKit.ExtraHigh5: 6.36
    }
  }
