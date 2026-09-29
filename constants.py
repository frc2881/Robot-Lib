from wpimath import units
from .classes import MotorModel, SwerveDriveModuleGearKit

class Motors:
  FREE_SPEEDS: dict[MotorModel, units.revolutions_per_minute] = {
    MotorModel.NEO: 5676.0,
    MotorModel.NEOVortex: 6784.0,
    MotorModel.NEO550: 11000.0
  }

class Drive:
  class Swerve:
    GEAR_RATIOS: dict[SwerveDriveModuleGearKit, float] = {
      SwerveDriveModuleGearKit.Low: 5.50,        # 12 / 22 
      SwerveDriveModuleGearKit.Medium: 5.08,     # 13 / 22
      SwerveDriveModuleGearKit.High: 4.71,       # 14 / 22
      SwerveDriveModuleGearKit.ExtraHigh1: 4.50, # 14 / 21
      SwerveDriveModuleGearKit.ExtraHigh2: 4.29, # 14 / 20
      SwerveDriveModuleGearKit.ExtraHigh3: 4.00, # 15 / 20
      SwerveDriveModuleGearKit.ExtraHigh4: 3.75, # 16 / 20
      SwerveDriveModuleGearKit.ExtraHigh5: 3.56  # 16 / 19
    }

    FREE_SPEEDS: dict[MotorModel, dict[SwerveDriveModuleGearKit, units.meters_per_second]] = {
      MotorModel.NEOVortex: {
        SwerveDriveModuleGearKit.Low: 4.92,
        SwerveDriveModuleGearKit.Medium: 5.33,
        SwerveDriveModuleGearKit.High: 5.74,
        SwerveDriveModuleGearKit.ExtraHigh1: 6.01,
        SwerveDriveModuleGearKit.ExtraHigh2: 6.32,
        SwerveDriveModuleGearKit.ExtraHigh3: 6.77,
        SwerveDriveModuleGearKit.ExtraHigh4: 7.22,
        SwerveDriveModuleGearKit.ExtraHigh5: 7.60
      },
      MotorModel.NEO: {
        SwerveDriveModuleGearKit.Low: 4.12,
        SwerveDriveModuleGearKit.Medium: 4.46,
        SwerveDriveModuleGearKit.High: 4.80,
        SwerveDriveModuleGearKit.ExtraHigh1: 5.03,
        SwerveDriveModuleGearKit.ExtraHigh2: 5.28,
        SwerveDriveModuleGearKit.ExtraHigh3: 5.66,
        SwerveDriveModuleGearKit.ExtraHigh4: 6.04,
        SwerveDriveModuleGearKit.ExtraHigh5: 6.36
      }
    }
