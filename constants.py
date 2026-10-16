from wpimath import units
from .classes import MotorModel, SwerveDriveModuleGearKit

class Motors:
  FREE_SPEEDS: dict[MotorModel, units.revolutions_per_minute] = {
    MotorModel.NEO: 5676.0,
    MotorModel.NEO_VORTEX: 6784.0,
    MotorModel.NEO_550: 11000.0
  }

class Drive:
  class Swerve:
    GEAR_RATIOS: dict[SwerveDriveModuleGearKit, float] = {
      SwerveDriveModuleGearKit.LOW: 5.50,        # 12 / 22 
      SwerveDriveModuleGearKit.MEDIUM: 5.08,     # 13 / 22
      SwerveDriveModuleGearKit.HIGH: 4.71,       # 14 / 22
      SwerveDriveModuleGearKit.EXTRA_HIGH_1: 4.50, # 14 / 21
      SwerveDriveModuleGearKit.EXTRA_HIGH_2: 4.29, # 14 / 20
      SwerveDriveModuleGearKit.EXTRA_HIGH_3: 4.00, # 15 / 20
      SwerveDriveModuleGearKit.EXTRA_HIGH_4: 3.75, # 16 / 20
      SwerveDriveModuleGearKit.EXTRA_HIGH_5: 3.56  # 16 / 19
    }

    FREE_SPEEDS: dict[MotorModel, dict[SwerveDriveModuleGearKit, units.meters_per_second]] = {
      MotorModel.NEO_VORTEX: {
        SwerveDriveModuleGearKit.LOW: 4.92,
        SwerveDriveModuleGearKit.MEDIUM: 5.33,
        SwerveDriveModuleGearKit.HIGH: 5.74,
        SwerveDriveModuleGearKit.EXTRA_HIGH_1: 6.01,
        SwerveDriveModuleGearKit.EXTRA_HIGH_2: 6.32,
        SwerveDriveModuleGearKit.EXTRA_HIGH_3: 6.77,
        SwerveDriveModuleGearKit.EXTRA_HIGH_4: 7.22,
        SwerveDriveModuleGearKit.EXTRA_HIGH_5: 7.60
      },
      MotorModel.NEO: {
        SwerveDriveModuleGearKit.LOW: 4.12,
        SwerveDriveModuleGearKit.MEDIUM: 4.46,
        SwerveDriveModuleGearKit.HIGH: 4.80,
        SwerveDriveModuleGearKit.EXTRA_HIGH_1: 5.03,
        SwerveDriveModuleGearKit.EXTRA_HIGH_2: 5.28,
        SwerveDriveModuleGearKit.EXTRA_HIGH_3: 5.66,
        SwerveDriveModuleGearKit.EXTRA_HIGH_4: 6.04,
        SwerveDriveModuleGearKit.EXTRA_HIGH_5: 6.36
      }
    }
