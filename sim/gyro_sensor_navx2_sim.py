from wpimath.geometry import  Rotation2d, Rotation3d
from wpilib.simulation import SimDeviceSim
from wpimath import units

class AHRSSim:
  """Class to control a simulated AHRS NavX2 Gyro"""
  def __init__(self):
    self._simDevice = SimDeviceSim("navX-Sensor[2]")
    # See the docs for a current list of supported simulation variables -
    # https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
    self._connected = self._simDevice.getBoolean("Connected")
    self._rate = self._simDevice.getDouble("Rate")
    self._yaw = self._simDevice.getDouble("Yaw")
    self._pitch = self._simDevice.getDouble("Pitch")
    self._roll = self._simDevice.getDouble("Roll")
    self._compassHeading = self._simDevice.getDouble("CompassHeading")
    # The following are currently unused, provided for completeness
    self._fusedHeading = self._simDevice.getDouble("FusedHeading")
    self._linearWorldAccelX = self._simDevice.getDouble("LinearWorldAccelX")
    self._linearWorldAccelY = self._simDevice.getDouble("LinearWorldAccelY")
    self._linearWorldAccelZ = self._simDevice.getDouble("LinearWorldAccelZ")
    # Quaternion W
    # Quaternion X
    # Quaternion Y
    # Quaternion Z
    # Velocity X
    # Velocity Y
    # Velocity Z
    # Rate

    self._connected.set(True)

  def setConnected(self, connected: bool) -> None:
    self._connected.set(connected)
    
  def setHeading(self, heading: units.degrees) -> None:
    self._yaw.set(-heading)
    self._compassHeading.set(-heading)

  def setRotation(self, rotation: Rotation2d) -> None:
    self.setHeading(rotation.degrees())
    
  def setRotation3d(self, rotation: Rotation3d) -> None:
    """Sets pitch, roll and yaw using the provided Rotation3d"""
    self._roll.set(rotation.x_degrees)
    self._pitch.set(rotation.y_degrees)
    self.setHeading(rotation.z_degrees)

  def setTurnRate(self, rate: units.degrees_per_second) -> None:
    self._rate.set(rate)