from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Rotation3d, Rotation2d
from photonlibpy.targeting import PhotonTrackedTarget
from photonlibpy.photonCamera import PhotonCamera
from .. import logger, utils
from ..classes import ObjectSensorConfig

class ObjectSensor:
  def __init__(
      self, 
      config: ObjectSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Object/{config.cameraName}'
    
    self._photonCamera = PhotonCamera(config.cameraName)

    self._objectTransform = Transform3d
    self._hasTarget = False

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateObjectTransform()
    self._updateTelemetry()

  def _updateObjectTransform(self) -> None:
    objectTransform = Transform3d()
    hasTarget = False
    if self._photonCamera.isConnected():
      results = self._photonCamera.getAllUnreadResults()
      if len(results) > 0:
        result = results[-1]
        target = result.getBestTarget()
        if target is not None:
          hasTarget = True

          # TODO: replace hard-coded/temp linear interpolation for note objects with generalized homography calculation for coral/algae
          distances = (2.275, 1.783, 1.581, 1.097, 0.732, 0.424, 0.0)
          areas = (0.01, 1.2, 1.6, 3.2, 6.8, 18, 99)
          area = target.getArea()
          if area > 0:
            distance = utils.getInterpolatedValue(area, areas, distances)
            yaw = -target.getYaw() + 185.0
            objectTransform = Transform3d(distance, 0, 0, Rotation3d(Rotation2d.fromDegrees(yaw)))
            
    self._hasTarget = hasTarget
    self._objectTransform = objectTransform

  def getObjectTransform(self) -> Transform3d:
    return self._objectTransform

  def hasTarget(self) -> bool:
    return self._hasTarget

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
    SmartDashboard.putString(f'{self._baseKey}/Transform', utils.toJson(self.getObjectTransform()))
