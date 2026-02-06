import math
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Translation2d, Rotation2d, Transform2d
from photonlibpy.photonCamera import PhotonCamera
from .. import logger, utils
from ..classes import ObjectSensorConfig, Objects

class ObjectSensor:
  def __init__(
      self, 
      config: ObjectSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Object/{config.name}'
    
    self._photonCamera = PhotonCamera(config.name)
    self._photonCamera.setDriverMode(False)

    self._cameraTranslation = self._config.transform.translation().toTranslation2d()
    self._cameraRotation = self._config.transform.rotation().toRotation2d()
    self._cameraPitch = self._config.transform.rotation().Y()
    self._cameraHeight = self._config.transform.translation().Z()
    self._targetHeightDelta = self._cameraHeight - (self._config.objectHeight / 2)
    
    self._hasTarget = False
    self._pipelineResultBufferTimer = Timer()

    SmartDashboard.putString(f'{self._baseKey}/Stream', config.stream)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getObjects(self) -> Objects | None:
    objects: Objects | None = None
    if self._photonCamera.isConnected():
      photonPipelineResults = self._photonCamera.getAllUnreadResults()
      if len(photonPipelineResults) > 0:
        photonPipelineResult = photonPipelineResults[-1]
        if photonPipelineResult.hasTargets():
          self._hasTarget = True
          targets = photonPipelineResult.getTargets()
          distance = self._targetHeightDelta / math.tan(self._cameraPitch - units.degreesToRadians(targets[0].getPitch()))
          rotation = Rotation2d().fromDegrees(targets[0].getYaw())
          translation = Translation2d(rotation.cos() * distance, rotation.sin() * distance)
          objects = Objects(Transform2d(translation - self._cameraTranslation, rotation - self._cameraRotation), len(targets))
          self._pipelineResultBufferTimer.restart()
        else:
          if self._hasTarget and self._pipelineResultBufferTimer.hasElapsed(0.1):
            self._hasTarget = False
    return objects

  def getCameraName(self) -> str:
    return self._config.name
  
  def hasTarget(self) -> bool:
    return self._hasTarget

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
