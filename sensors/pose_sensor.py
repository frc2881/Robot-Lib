from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, EstimatedRobotPose
from .. import logger, utils
from ..classes import PoseSensorConfig

class PoseSensor:
  def __init__(
      self, 
      config: PoseSensorConfig
    ) -> None:
    self._baseKey = f'Robot/Sensors/Pose/{config.location.name}'

    self._photonCamera = PhotonCamera(config.location.name)
    self._photonCamera.setDriverMode(False)
    self._photonPoseEstimator = PhotonPoseEstimator(
      config.aprilTagFieldLayout, 
      config.poseStrategy, 
      self._photonCamera, 
      config.cameraTransform
    )
    self._photonPoseEstimator.multiTagFallbackStrategy = config.fallbackPoseStrategy

    self._hasTarget = False
    
    utils.addRobotPeriodic(self._updateTelemetry)

  def getEstimatedRobotPose(self) -> EstimatedRobotPose | None:
    self._hasTarget = False
    if self._photonCamera.isConnected():
      photonPipelineResult = self._photonCamera.getLatestResult()
      self._hasTarget = photonPipelineResult.hasTargets()
      if self._hasTarget:
        return self._photonPoseEstimator.update(photonPipelineResult)
    return None
  
  def hasTarget(self) -> bool:
    return self._hasTarget
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
