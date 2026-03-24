from typing import Optional
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, EstimatedRobotPose
from .. import logger, utils
from ..classes import PoseSensorConfig, PoseSensorResult

class PoseSensor:
  def __init__(
      self, 
      config: PoseSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Pose/{config.name}'

    self._photonCamera = PhotonCamera(config.name)
    self._photonCamera.setDriverMode(False)
    self._photonPoseEstimator = PhotonPoseEstimator(
      config.aprilTagFieldLayout, 
      config.transform
    )

    SmartDashboard.putString(f'{self._baseKey}/Stream', config.stream)

    self._photonCamera.getAllUnreadResults()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getEstimatedRobotPose(self) -> Optional[EstimatedRobotPose]:
    estimatedRobotPose: Optional[EstimatedRobotPose] = None
    if self._photonCamera.isConnected():
      photonPipelineResults = self._photonCamera.getAllUnreadResults()
      if len(photonPipelineResults) > 0:
        photonPipelineResult = photonPipelineResults[-1]
        estimatedRobotPose = self._photonPoseEstimator.estimateCoprocMultiTagPose(photonPipelineResult)
        if estimatedRobotPose is None:
          estimatedRobotPose = self._photonPoseEstimator.estimateLowestAmbiguityPose(photonPipelineResult)
    return estimatedRobotPose
  
  def getLatestResult(self) -> Optional[PoseSensorResult]:
    poseSensorResult: Optional[PoseSensorResult] = None
    if self._photonCamera.isConnected():
      photonPipelineResults = self._photonCamera.getAllUnreadResults()
      if len(photonPipelineResults) > 0:
        photonPipelineResult = photonPipelineResults[-1]
        if photonPipelineResult.hasTargets():
          estimatedRobotPose = self._photonPoseEstimator.estimateCoprocMultiTagPose(photonPipelineResult)
          if estimatedRobotPose is None:
            estimatedRobotPose = self._photonPoseEstimator.estimateLowestAmbiguityPose(photonPipelineResult)
          if estimatedRobotPose is not None:
            poseSensorResult = PoseSensorResult(
              timestamp = estimatedRobotPose.timestampSeconds,
              estimatedPose = estimatedRobotPose.estimatedPose,
              bestTargetReprojectionError = photonPipelineResult.multitagResult.estimatedPose.bestReprojErr if photonPipelineResult.multitagResult is not None else -1,
              bestTargetAmbiguity = -1 if photonPipelineResult.multitagResult is not None else photonPipelineResult.getBestTarget().poseAmbiguity,
              bestTargetDistance = photonPipelineResult.getBestTarget().bestCameraToTarget.translation().norm()
            )
    return poseSensorResult

  def getCameraName(self) -> str:
    return self._photonCamera.getName()

  def isConnected(self) -> bool:
    return self._photonCamera.isConnected()

  def hasTarget(self) -> bool:
    return self._photonCamera._cameraTable.getBoolean("hasTarget", False)
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
