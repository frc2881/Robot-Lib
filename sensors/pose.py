from wpilib import SmartDashboard, Timer
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, EstimatedRobotPose
from .. import logger, utils
from ..classes import PoseSensorConfig

class PoseSensor:
  def __init__(
      self, 
      config: PoseSensorConfig
    ) -> None:
    self._config = config
    self._baseKey = f'Robot/Sensors/Pose/{config.cameraName}'

    self._photonCamera = PhotonCamera(config.cameraName)
    self._photonCamera.setDriverMode(False)
    self._photonPoseEstimator = PhotonPoseEstimator(
      config.constants.aprilTagFieldLayout, 
      config.constants.poseStrategy, 
      self._photonCamera, 
      config.cameraTransform
    )
    self._photonPoseEstimator.multiTagFallbackStrategy = config.constants.fallbackPoseStrategy

    self._hasTarget = False
    self._pipelineResultBufferTimer = Timer()

    SmartDashboard.putString(f'{self._baseKey}/Stream', config.stream)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getEstimatedRobotPose(self) -> EstimatedRobotPose | None:
    estimatedRobotPose: EstimatedRobotPose | None = None
    if self._photonCamera.isConnected():
      for photonPipelineResult in self._photonCamera.getAllUnreadResults():
        estimatedRobotPose = self._photonPoseEstimator.update(photonPipelineResult)
    if estimatedRobotPose is not None:
      self._hasTarget = len(estimatedRobotPose.targetsUsed) > 0
      self._pipelineResultBufferTimer.restart()
    else: 
      if self._hasTarget and self._pipelineResultBufferTimer.hasElapsed(0.1):
        self._hasTarget = False
    return estimatedRobotPose
  
  def getCameraName(self) -> str:
    return self._config.cameraName

  def hasTarget(self) -> bool:
    return self._hasTarget
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
