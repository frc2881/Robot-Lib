from typing import Optional
from wpimath import units
from wpimath.geometry import Rotation2d, Rotation3d, Pose2d, Transform3d, Translation3d
from robotpy_apriltag import AprilTagFieldLayout
from lib.sensors.pose_sensor import PoseSensor
from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
from photonlibpy.simulation.simCameraProperties import SimCameraProperties


class LocalizationSim:
    def __init__(
        self,
        aprilTagFieldLayout: AprilTagFieldLayout,
        poseSensors: tuple[PoseSensor, ...],
        cameraProperties: Optional[SimCameraProperties] = None,
    ):
        # Create the vision system simulation which handles cameras and targets on the field.
        self._visionSim = VisionSystemSim("main")
        # Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        self._visionSim.addAprilTags(aprilTagFieldLayout)
        # self._visionFieldPublisher = NetworkTableInstance.getDefault().getStructTopic("Vision/Field", Field2d).publish()
        if cameraProperties is None:
            # Use 800x600 OV9281 specs as default
            cameraProperties = SimCameraProperties()
            cameraProperties.setCalibrationFromFOV(800, 600, Rotation2d.fromDegrees(70))
            # Approximate detection noise with average and standard deviation error in pixels.
            cameraProperties.setCalibError(0.35, 0.10)
            # Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProperties.setFPS(20)
            # The average and standard deviation in milliseconds of image data latency.
            cameraProperties.setAvgLatency(units.milliseconds(25))
            cameraProperties.setLatencyStdDev(units.milliseconds(10))

        for sensor in poseSensors:
            # Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
            cameraSim = PhotonCameraSim(sensor._photonCamera, cameraProperties)
            cameraTransform = Transform3d(
                Translation3d(0, 0, 0.5),
                Rotation3d()
            )
            self._visionSim.addCamera(cameraSim, cameraTransform)

    def simulationPeriodic(self, dt: float, pose: Pose2d) -> None:
        self._visionSim.update(pose)