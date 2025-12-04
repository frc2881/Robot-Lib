from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Quaternion

"""
Use this script for empirical calculation of robot to camera transform 
for each pose sensor configuration using fixed target measured 
from robot and targetPose averages from PhotonVision.

Steps:
1) Place the robot in a fixed position with the target camera facing 
a single AprilTag target with the center of robot precisely aligned 
with the center of the AprilTag (so that the Y translation below is zero)

2) Accurately measure the distance from the center of the robot to
the face of the AprilTag to set the X translation value below

3) Accurately measure the distance from the floor (base of the roobt)
to the center of the AprilTag to set the Z translation value below

4) Using PhotonVision web UI, confirm that the target AprilTag is being
detected and the camera to target translation values are being reported

5) Using AdvantageScope, connect to the robot and navigate to the "targetPose"
topic category for the camera transform being calculated. Create a new
"Statistics" tab with the "Measurements" section clear for new entries.
Under both translation and rotation sections under targetPose, drag each
individual value into the Measurements section so that the calculation of
averages can begin. Wait at least 30 seconds for the Median value for each
translation and rotation property to settle. Use each of those Median values
to set the appropriate constant value below for target to camera transform.

6) After all of the script constant values have been set, save the script
and open a new Terminal window in VSCode. After the Python virtual environment 
activates, run the script with the following command (without quotes):
"python .\lib\scripts\calculateRobotToCameraTransform.py"

7) Copy the Transform3d value that is printed to the terminal console and 
paste it into the appropriate place in the robot constants for the associated 
pose sensor configuration being calculated here.

Example: Transform3d(Translation3d(0.149506, -0.055318, 0.271137), Rotation3d(-0.001852, -0.181301, 0.020370))

8) Repeat this same process from step 1 above for all other pose sensor cameras on the robot

"""

# Note: the constant values below are for example only - see the steps above for setting real/correct values

TARGET_TO_ROBOT_TRANSLATION_X = 0.860 # distance from center of robot to face of AprilTag
TARGET_TO_ROBOT_TRANSLATION_Y = 0.0 # distance from center of robot to center of AprilTag (should be zero)
TARGET_TO_ROBOT_TRANSLATION_Z = -0.311 # distance from floor to center of AprilTag
TARGET_TO_ROBOT_ROTATION_YAW = -180.0 # rotation of camera on robot to AprilTag (e.g. front facing camera/front of robot facing toward AprilTag)

TARGET_TO_CAMERA_TRANSLATION_X = 0.707 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_TRANSLATION_Y = 0.041 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_TRANSLATION_Z = -0.089 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_W = -0.01 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_X = -0.09 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_Y = 0.0 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_QUATERNION_Z = -0.99 # calculated/average median value under targetPose reported by PhotonVision via AdvantageScope statistics

# ===========================================================================

targetToRobot = Transform3d(
  Translation3d(TARGET_TO_ROBOT_TRANSLATION_X, TARGET_TO_ROBOT_TRANSLATION_Y, TARGET_TO_ROBOT_TRANSLATION_Z), 
  Rotation3d().fromDegrees(0, 0, TARGET_TO_ROBOT_ROTATION_YAW)
)

targetToCamera = Transform3d(
  Translation3d(TARGET_TO_CAMERA_TRANSLATION_X, TARGET_TO_CAMERA_TRANSLATION_Y, TARGET_TO_CAMERA_TRANSLATION_Z), 
  Rotation3d(Quaternion(TARGET_TO_CAMERA_QUATERNION_W, TARGET_TO_CAMERA_QUATERNION_X, TARGET_TO_CAMERA_QUATERNION_Y, TARGET_TO_CAMERA_QUATERNION_Z)))

robotToCamera = Pose3d().transformBy(targetToCamera.inverse()) - Pose3d().transformBy(targetToRobot)

constant = str(robotToCamera).replace("x=", "").replace("y=", "").replace("z=", "")

print(constant)
