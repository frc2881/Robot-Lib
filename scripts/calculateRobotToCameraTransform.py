from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Quaternion

# Use this script for empirical calculation of robot to camera transform 
# for each pose sensor configuration using fixed target measured 
# from robot and targetPose averages from PhotonVision.

# Steps:
# 1) Place the robot in a fixed measured position with the target camera facing 
# a single AprilTag target.

# 2) Accurately measure the distances from the center of the robot to
# the face of the AprilTag to set the X and Y translation values below.

# 3) Accurately measure the distance from the floor (base of the roobt)
# to the center of the AprilTag to set the Z translation value below.

# 4) Using PhotonVision web UI, confirm that the target AprilTag is being
# detected and the target translation values are being reported correctly.

# 5) Using AdvantageScope, connect to the robot and expand the photonvision
# topic, then expand the camera name being calculated, then expand to the 
# "targetPose", and finally expand both translation and rotation/quaternion. 
# Create a new "Statistics" panel in AdvantageScope with an empty "Measurements" 
# section. Under both translation and rotation sections under targetPose, drag each 
# individual value into the Measurements section so that the calculation of 
# median averages can begin. Wait at least 30 seconds for the Median value for each
# translation (x, y, z) and rotation (w, x, y, z) property to settle. Use each of 
# those Median values to set the appropriate constant value in the script 
# below for target to camera transform calculation.

# 6) After all of the script constant values have been set, save the script
# and open a new Terminal window in VSCode (do not commit the script changes in git). 
# After the Python virtual environment activates, copy, paste, and execute the 
# script with the following command: 
# python .\lib\scripts\calculateRobotToCameraTransform.py

# 7) Copy the Transform3d value that is printed to the terminal console and 
# paste it into the appropriate place in the robot constants for the associated 
# pose sensor configuration being calculated here.

# Example output: 
# Transform3d(
#   Translation3d(x = units.inchesToMeters(12.35), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(11.67)), 
#   Rotation3d(roll = units.degreesToRadians(1.39), pitch = units.degreesToRadians(-15.56), yaw = units.degreesToRadians(-32.15))
# )

# 8) Repeat this same process from step 1 above for all other pose sensor cameras on the robot.

# Note: the constant values below are for example only - see the steps above for setting real/correct values.

# distance from center of AprilTag to center of robot
TARGET_TO_ROBOT_TRANSLATION_X: units.meters = 1.5748

# distance from center of AprilTag to center of robot
TARGET_TO_ROBOT_TRANSLATION_Y: units.meters = -0.381

# distance from center of AprilTag to the floor (will be negative value as the robot is below the AprilTag)
TARGET_TO_ROBOT_TRANSLATION_Z: units.meters = -1.12395 

# rotation of the AprilTag to the robot (e.g. front of robot turned to face toward AprilTag is 180 degrees, rear of robot facing tag will be 0 degrees, sides will be -90/90 accordingly)
TARGET_TO_ROBOT_ROTATION_YAW: units.degrees = 180.0 

# calculated median average values under targetPose for the specific camera as reported by PhotonVision via AdvantageScope statistics
TARGET_TO_CAMERA_TRANSLATION_X = 1.154 
TARGET_TO_CAMERA_TRANSLATION_Y = 0.894
TARGET_TO_CAMERA_TRANSLATION_Z = 0.384
TARGET_TO_CAMERA_QUATERNION_W = 0.386
TARGET_TO_CAMERA_QUATERNION_X = -0.187
TARGET_TO_CAMERA_QUATERNION_Y = 0.062
TARGET_TO_CAMERA_QUATERNION_Z = -0.901

# ===========================================================================

targetToRobot = Transform3d(
  Translation3d(TARGET_TO_ROBOT_TRANSLATION_X, TARGET_TO_ROBOT_TRANSLATION_Y, TARGET_TO_ROBOT_TRANSLATION_Z), 
  Rotation3d().fromDegrees(0, 0, TARGET_TO_ROBOT_ROTATION_YAW)
)

targetToCamera = Transform3d(
  Translation3d(TARGET_TO_CAMERA_TRANSLATION_X, TARGET_TO_CAMERA_TRANSLATION_Y, TARGET_TO_CAMERA_TRANSLATION_Z), 
  Rotation3d(Quaternion(TARGET_TO_CAMERA_QUATERNION_W, TARGET_TO_CAMERA_QUATERNION_X, TARGET_TO_CAMERA_QUATERNION_Y, TARGET_TO_CAMERA_QUATERNION_Z))
)

robotToCamera = Pose3d().transformBy(targetToCamera.inverse()) - Pose3d().transformBy(targetToRobot)

print(f'''Transform3d(
  Translation3d(x = units.inchesToMeters({"%.2f" % round(units.metersToInches(robotToCamera.translation().X()), 2)}), y = units.inchesToMeters({"%.2f" % round(units.metersToInches(robotToCamera.translation().Y()), 2)}), z = units.inchesToMeters({"%.2f" % round(units.metersToInches(robotToCamera.translation().Z()), 2)})), 
  Rotation3d(roll = units.degreesToRadians({"%.2f" % round(units.radiansToDegrees(robotToCamera.rotation().X()), 2)}), pitch = units.degreesToRadians({"%.2f" % round(units.radiansToDegrees(robotToCamera.rotation().Y()), 2)}), yaw = units.degreesToRadians({"%.2f" % round(units.radiansToDegrees(robotToCamera.rotation().Z()), 2)}))
)''')
