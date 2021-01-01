uint8 HAND_POSE_UNKNOWN = 0
uint8 HAND_POSE_GRIP = 1

uint8 STATUS_NOT_TRACKING = 0
uint8 STATUS_LOST = 1
uint8 STATUS_TRACKING_STARTED = 2
uint8 STATUS_TRACKING = 3

Header header

# Persistent ID of the body
uint8 id

# One of the STATUS_* values defined in this message
uint8 status

bool is_tracking_joints
bool is_tracking_hand_poses

# The center of mass of the humanoid
geometry_msgs/Vector3 center_of_mass

# The joints of the humanoid
astra_ros/Joint[] joints

# Only available if color and registered depth images are
# available.
sensor_msgs/PointCloud point_cloud

# One of the HAND_POSE_* values defined in this message
uint8 left_hand_pose
uint8 right_hand_pose