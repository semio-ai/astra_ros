uint8 STATUS_NOTTRACKING = 0
uint8 STATUS_CANDIDATE = 1
uint8 STATUS_TRACKING = 2
uint8 STATUS_LOST = 3

int32 tracking_id
uint8 status
geometry_msgs/Vector3 world_position
geometry_msgs/Vector3 world_delta_position
astra_ros/Vector2i depth_position