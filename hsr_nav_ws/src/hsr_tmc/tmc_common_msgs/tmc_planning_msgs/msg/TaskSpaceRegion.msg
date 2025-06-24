# This message describe TSR(Task Space Region)

# Name of end effector frame
string end_frame_id

# Origin to TSR pose
geometry_msgs/Pose origin_to_tsr

# TSR to end effector pose
geometry_msgs/Pose tsr_to_end

# Minimum boundary of TSR
float64[6] min_bounds

# Maximum boundary of TSR
float64[6] max_bounds

# Flag to perform rotation first when transformation from TSR to sampled
bool rotation_first
