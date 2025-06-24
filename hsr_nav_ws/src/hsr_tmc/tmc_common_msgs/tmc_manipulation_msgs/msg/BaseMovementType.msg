# This message defines types of robot base movement

# Movement type
int32 val

# The base does not move
int32 NONE=0
# The base moves without constraints [x, y, z, roll, pitch, yaw]
int32 FLOAT=1
# The base moves on the plane [x, y, yaw]
int32 PLANAR=2
# The base moves on the x-axis
int32 RAIL_X=3
# The base moves on the y-axis
int32 RAIL_Y=4
# The base moves on the z-axis
int32 RAIL_Z=5
# The base is rotated around the x-axis
int32 ROTATION_X=6
# The base is rotated around the y-axis
int32 ROTATION_Y=7
# The base is rotated around the z-axis
int32 ROTATION_Z=8
