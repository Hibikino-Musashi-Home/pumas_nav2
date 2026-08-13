# Atomic motion goal published by motion_synth: a name-keyed joint snapshot +
# the time_from_start to use when the hardware controllers forward the
# trajectory. arm_controller/head_controller each look up the joint names
# they care about (see their arm_default_names/head_default_names params)
# instead of depending on a fixed field layout.

sensor_msgs/JointState joints
# time_from_start (sec). 0.0 → controller default.
float32 motion_execution_time
