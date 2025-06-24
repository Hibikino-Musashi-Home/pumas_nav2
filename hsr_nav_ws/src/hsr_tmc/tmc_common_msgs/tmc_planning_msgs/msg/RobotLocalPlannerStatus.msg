std_msgs/Header header

int32 planner_status

int32 SUCCESS=1
int32 CONSTRAINTS_EMPTY=-1
int32 GENERATION_FAILURE=-2
int32 EVALUATION_FAILURE=-3
int32 VALIDATION_FAILURE=-4
int32 OPTIMIZATION_FAILURE=-5

ConstraintsStatus[] constraints_statuses