# Goal constraints
tmc_planning_msgs/TsrLinkConstraint[] hard_link_constraints
tmc_planning_msgs/RangeJointConstraint[] hard_joint_constraints

# Path constraints(Soft)
tmc_planning_msgs/TsrLinkConstraint[] soft_link_constraints
tmc_planning_msgs/RangeJointConstraint[] soft_joint_constraints

# Path constraints(Hard)
tmc_planning_msgs/LinearConstraint goal_relative_linear_constraint
tmc_planning_msgs/TsrLinkConstraint[] hard_path_link_constraints

# Path constraints(Soft)
tmc_planning_msgs/RangeJointConstraint[] soft_path_joint_constraints
