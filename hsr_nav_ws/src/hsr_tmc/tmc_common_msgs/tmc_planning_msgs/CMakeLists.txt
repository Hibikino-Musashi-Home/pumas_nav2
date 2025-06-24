cmake_minimum_required(VERSION 3.8)
project(tmc_planning_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tmc_manipulation_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/EvaluateRobotTrajectories.action"
  "action/GenerateRobotTrajectories.action"
  "action/OptimizeRobotTrajectory.action"
  "action/ValidateRobotTrajectories.action"
  "msg/Constraints.msg"
  "msg/ConstraintsStatus.msg"
  "msg/JointDisplacement.msg"
  "msg/JointPosition.msg"
  "msg/JointWeight.msg"
  "msg/LinearConstraint.msg"
  "msg/LinkDisplacement.msg"
  "msg/ObjectPair.msg"
  "msg/ObjectPairsList.msg"
  "msg/RangeJointConstraint.msg"
  "msg/RobotDisplacements.msg"
  "msg/RobotLocalGoal.msg"
  "msg/RobotLocalPlannerStatus.msg"
  "msg/TaskSpaceRegion.msg"
  "msg/TsrLinkConstraint.msg"
  "srv/PlanWithHandGoals.srv"
  "srv/PlanWithHandLine.srv"
  "srv/PlanWithJointGoals.srv"
  "srv/PlanWithTsrConstraints.srv"
  "srv/UpdateJointWeights.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs moveit_msgs std_msgs tmc_manipulation_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
