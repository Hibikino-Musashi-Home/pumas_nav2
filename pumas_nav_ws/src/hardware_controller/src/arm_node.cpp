// Copyright (C) 2016 Toyota Motor Corporation
// maintainer by ry0hei-kobayashi 
// WIP
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <actionlib_msgs/msg/goal_status.hpp>

#include <string>
#include <cmath>

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Arm Controller Node has been started.");

    // Parameters
    arm_cmd_topic_   = this->declare_parameter<std::string>(
        "arm_cmd_topic",     "/arm_trajectory_controller/joint_trajectory");
    arm_state_topic_ = this->declare_parameter<std::string>(
        "arm_state_topic",   "/arm_trajectory_controller/controller_state");


    // Publishers
    pub_torso_current_pose_ =
        this->create_publisher<std_msgs::msg::Float32>("/hardware/torso/current_pose", 10);
    pub_arm_current_pose_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("/hardware/arm/current_pose", 10);
    pub_arm_goal_pose_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(arm_cmd_topic_, 10);
    pub_arm_goal_reached_ =
        this->create_publisher<std_msgs::msg::Bool>("/hardware/arm/goal_reached", 10);
    pub_torso_goal_reached_ =
        this->create_publisher<std_msgs::msg::Bool>("/hardware/torso/goal_reached", 10);

    // qos
    auto qos_goal = rclcpp::QoS(rclcpp::KeepLast(10))
                    .reliability(rclcpp::ReliabilityPolicy::Reliable)
                    .durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                     .durability(rclcpp::DurabilityPolicy::Volatile);

    // Subscribers
    sub_arm_goal_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/hardware/arm/goal_pose", qos_goal,
        std::bind(&ArmController::armGoalPoseCallback, this, std::placeholders::_1));

    sub_torso_goal_pose_ = this->create_subscription<std_msgs::msg::Float32>(
        "/hardware/torso/goal_pose", qos_goal,
        std::bind(&ArmController::torsoGoalPoseCallback, this, std::placeholders::_1));

    sub_arm_state_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        arm_state_topic_, qos_state,
        std::bind(&ArmController::armCurrentPoseCallback, this, std::placeholders::_1));

    //sub_nav_status_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
    //    "/navigation/status", 10,
    //    [this](const actionlib_msgs::msg::GoalStatus::SharedPtr msg){
    //      if (msg->status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
    //        nav_trigger_ = true;
    //      }
    //    }
    //);
    //bool nav_trigger_ = false;

    // Init
    arm_goal_pose_.assign(4, 0.0f);
    arm_current_pose_.assign(4, 0.0f);
    torso_goal_pose_ = 0.0f;
    torso_current_pose_ = 0.0f;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArmController::timerCallback, this));
  }

private:

  std::string arm_cmd_topic_;
  std::string arm_state_topic_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               pub_torso_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     pub_arm_current_pose_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_goal_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  pub_arm_goal_reached_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  pub_torso_goal_reached_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_arm_goal_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            sub_torso_goal_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_arm_state_;

  rclcpp::TimerBase::SharedPtr timer_;

  float torso_goal_pose_{0.0f};
  float torso_current_pose_{0.0f};
  std::vector<float> arm_goal_pose_;
  std::vector<float> arm_current_pose_;

  bool msg_arm_received_{false};
  bool msg_torso_received_{false};

  double torso_default_pose_{0.0};
  std::vector<double> arm_default_pose_{0.0, -1.57, -1.57, 0.0};

  bool default_send_once_{true};

  void armCurrentPoseCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
  
    torso_current_pose_ = static_cast<float>(msg->feedback.positions[0]); // arm_lift_joint
  
    if (arm_current_pose_.size() != 4) arm_current_pose_.assign(4, 0.0f);
    arm_current_pose_[0] = static_cast<float>(msg->feedback.positions[1]); // arm_flex_joint
    arm_current_pose_[1] = static_cast<float>(msg->feedback.positions[2]); // arm_roll_joint
    arm_current_pose_[2] = static_cast<float>(msg->feedback.positions[3]); // wrist_flex_joint
    arm_current_pose_[3] = static_cast<float>(msg->feedback.positions[4]); // wrist_roll_joint
  
    // Publish current
    std_msgs::msg::Float32 torso_msg; torso_msg.data = torso_current_pose_;
    pub_torso_current_pose_->publish(torso_msg);
  
    std_msgs::msg::Float32MultiArray arm_msg; arm_msg.data = arm_current_pose_;
    pub_arm_current_pose_->publish(arm_msg);

    if (default_send_once_) {
      publish_default_pose();
      default_send_once_ = false;
    }

  }

  void armGoalPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "arm_node.->Arm data must be 4 values (got %zu)",
                   msg->data.size());
      return;
    }

    arm_goal_pose_ = std::vector<float>(msg->data.begin(), msg->data.end());
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received arm goal pose: [%f, %f, %f, %f]",
                 arm_goal_pose_[0], arm_goal_pose_[1], arm_goal_pose_[2], arm_goal_pose_[3]);
    msg_arm_received_ = true;
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received arm goal pose.");
    sendArmGoalTrajectory();
  }

  void torsoGoalPoseCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    torso_goal_pose_ = msg->data;
    msg_torso_received_ = true;
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received torso goal pose.");
  }

  void timerCallback()
  {
    constexpr float eps_arm = 0.01f;
    constexpr float eps_torso = 0.005f;

    bool arm_reached = true;
    for (size_t i = 0; i < 4; ++i) {
      if (std::fabs(arm_current_pose_[i] - arm_goal_pose_[i]) > eps_arm) {
        arm_reached = false;
        break;
      }
    }

    bool torso_reached = (std::fabs(torso_current_pose_ - torso_goal_pose_) <= eps_torso);

    std_msgs::msg::Bool b;
    b.data = arm_reached;
    pub_arm_goal_reached_->publish(b);

    b.data = torso_reached;
    pub_torso_goal_reached_->publish(b);

    //if (nav_trigger_){
    //    nav_trigger_ = false;
    //    publish_default_pose();
    //}
  }

  void publish_default_pose()
  {
    torso_goal_pose_ = static_cast<float>(torso_default_pose_);
    arm_goal_pose_.resize(4);
    arm_goal_pose_[0] = static_cast<float>(arm_default_pose_[0]);
    arm_goal_pose_[1] = static_cast<float>(arm_default_pose_[1]);
    arm_goal_pose_[2] = static_cast<float>(arm_default_pose_[2]);
    arm_goal_pose_[3] = static_cast<float>(arm_default_pose_[3]);

    sendArmGoalTrajectory();

    RCLCPP_INFO(this->get_logger(),
      "arm_node.->Publish Default Arm Joints: torso=%.3f arm=[%.3f %.3f %.3f %.3f]",
      torso_goal_pose_, arm_goal_pose_[0], arm_goal_pose_[1], arm_goal_pose_[2], arm_goal_pose_[3]);

  }

  void sendArmGoalTrajectory()
  {
  
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "arm_lift_joint","arm_flex_joint","arm_roll_joint","wrist_flex_joint","wrist_roll_joint"
    };
    traj.points.resize(1);
    auto &pt = traj.points[0];
    pt.positions.resize(5);
  
    pt.positions[0] = static_cast<double>(torso_goal_pose_);  // arm_lift_joint
    pt.positions[1] = static_cast<double>(arm_goal_pose_[0]); // arm_flex_joint
    pt.positions[2] = static_cast<double>(arm_goal_pose_[1]); // arm_roll_joint
    pt.positions[3] = static_cast<double>(arm_goal_pose_[2]); // wrist_flex_joint   
    pt.positions[4] = static_cast<double>(arm_goal_pose_[3]); // wrist_roll_joint
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0); 
  
    pub_arm_goal_pose_->publish(traj);
  
    msg_arm_received_   = false;
    msg_torso_received_ = false;
  
    RCLCPP_INFO(this->get_logger(),
      "arm_node.->Publish Arm Joints: torso=%.3f arm=[%.3f %.3f %.3f %.3f]",
      pt.positions[0], pt.positions[1], pt.positions[2], pt.positions[3], pt.positions[4]);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto arm_controller_node = std::make_shared<ArmController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_controller_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
