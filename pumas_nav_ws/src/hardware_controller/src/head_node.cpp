// Copyright (C) 2016 Toyota Motor Corporation
// maintainer ry0hei-kobayashi
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <string>
#include <cmath>
#include <algorithm>

class HeadController : public rclcpp::Node
{
public:
  HeadController()
  : Node("head_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Head Controller Node has been started.");

    // Parameters
    head_cmd_topic_          = this->declare_parameter<std::string>(
        "head_cmd_topic",          "/head_trajectory_controller/joint_trajectory");
    head_state_topic_        = this->declare_parameter<std::string>(
        "head_state_topic",        "/head_trajectory_controller/controller_state");
    head_goal_topic_         = this->declare_parameter<std::string>(
        "head_goal_topic",         "/hardware/head/goal_pose");
    head_current_topic_      = this->declare_parameter<std::string>(
        "head_current_topic",      "/hardware/head/current_pose");
    head_goal_reached_topic_ = this->declare_parameter<std::string>(
        "head_goal_reached_topic", "/hardware/head/goal_reached");

    // Publishers
    pub_head_goal_traj_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(head_cmd_topic_, 10);
    pub_head_current_pose_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(head_current_topic_, 10);
    pub_head_goal_reached_ =
        this->create_publisher<std_msgs::msg::Bool>(head_goal_reached_topic_, 10);

    //qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.transient_local();
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Subscribers
    sub_head_goal_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        head_goal_topic_, qos,
        std::bind(&HeadController::headGoalPoseCallback, this, std::placeholders::_1));

    sub_head_state_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        head_state_topic_, 10,
        std::bind(&HeadController::headStateCallback, this, std::placeholders::_1));

    // Init
    head_goal_pose_.assign(2, 0.0f);
    head_current_pose_.assign(2, 0.0f);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HeadController::timerCallback, this));
  }

private:
  static constexpr float PAN_MIN  = -3.141592f;
  static constexpr float PAN_MAX  =  1.74f;
  static constexpr float TILT_MIN = -0.9f;
  static constexpr float TILT_MAX =  0.47f;

  void headStateCallback(
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    // expected: positions.size() >= 2  (0:pan, 1:tilt)
    if (msg->actual.positions.size() >= 2) {
      head_current_pose_[0] = static_cast<float>(msg->actual.positions[0]); // pan
      head_current_pose_[1] = static_cast<float>(msg->actual.positions[1]); // tilt

      std_msgs::msg::Float32MultiArray arr;
      arr.data = head_current_pose_;
      pub_head_current_pose_->publish(arr);
    }
  }

  void headGoalPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(this->get_logger(),
                   "[head_controller] Head goal must be 2 values [pan, tilt] (got %zu)",
                   msg->data.size());
      return;
    }

    float pan  = static_cast<float>(msg->data[0]);
    float tilt = static_cast<float>(msg->data[1]);

    // clamp to limits
    pan  = std::clamp(pan,  PAN_MIN,  PAN_MAX);
    tilt = std::clamp(tilt, TILT_MIN, TILT_MAX);

    head_goal_pose_[0] = pan;
    head_goal_pose_[1] = tilt;
    goal_received_ = true;
    sendHeadGoalTrajectory();
  }

  void timerCallback()
  {
    // reach check
    constexpr float eps_pan  = 0.01f;
    constexpr float eps_tilt = 0.01f;

    bool reached = (std::fabs(head_current_pose_[0] - head_goal_pose_[0]) <= eps_pan) &&
                   (std::fabs(head_current_pose_[1] - head_goal_pose_[1]) <= eps_tilt);

    std_msgs::msg::Bool b;
    b.data = reached;
    pub_head_goal_reached_->publish(b);
  }

  void sendHeadGoalTrajectory()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_pan_joint", "head_tilt_joint"}; 

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = { head_goal_pose_[0], head_goal_pose_[1] };
    p.time_from_start = rclcpp::Duration::from_seconds(0.0);

    traj.points.push_back(p);
    pub_head_goal_traj_->publish(traj);
  }

  std::string head_cmd_topic_;
  std::string head_state_topic_;
  std::string head_goal_topic_;
  std::string head_current_topic_;
  std::string head_goal_reached_topic_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_goal_traj_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       pub_head_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    pub_head_goal_reached_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_head_goal_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_head_state_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> head_goal_pose_;
  std::vector<float> head_current_pose_;
  bool goal_received_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

