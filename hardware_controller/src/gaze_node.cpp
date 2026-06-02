#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pumas_interfaces/action/gaze_head.hpp>

#include <string>
#include <cmath>
#include <thread>
#include <chrono>


class GazeController : public rclcpp::Node
{
public:
  using GazeHead       = pumas_interfaces::action::GazeHead;
  using GoalHandleGaze = rclcpp_action::ServerGoalHandle<GazeHead>;

  GazeController()
  : Node("gaze_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<bool>(        "use_namespace",  false);
    this->declare_parameter<std::string>( "gaze_tf_name",   "gaze_point");
    this->declare_parameter<std::string>( "base_link_name", "base_footprint");
    this->declare_parameter<double>(      "head_height",    1.1);

    this->get_parameter("use_namespace",  use_namespace_);
    this->get_parameter("gaze_tf_name",   gaze_tf_name_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("head_height",    head_height_);

    pub_head_goal_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      make_name("/hardware/head/goal_pose"),
      rclcpp::QoS(10).reliable());

    // Action server on /gaze_head.
    // execute() runs in a detached thread so it can block without starving
    // the TF listener subscription that the MultiThreadedExecutor serves.
    action_server_ = rclcpp_action::create_server<GazeHead>(
      this,
      "gaze_head",
      std::bind(&GazeController::handle_goal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&GazeController::handle_cancel,   this, std::placeholders::_1),
      std::bind(&GazeController::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Action server ready on /gaze_head");
  }

private:
  bool        use_namespace_ = false;
  std::string gaze_tf_name_;
  std::string base_link_name_;
  double      head_height_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_head_goal_pose_;
  rclcpp_action::Server<GazeHead>::SharedPtr action_server_;

  std::string make_name(const std::string &suffix) const
  {
    std::string sfx = suffix;
    if (!sfx.empty() && sfx.front() != '/')
      sfx = "/" + sfx;

    std::string name;
    if (use_namespace_) {
      name = this->get_namespace() + sfx;
      if (name.size() > 1 && name[0] == '/' && name[1] == '/')
        name.erase(0, 1);
    } else {
      name = sfx;
    }
    return name;
  }

  // Compute [pan, tilt] toward gaze_tf_name in the map frame.
  // Returns false (and leaves msg unchanged) when any TF lookup fails.
  bool gaze_point_to_head_angles(
    std_msgs::msg::Float32MultiArray &msg,
    const std::string &gaze_tf_name)
  {
    try {
      geometry_msgs::msg::TransformStamped robot_tf =
        tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

      float robot_x = static_cast<float>(robot_tf.transform.translation.x);
      float robot_y = static_cast<float>(robot_tf.transform.translation.y);

      tf2::Quaternion q(
        robot_tf.transform.rotation.x,
        robot_tf.transform.rotation.y,
        robot_tf.transform.rotation.z,
        robot_tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      float robot_t = static_cast<float>(yaw);

      geometry_msgs::msg::TransformStamped gaze_tf =
        tf_buffer_.lookupTransform("map", gaze_tf_name, tf2::TimePointZero);

      float gaze_x = static_cast<float>(gaze_tf.transform.translation.x);
      float gaze_y = static_cast<float>(gaze_tf.transform.translation.y);
      float gaze_z = static_cast<float>(gaze_tf.transform.translation.z);

      float pan = atan2(gaze_y - robot_y, gaze_x - robot_x) - robot_t;
      if (pan >  M_PI) pan -= 2.0f * M_PI;
      if (pan <= -M_PI) pan += 2.0f * M_PI;

      float dx     = gaze_x - robot_x;
      float dy     = gaze_y - robot_y;
      float h_dist = std::sqrt(dx * dx + dy * dy);
      float tilt   = std::atan2(gaze_z - static_cast<float>(head_height_), h_dist);

      msg.data = {pan, tilt};
      return true;
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "GazeController.-> TF lookup failed: %s", ex.what());
      return false;
    }
  }

  // ---------- Action server callbacks ----------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GazeHead::Goal> goal)
  {
    const std::string tf_name =
      goal->gaze_tf_name.empty() ? gaze_tf_name_ : goal->gaze_tf_name;
    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Goal received. TF target: '%s'", tf_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGaze>)
  {
    RCLCPP_INFO(this->get_logger(), "GazeController.-> Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGaze> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Main gaze loop. Runs in a detached thread; publishes head angle commands
  // at 100 ms intervals until canceled or the node shuts down.
  void execute(const std::shared_ptr<GoalHandleGaze> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string tf_name =
      goal->gaze_tf_name.empty() ? gaze_tf_name_ : goal->gaze_tf_name;

    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Gaze started. Target TF: '%s'", tf_name.c_str());

    auto feedback = std::make_shared<GazeHead::Feedback>();
    auto result   = std::make_shared<GazeHead::Result>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "GazeController.-> Gaze canceled.");
        // Final head command (after the gaze loop has stopped publishing), so
        // the head settles at the requested restore pose, not the gaze target.
        publish_restore_head(goal);
        return;
      }

      std_msgs::msg::Float32MultiArray msg;
      if (gaze_point_to_head_angles(msg, tf_name)) {
        pub_head_goal_pose_->publish(msg);
        feedback->pan  = msg.data[0];
        feedback->tilt = msg.data[1];
        goal_handle->publish_feedback(feedback);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = false;
    goal_handle->abort(result);
    publish_restore_head(goal);
  }

  // Publish the restore head once as the final head command when gaze ends.
  void publish_restore_head(const std::shared_ptr<const GazeHead::Goal> &goal)
  {
    if (!goal->restore_head)
      return;
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {goal->restore_pan, goal->restore_tilt};
    pub_head_goal_pose_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Restored head to [%.3f, %.3f] on gaze end.",
      goal->restore_pan, goal->restore_tilt);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
