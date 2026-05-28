#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <cmath>

class GazeController : public rclcpp::Node
{
public:
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
      rclcpp::QoS(10).transient_local());

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GazeController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "GazeController.-> Node has been started.");
  }

private:
  bool        use_namespace_ = false;
  std::string gaze_tf_name_;
  std::string base_link_name_;
  double      head_height_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_head_goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  // Look up gaze_tf_name in the map frame and compute [pan, tilt] head angles.
  // Returns false and leaves msg unchanged when any TF lookup fails.
  bool gaze_point_to_goal_head_angles(
    std_msgs::msg::Float32MultiArray &msg,
    const std::string &gaze_tf_name = "gaze_point")
  {
    try {
      // Robot pose in map frame
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

      // Gaze point in map frame
      geometry_msgs::msg::TransformStamped gaze_tf =
        tf_buffer_.lookupTransform("map", gaze_tf_name, tf2::TimePointZero);

      float gaze_x = static_cast<float>(gaze_tf.transform.translation.x);
      float gaze_y = static_cast<float>(gaze_tf.transform.translation.y);
      float gaze_z = static_cast<float>(gaze_tf.transform.translation.z);

      // Pan: horizontal angle from robot heading toward gaze point
      float pan = atan2(gaze_y - robot_y, gaze_x - robot_x) - robot_t;
      if (pan >  M_PI) pan -= 2.0f * M_PI;
      if (pan <= -M_PI) pan += 2.0f * M_PI;

      // Tilt: vertical angle from head height toward gaze point
      float dx = gaze_x - robot_x;
      float dy = gaze_y - robot_y;
      float h_dist = std::sqrt(dx * dx + dy * dy);
      float tilt = std::atan2(gaze_z - static_cast<float>(head_height_), h_dist);

      msg.data = {pan, tilt};
      return true;
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "GazeController.-> TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void timerCallback()
  {
    std_msgs::msg::Float32MultiArray msg;
    if (gaze_point_to_goal_head_angles(msg, gaze_tf_name_))
      pub_head_goal_pose_->publish(msg);
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
