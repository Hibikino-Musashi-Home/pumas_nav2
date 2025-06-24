/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// Posture change node

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <tmc_utils/caching_subscriber.hpp>

#include "../src/safe_pose_changer.hpp"

namespace {
const std::vector<std::string> kArmJoints =
    {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
const std::vector<std::string> kHeadJoints = {"head_pan_joint", "head_tilt_joint"};

// Turn Spin with a separate thread to get the value of the parameter server
void SpinSomeThread(const rclcpp::Node::SharedPtr& node) {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  for (auto wait_count = 0; wait_count < 20; ++wait_count) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

/// TRAJECTORY contains the joints as expected, and verify whether the final posture is expected.
void ValidateResponse(const trajectory_msgs::msg::JointTrajectory& trajectory,
                      const std::vector<std::string>& expected_joint_names,
                      const std::vector<double>& expected_joint_positions,
                      const double expected_sec = 1.0) {
  ASSERT_FALSE(trajectory.points.empty());
  EXPECT_NEAR(rclcpp::Duration(trajectory.points.back().time_from_start).seconds(), expected_sec, 1.0e-6);
  for (const auto& point : trajectory.points) {
    ASSERT_EQ(point.positions.size(), expected_joint_names.size());
  }

  for (auto expected_index = 0; expected_index < expected_joint_names.size(); ++expected_index) {
    const auto trajectory_name_it =
        std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), expected_joint_names[expected_index]);
    ASSERT_NE(trajectory_name_it, trajectory.joint_names.end());

    const auto trajectory_index = std::distance(trajectory.joint_names.begin(), trajectory_name_it);
    EXPECT_DOUBLE_EQ(trajectory.points.back().positions[trajectory_index], expected_joint_positions[expected_index]);
  }
}

class PlannerServiceStub {
 public:
  explicit PlannerServiceStub(const rclcpp::Node::SharedPtr& node) : is_success_(true) {
    server_ = node->create_service<tmc_planning_msgs::srv::PlanWithJointGoals>(
        "plan_with_joint_goals",
        std::bind(&PlannerServiceStub::Callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  sensor_msgs::msg::JointState input_initial_state() const { return input_initial_state_; }
  moveit_msgs::msg::PlanningSceneWorld input_environment() const { return input_environment_; }
  void set_is_success(bool is_success) { is_success_ = is_success; }

 private:
  rclcpp::Service<tmc_planning_msgs::srv::PlanWithJointGoals>::SharedPtr server_;

  bool is_success_;
  sensor_msgs::msg::JointState input_initial_state_;
  moveit_msgs::msg::PlanningSceneWorld input_environment_;

  void Callback(const tmc_planning_msgs::srv::PlanWithJointGoals::Request::SharedPtr req,
                tmc_planning_msgs::srv::PlanWithJointGoals::Response::SharedPtr res) {
    input_initial_state_ = req->initial_joint_state;
    input_environment_ = req->environment_before_planning;
    if (is_success_ &&
        !req->goal_joint_states.empty() &&
        (req->goal_joint_states[0].position.size() == req->use_joints.size())) {
      res->solution.joint_names = req->use_joints;
      res->solution.points.resize(2);
      // Actually, it should be pulled out from the inn ally_joint_state, but since it is 0, just put 0 in the size.
      res->solution.points[0].positions.resize(req->use_joints.size(), 0.0);
      res->solution.points[1].positions = req->goal_joint_states[0].position;
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    } else {
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    }
  }
};

class FilterServiceStub {
 public:
  explicit FilterServiceStub(const rclcpp::Node::SharedPtr& node) : is_success_(true), clock_(node->get_clock()) {
    node->declare_parameter<double>("velocity_ratio", 0.8);
    node->declare_parameter<double>("acceleration_ratio", 0.9);

    handle_ = node->add_on_set_parameters_callback(
        std::bind(&FilterServiceStub::SetParameterCallback, this, std::placeholders::_1));

    server_ = node->create_service<tmc_manipulation_msgs::srv::FilterJointTrajectory>(
        "timeopt_filter_node/filter_trajectory",
        std::bind(&FilterServiceStub::Callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  sensor_msgs::msg::JointState input_initial_state() const { return input_initial_state_; }
  void set_is_success(bool is_success) { is_success_ = is_success; }

  void ValidateVelocityAndAccelerationUpdate(double velocity_ratio, double acceleration_ratio) {
    ASSERT_EQ(velocity_update_stamp_and_values_.size(), 2);
    EXPECT_LT(std::get<0>(velocity_update_stamp_and_values_[0]), service_call_stamp_);
    EXPECT_LT(service_call_stamp_, std::get<0>(velocity_update_stamp_and_values_[1]));
    EXPECT_DOUBLE_EQ(std::get<1>(velocity_update_stamp_and_values_[0]), velocity_ratio);
    EXPECT_DOUBLE_EQ(std::get<1>(velocity_update_stamp_and_values_[1]), 0.8);

    ASSERT_EQ(acceleration_update_stamp_and_values_.size(), 2);
    EXPECT_LT(std::get<0>(acceleration_update_stamp_and_values_[0]), service_call_stamp_);
    EXPECT_LT(service_call_stamp_, std::get<0>(acceleration_update_stamp_and_values_[1]));
    EXPECT_DOUBLE_EQ(std::get<1>(acceleration_update_stamp_and_values_[0]), acceleration_ratio);
    EXPECT_DOUBLE_EQ(std::get<1>(acceleration_update_stamp_and_values_[1]), 0.9);
  }

 private:
  rclcpp::Service<tmc_manipulation_msgs::srv::FilterJointTrajectory>::SharedPtr server_;

  bool is_success_;
  sensor_msgs::msg::JointState input_initial_state_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr handle_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time service_call_stamp_;
  std::vector<std::tuple<rclcpp::Time, double>> velocity_update_stamp_and_values_;
  std::vector<std::tuple<rclcpp::Time, double>> acceleration_update_stamp_and_values_;

  void Callback(const tmc_manipulation_msgs::srv::FilterJointTrajectory::Request::SharedPtr req,
                tmc_manipulation_msgs::srv::FilterJointTrajectory::Response::SharedPtr res) {
    service_call_stamp_ = clock_->now();
    input_initial_state_ = req->start_state.joint_state;
    if (is_success_ && req->trajectory.points.size() == 2) {
      res->trajectory = req->trajectory;
      res->trajectory.points[1].time_from_start.sec = 1;
      res->is_success = true;
    } else {
      res->is_success = false;
    }
  }

  rcl_interfaces::msg::SetParametersResult SetParameterCallback(const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
      if (param.get_name() == "velocity_ratio") {
        velocity_update_stamp_and_values_.emplace_back(clock_->now(), param.get_value<double>());
      }
      if (param.get_name() == "acceleration_ratio") {
        acceleration_update_stamp_and_values_.emplace_back(clock_->now(), param.get_value<double>());
      }
    }
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
  }
};
}  // namespace

namespace tmc_safe_pose_changer {

class SafePoseChangerNodeTest : public ::testing::Test {
 protected:
  void SetUp() override;

  std::shared_ptr<SafePoseChangerNode> safe_pose_changer_node_;
  rclcpp::executors::SingleThreadedExecutor safe_pose_changer_executor_;

  rclcpp::Node::SharedPtr test_client_node_;
  rclcpp::executors::SingleThreadedExecutor test_client_executor_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr request_topic_publisher_;

  std::shared_ptr<PlannerServiceStub> planner_server_;
  std::shared_ptr<FilterServiceStub> filter_server_;

  void SpinSome();
  void PublishJointState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions);
  void WaitActionComplete();

  tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>::Ptr arm_trajectory_command_;
  tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>::Ptr head_trajectory_command_;
};

void SafePoseChangerNodeTest::SetUp() {
  test_client_node_ = rclcpp::Node::make_shared("test_client");
  current_state_publisher_ = test_client_node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SystemDefaultsQoS());
  request_topic_publisher_ = test_client_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/safe_pose_changer/joint_reference", rclcpp::SystemDefaultsQoS());

  planner_server_ = std::make_shared<PlannerServiceStub>(test_client_node_);
  filter_server_ = std::make_shared<FilterServiceStub>(test_client_node_);

  arm_trajectory_command_ = std::make_shared<tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>>(
      test_client_node_, "/arm_trajectory_controller/joint_trajectory");
  head_trajectory_command_ = std::make_shared<tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory>>(
      test_client_node_, "/head_trajectory_controller/joint_trajectory");

  // Launch a node for parameter server
  auto arm_param_node = rclcpp::Node::make_shared("arm_trajectory_controller");
  auto head_param_node = rclcpp::Node::make_shared("head_trajectory_controller");

  // Parameter registration on the parameter server
  arm_param_node->declare_parameter("joints", kArmJoints);
  head_param_node->declare_parameter("joints", kHeadJoints);

  auto arm_spin_thread = std::thread(std::bind(SpinSomeThread, arm_param_node));
  auto head_spin_thread = std::thread(std::bind(SpinSomeThread, head_param_node));

  // Since we will do Wait_for_server inside, it will be generated after the server preparation
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("filter_node_name", "test_client"),
      rclcpp::Parameter("velocity_ratio", 0.4),
      rclcpp::Parameter("acceleration_ratio", 0.6)};
  safe_pose_changer_node_ = std::make_shared<tmc_safe_pose_changer::SafePoseChangerNode>(options);

  // I haven't grasped the cause, but when I Interrupt the thread,
  // There is a phenomenon that the node remains, so
  // As a treatment, we have to wait until the thread is completed.
  arm_spin_thread.join();
  head_spin_thread.join();

  safe_pose_changer_executor_.add_node(safe_pose_changer_node_);
  test_client_executor_.add_node(test_client_node_);

  sensor_msgs::msg::JointState initial_state;
  initial_state.name = kArmJoints;
  initial_state.name.insert(initial_state.name.begin(), kHeadJoints.begin(), kHeadJoints.end());
  initial_state.position.resize(initial_state.name.size(), 0.0);
  current_state_publisher_->publish(initial_state);
  WaitActionComplete();
}

void SafePoseChangerNodeTest::SpinSome() {
  safe_pose_changer_executor_.spin_some();
  test_client_executor_.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void SafePoseChangerNodeTest::PublishJointState(const std::vector<std::string>& joint_names,
                                                const std::vector<double>& joint_positions) {
  sensor_msgs::msg::JointState request;
  request.name = joint_names;
  request.position = joint_positions;
  request_topic_publisher_->publish(request);

  WaitActionComplete();
}

void SafePoseChangerNodeTest::WaitActionComplete() {
  // Since SPIN is turned appropriately and initialized, I want to do it with SPIN until the initialization is completed.
  for (auto i = 0; i < 20; ++i) {
    SpinSome();
  }
}

TEST_F(SafePoseChangerNodeTest, AllArmJointsReferenceTopic) {
  PublishJointState(kArmJoints, {0.1, 0.2, 0.3, 0.4, 0.5});

  EXPECT_TRUE(arm_trajectory_command_->IsSubscribed());
  ValidateResponse(arm_trajectory_command_->GetValue(), kArmJoints, {0.1, 0.2, 0.3, 0.4, 0.5});

  EXPECT_TRUE(head_trajectory_command_->IsSubscribed());
  ValidateResponse(head_trajectory_command_->GetValue(), kHeadJoints, {0.0, 0.0});

  auto joint_names = kArmJoints;
  joint_names.insert(joint_names.begin(), kHeadJoints.begin(), kHeadJoints.end());
  const auto planning_initial_state = planner_server_->input_initial_state();
  EXPECT_EQ(planning_initial_state.name, joint_names);
  EXPECT_EQ(planning_initial_state.position, std::vector<double>(joint_names.size(), 0.0));

  joint_names = kArmJoints;
  joint_names.push_back("base_roll_joint");
  const auto filter_initial_state = filter_server_->input_initial_state();
  EXPECT_EQ(filter_initial_state.name, joint_names);
  EXPECT_EQ(filter_initial_state.position, std::vector<double>(joint_names.size(), 0.0));

  filter_server_->ValidateVelocityAndAccelerationUpdate(0.4, 0.6);

  const auto environment = planner_server_->input_environment();
  ASSERT_EQ(environment.collision_objects.size(), 1);
  EXPECT_EQ(environment.collision_objects[0].pose.orientation.w, 1.0);

  ASSERT_EQ(environment.collision_objects[0].primitives.size(), 1);
  EXPECT_EQ(environment.collision_objects[0].primitives[0].type, shape_msgs::msg::SolidPrimitive::BOX);

  ASSERT_EQ(environment.collision_objects[0].primitive_poses.size(), 1);
  EXPECT_EQ(environment.collision_objects[0].primitive_poses[0].orientation.w, 1.0);

  const double floor_height = environment.collision_objects[0].pose.position.z
                            + environment.collision_objects[0].primitive_poses[0].position.z
                            + environment.collision_objects[0].primitives[0].dimensions[2] / 2.0;
  EXPECT_NEAR(floor_height, -0.001, 1.0e-6);
}

TEST_F(SafePoseChangerNodeTest, ArmAndHeadJointsReferenceTopic) {
  auto joint_names = kArmJoints;
  joint_names.insert(joint_names.end(), kHeadJoints.begin(), kHeadJoints.end());
  PublishJointState(joint_names, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});

  EXPECT_TRUE(arm_trajectory_command_->IsSubscribed());
  ValidateResponse(arm_trajectory_command_->GetValue(), kArmJoints, {0.1, 0.2, 0.3, 0.4, 0.5});

  EXPECT_TRUE(head_trajectory_command_->IsSubscribed());
  ValidateResponse(head_trajectory_command_->GetValue(), kHeadJoints, {0.6, 0.7});
}

TEST_F(SafePoseChangerNodeTest, OneArmJointReferenceTopic) {
  PublishJointState({"arm_flex_joint"}, {0.1});

  EXPECT_TRUE(arm_trajectory_command_->IsSubscribed());
  ValidateResponse(arm_trajectory_command_->GetValue(), kArmJoints, {0.0, 0.1, 0.0, 0.0, 0.0});

  EXPECT_TRUE(head_trajectory_command_->IsSubscribed());
  ValidateResponse(head_trajectory_command_->GetValue(), kHeadJoints, {0.0, 0.0});
}

TEST_F(SafePoseChangerNodeTest, OneArmAndOneHeadJointsReferenceTopic) {
  PublishJointState({"arm_flex_joint", "head_pan_joint"}, {0.1, 0.2});

  EXPECT_TRUE(arm_trajectory_command_->IsSubscribed());
  ValidateResponse(arm_trajectory_command_->GetValue(), kArmJoints, {0.0, 0.1, 0.0, 0.0, 0.0});

  EXPECT_TRUE(head_trajectory_command_->IsSubscribed());
  ValidateResponse(head_trajectory_command_->GetValue(), kHeadJoints, {0.2, 0.0});
}

TEST_F(SafePoseChangerNodeTest, EmptyReferenceTopic) {
  PublishJointState({}, {});

  EXPECT_FALSE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

TEST_F(SafePoseChangerNodeTest, SizeMismatch) {
  PublishJointState(kArmJoints, {0.1});

  EXPECT_FALSE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

TEST_F(SafePoseChangerNodeTest, InvalidJointName) {
  PublishJointState({"invalid"}, {0.1});

  EXPECT_FALSE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

TEST_F(SafePoseChangerNodeTest, PlanningFailure) {
  planner_server_->set_is_success(false);
  PublishJointState(kArmJoints, {0.1, 0.2, 0.3, 0.4, 0.5});

  EXPECT_FALSE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

TEST_F(SafePoseChangerNodeTest, FilterFailure) {
  filter_server_->set_is_success(false);
  PublishJointState(kArmJoints, {0.1, 0.2, 0.3, 0.4, 0.5});

  EXPECT_FALSE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

TEST_F(SafePoseChangerNodeTest, SetController) {
  safe_pose_changer_executor_.remove_node(safe_pose_changer_node_);

  // Set for only Arm_trajectory_controller
  auto arm_param_node = rclcpp::Node::make_shared("arm_trajectory_controller");
  arm_param_node->declare_parameter("joints", kArmJoints);
  auto arm_spin_thread = std::thread(std::bind(SpinSomeThread, arm_param_node));

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("controllers", std::vector<std::string>({"/arm_trajectory_controller"})),
      rclcpp::Parameter("filter_node_name", "test_client")};

  auto safe_pose_changer_node = std::make_shared<tmc_safe_pose_changer::SafePoseChangerNode>(options);
  arm_spin_thread.join();

  safe_pose_changer_executor_.add_node(safe_pose_changer_node);

  // Only ARM reacts
  sensor_msgs::msg::JointState current_state;
  current_state.name = kArmJoints;
  current_state.position.resize(kArmJoints.size(), 0.0);
  current_state_publisher_->publish(current_state);
  WaitActionComplete();

  PublishJointState(kArmJoints, {0.1, 0.0, 0.0, 0.0, 0.0});

  EXPECT_TRUE(arm_trajectory_command_->IsSubscribed());
  EXPECT_FALSE(head_trajectory_command_->IsSubscribed());
}

}  // namespace tmc_safe_pose_changer

// If the target speed is the same as the current angle, consider what to do

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
