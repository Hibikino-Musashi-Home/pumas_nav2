#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/srv/get_plan.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"


// Standard
#include <iomanip>   // for std::hex and std::setw
#include <sstream>   // for std::stringstream
#include <iostream>
#include <vector>
#include <exception>
///

#define RATE 10

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_CHECK_IF_INSIDE_OBSTACLES 19
#define SM_WAITING_FOR_MOVE_BACKWARDS 18
#define SM_CHECK_IF_OBSTACLES 21
#define SM_WAIT_FOR_NO_OBSTACLES 22
#define SM_ENABLE_POT_FIELDS 23
#define SM_WAIT_FOR_POT_FIELDS 24
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_STOP_RECEIVED 51
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_FINAL    17

class MotionPlannerNode : public rclcpp::Node
{
public:
    MotionPlannerNode() : Node("simple_move_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<bool>("patience",              true);
        this->declare_parameter<float>("proximity_criterion",  2.0f);
        this->declare_parameter<std::string>("base_link_name", "base_footprint");

        // Initialize internal variables from declared parameters
        this->get_parameter("patience",             patience_);
        this->get_parameter("proximity_criterion",  proximity_criterion_);
        this->get_parameter("base_link_name",       base_link_name_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MotionPlannerNode::on_parameter_change, this, std::placeholders::_1));

        // Initialize service clients (non-blocking)
        init_service_clients();

        //############
        // Publishers
        pub_pot_fields_enable_ = this->create_publisher<std_msgs::msg::Bool>("/navigation/potential_fields/enable", 1);
        pub_goal_path_ = this->create_publisher<nav_msgs::msg::Path>("/simple_move/goal_path", 1);
        pub_goal_dist_angle_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/simple_move/goal_dist_angle", 1);
        pub_status_ = this->create_publisher<actionlib_msgs::msg::GoalStatus>("/navigation/status", 10);
        pub_simple_move_stop_ = this->create_publisher<std_msgs::msg::Empty>("/simple_move/stop", 1);


        //############
        // Subscribers
        //// stop
        sub_general_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 10, std::bind(&MotionPlannerNode::callback_general_stop, this, std::placeholders::_1));

        sub_nav_ctrl_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/navigation/stop", 10, std::bind(&MotionPlannerNode::callback_navigation_stop, this, std::placeholders::_1));

        //// goal
        sub_simple_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/nav_control/goal", 10, std::bind(&MotionPlannerNode::callback_simple_goal, this, std::placeholders::_1));
        
        sub_move_goal_status_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
            "/simple_move/goal_reached", 10, std::bind(&MotionPlannerNode::callback_simple_move_goal_status, this, std::placeholders::_1));

        //// motion
        sub_set_patience_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/set_patience", 10, std::bind(&MotionPlannerNode::callback_set_patience, this, std::placeholders::_1));

        sub_collision_risk_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/potential_fields/collision_risk", 10, std::bind(&MotionPlannerNode::callback_collision_risk, this, std::placeholders::_1));

      
        //############
        // Wait for transforms
        wait_for_transforms("map", base_link_name_);

        // Simple Move main processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&MotionPlannerNode::motion_planner_processing, this));

        RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> MotionPlannerNode is ready.");
    }

private:
    //############
    // State variables
    bool stop_                  = false;
    bool collision_risk_        = false;
    bool new_global_goal_       = false;
    int simple_move_status_id_  = 0;

    geometry_msgs::msg::Pose            global_goal_;
    actionlib_msgs::msg::GoalStatus     simple_move_goal_status_;

    float robot_x_ = 0.0f;
    float robot_y_ = 0.0f;
    float robot_t_ = 0.0f;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Internal parameter values
    bool  patience_;
    float proximity_criterion_;
    std::string base_link_name_;

    // Motion Planner processing variables
    float error   = 0.0f;

    int state = SM_INIT;
    int simple_move_sequencer = -1;
    int goal_id = -1;
    int current_status = 0;

    bool near_goal_sent = false;

    // ROS 2 message types
    std_msgs::msg::Bool msg_bool;
    nav_msgs::msg::Path path;
    std_msgs::msg::Float32MultiArray msg_goal_dist_angle;

    // ROS 2 service object (split request/response)
    std_srvs::srv::Trigger::Request srv_check_obstacles_request;
    std_srvs::srv::Trigger::Response srv_check_obstacles_response;


    //############
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               pub_pot_fields_enable_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_goal_path_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_goal_dist_angle_;
    rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr   pub_status_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr              pub_simple_move_stop_;

    //############
    // Subscribers
        //// stop
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_general_stop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_nav_ctrl_stop_;

    //// goal
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_simple_goal_;
    rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr sub_move_goal_status_;

    //// motion
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_set_patience_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collision_risk_;

    //############
    // Service clients
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr clt_plan_path_static_;
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr clt_plan_path_augmented_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_get_aug_map_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_get_aug_costmap_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_are_there_obs_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_is_in_obstacles_;


    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Timer and readiness flag
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;

    // Used to wait for first message
    std::shared_ptr<std::promise<std_msgs::msg::Bool::SharedPtr>> collision_risk_promise_;
    bool waiting_for_potential_fields_ = false;

    // Main processing loop
    rclcpp::TimerBase::SharedPtr processing_timer_;


    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name()      == "patience")            patience_               = param.as_bool();
            else if (param.get_name() == "proximity_criterion") proximity_criterion_    = param.as_double();
            else if (param.get_name() == "base_link_name")      base_link_name_         = param.as_string();

            else {
                result.successful = false;
                result.reason = "MotionPlanner.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    // Wait for transforms 
    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "MotionPlanner.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

        rclcpp::Time start_time = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0); 

        bool transform_ok = false;

        while (rclcpp::ok() && (this->now() - start_time) < timeout) {
            try {
                tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1)
                );
                transform_ok = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "MotionPlanner.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "MotionPlanner.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "MotionPlanner.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }

   //############
    // Initialize service clients (non-blocking)
    void init_service_clients()
    {
        clt_plan_path_static_       = this->create_client<nav_msgs::srv::GetPlan>("/path_planner/plan_path_with_static");
        clt_plan_path_augmented_    = this->create_client<nav_msgs::srv::GetPlan>("/path_planner/plan_path_with_augmented");
        clt_get_aug_map_            = this->create_client<std_srvs::srv::Trigger>("/map_augmenter/get_augmented_map");
        clt_get_aug_costmap_        = this->create_client<std_srvs::srv::Trigger>("/map_augmenter/get_augmented_cost_map");
        clt_are_there_obs_          = this->create_client<std_srvs::srv::Trigger>("/map_augmenter/are_there_obstacles");
        clt_is_in_obstacles_        = this->create_client<std_srvs::srv::Trigger>("/map_augmenter/is_inside_obstacles");

        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (clt_plan_path_static_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_plan_path_augmented_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_aug_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_aug_costmap_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_are_there_obs_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_is_in_obstacles_->wait_for_service(std::chrono::seconds(0)))
                {
                    RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> All motion planner clients are now available.");
                    services_ready_ = true;
                    service_check_timer_->cancel();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> Waiting for motion planner clients to become available...");
                }
            });
    }


    //############
    //Motion Planner callbacks
    void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg)
    {
        try 
        {
            std::cout << "MotionPlanner.-> General Stop signal received" << std::endl;
            stop_     = true;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_general_stop: %s", e.what());
        }
    }

    void callback_navigation_stop(const std_msgs::msg::Empty::SharedPtr msg)
    {
        try 
        {
            std::cout << "MotionPlanner.-> Navigation Stop signal received" << std::endl;
            stop_     = true;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_navigation_stop: %s", e.what());
        }
    }

    void callback_simple_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try 
        {
            std::cout << "MotionPlanner.-> New goal received." << std::endl;
            global_goal_ = msg->pose;
            new_global_goal_ = true;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_simple_goal: %s", e.what());
        }
    }

    void callback_simple_move_goal_status(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
    {
        try 
        {
            simple_move_goal_status_ = *msg;

            std::stringstream ss;
            ss << msg->goal_id.id;
            ss >> simple_move_status_id_;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_simple_move_goal_status: %s", e.what());
        }
    }

    void callback_set_patience(const std_msgs::msg::Bool::SharedPtr msg)
    {
        try 
        {
            std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False") << std::endl;
            patience_ = msg->data;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_set_patience: %s", e.what());
        }
    }

    void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg)
    {
        try 
        {
            collision_risk_ = msg->data;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error processing callback_collision_risk: %s", e.what());
        }
    }

    //############
    //Motion Planner additional functions
    bool plan_path_from_augmented_map(
        float robot_x, float robot_y,
        float goal_x, float goal_y,
        nav_msgs::msg::Path& path)
    {
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->start.pose.position.x = robot_x;
        request->start.pose.position.y = robot_y;
        request->start.header.frame_id = "map";

        request->goal.pose.position.x = goal_x;
        request->goal.pose.position.y = goal_y;
        request->goal.header.frame_id = "map";

        // Wait for service to be available
        if (!clt_plan_path_augmented_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Service /path_planner/plan_path_with_augmented not available.");
            return false;
        }

        // Call service
        auto result_future = clt_plan_path_augmented_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Service call to /path_planner/plan_path_with_augmented failed.");
            return false;
        }

        path = result_future.get()->plan;
        return true;
    }

    void get_robot_position()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t_ = static_cast<float>(yaw);

            return;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s", ex.what());
            robot_x_ = robot_y_ = robot_t_ = 0.0f;
            return;
        }
    }

    int publish_status(
        int status, int id, const std::string& text)
    {
        actionlib_msgs::msg::GoalStatus msg;
        std::stringstream ss;
        ss << id;
        ss >> msg.goal_id.id;
        msg.status = status;
        msg.text = text;
        msg.goal_id.stamp = rclcpp::Clock().now();

        pub_status_->publish(msg);

        return status;
    }


    //############
    //Simple Move main processing
    void motion_planner_processing() 
    {
        if (!services_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Services not ready.");
            return;
        }

        try
        {
            if(stop_)
            {
                stop_ = false;
                state = SM_INIT;
                if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE)
                {
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED,
                        goal_id,
                        "Stop signal received. Task cancelled"
                    );
                }
            }
            if(new_global_goal_)
                state = SM_WAITING_FOR_TASK;

            switch(state)
            {
            case SM_INIT:
            {
                std::cout << "MotionPlanner.-> MVN PLN READY. Waiting for new goal. " << std::endl;
                state = SM_WAITING_FOR_TASK;
                break;
            }
            
            
            case SM_WAITING_FOR_TASK:
            {
                if(new_global_goal_)
                {
                    new_global_goal_ = false;
                    state = SM_CALCULATE_PATH;
                    if(current_status == actionlib_msgs::msg::GoalStatus::ACTIVE)
                        current_status = publish_status(
                            actionlib_msgs::msg::GoalStatus::ABORTED,
                            goal_id, 
                            "Cancelling current movement."
                        );
                    goal_id++;
                    std::cout << "MotionPlanner.-> New goal received. Current task goal_id: " << goal_id << std::endl;
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ACTIVE,
                        goal_id,
                        "Starting new movement task"
                    );
                    near_goal_sent = false;
                }
                break;
            }

            
            case SM_CALCULATE_PATH:
            {
                get_robot_position();
                if(!plan_path_from_augmented_map(robot_x_, robot_y_, global_goal_.position.x, global_goal_.position.y, path))
                {
                    std::cout<<"MotionPlanner.-> Cannot calc path to "<< global_goal_.position.x <<" "<<global_goal_.position.y << std::endl;
                    pub_simple_move_stop_->publish(std_msgs::msg::Empty());
                    if(!patience_)
                        state = SM_CHECK_IF_INSIDE_OBSTACLES;
                    else
                        state = SM_CHECK_IF_OBSTACLES;
                }
                else
                    state = SM_ENABLE_POT_FIELDS;
                break;
            }
                

            case SM_CHECK_IF_INSIDE_OBSTACLES:
            {
                std::cout << "MotionPlanner.-> Checking if robot is inside an obstacle..." << std::endl;

                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result_future = clt_is_in_obstacles_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
                    != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Failed to get obstacles.");
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED, 
                        this->goal_id, 
                        "Cannot calc path from start to goal"
                    );
                    state = SM_INIT;

                    break;
                }
                auto result = result_future.get();

                if(result->success)
                {
                    std::cout << "MotionPlanner.-> Robot is inside an obstacle. Moving backwards..." << std::endl;
                    msg_goal_dist_angle.data[0] = -0.25;
                    msg_goal_dist_angle.data[1] = 0;
                    pub_goal_dist_angle_->publish(msg_goal_dist_angle);
                    state = SM_WAITING_FOR_MOVE_BACKWARDS;
                }
                else
                {
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED, 
                        goal_id, 
                        "Cannot calc path from start to goal"
                    );
                    state = SM_INIT;
                }
                break;
            }


            case SM_WAITING_FOR_MOVE_BACKWARDS:
            {
                if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED && simple_move_status_id_ == -1)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Moved backwards succesfully." << std::endl;
                }
                else if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Simple move reported move aborted. " << std::endl;
                }
                state = SM_CALCULATE_PATH;
                break;
            }


            case SM_CHECK_IF_OBSTACLES:
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result_future = clt_are_there_obs_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
                    != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Failed to get obstacles.");
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED, 
                        this->goal_id, 
                        "Cannot calculate path from start to goal point"
                    );
                    state = SM_INIT;

                    break;
                }
                auto result = result_future.get();
            
                if(!result->success)
                {
                    std::cout << "MotionPlanner.->There are no temporal obstacles. Announcing failure." << std::endl;
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED, 
                        goal_id, 
                        "Cannot calculate path from start to goal point"
                    );
                    state = SM_INIT;
                }
                else
                {
                    std::cout << "MotionPlanner.->Temporal obstacles detected. Waiting for them to move." << std::endl;
                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ACTIVE, 
                        goal_id,
                        "Waiting for temporal obstacles to move"
                    );
                    state = SM_WAIT_FOR_NO_OBSTACLES;
                }
                break;
            }


            case SM_WAIT_FOR_NO_OBSTACLES:
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result_future = clt_are_there_obs_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
                    != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Cannot call service for checking temporal obstacles. Announcing failure.");

                    current_status = publish_status(
                        actionlib_msgs::msg::GoalStatus::ABORTED, 
                        this->goal_id,
                        "Cannot calculate path from start to goal point"
                    );
                    state = SM_INIT;

                    break;
                }
                auto result = result_future.get();

                if(!result->success)
                {
                    std::cout << "MotionPlanner.-> Temporal obstacles removed. " << std::endl;
                    state = SM_CALCULATE_PATH;
                }
                else
                    rclcpp::sleep_for(std::chrono::seconds(1));
                break;
            }
 
                
            case SM_ENABLE_POT_FIELDS:
            {
                msg_bool.data = true;
                pub_pot_fields_enable_->publish(msg_bool);
                std::cout << "MotionPlanner.-> Potential fields enable flag sent. Waiting for potential fields to be enabled..." << std::endl;
                state = SM_WAIT_FOR_POT_FIELDS;
                break;
            }


            case SM_WAIT_FOR_POT_FIELDS:
            {
                if (!waiting_for_potential_fields_)
                {
                    RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> Waiting for potential fields message...");

                    // Create a new promise to wait for message
                    collision_risk_promise_ = std::make_shared<std::promise<std_msgs::msg::Bool::SharedPtr>>();
                    waiting_for_potential_fields_ = true;

                    // Detach a thread to wait asynchronously
                    std::thread([this]() {
                        auto future = collision_risk_promise_->get_future();
                        if (future.wait_for(std::chrono::seconds(100)) == std::future_status::ready)
                        {
                            RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> Potential fields is now available.");
                            this->state = SM_START_MOVE_PATH;
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> Timeout waiting for potential fields message.");
                            this->state = SM_INIT;  // Fallback
                        }

                        waiting_for_potential_fields_ = false;
                    }).detach();
                }
                break;
            }


            case SM_START_MOVE_PATH:
            {
                std::cout << "MotionPlanner.-> Starting path following " << std::endl;
                collision_risk_ = false;
                simple_move_sequencer++;

                path.header.frame_id = std::to_string(simple_move_sequencer);
                pub_goal_path_->publish(path);
                simple_move_goal_status_.status = 0;

                state = SM_WAIT_FOR_MOVE_FINISHED;
                break;
            }


            case SM_WAIT_FOR_MOVE_FINISHED:
            {
                get_robot_position();
                error = sqrt(pow(global_goal_.position.x - robot_x_, 2) + pow(global_goal_.position.y - robot_y_, 2));
                if(error < proximity_criterion_ && !near_goal_sent)
                {
                    near_goal_sent = true;
                    std::cout << "MotionPlanner.->Error less than proximity criterion. Sending near goal point status." << std::endl;
                    publish_status(actionlib_msgs::msg::GoalStatus::ACTIVE, 
                        goal_id, 
                        "Near goal point"
                    );
                }
                if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED && 
                                                      simple_move_status_id_ == simple_move_sequencer)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Path followed succesfully. " << std::endl;
                    msg_bool.data = false;
                    pub_pot_fields_enable_->publish(msg_bool);
                    state = SM_CORRECT_FINAL_ANGLE;
                }
                else if(collision_risk_)
                {
                    std::cout << "MotionPlanner.-> COLLISION RISK DETECTED before goal is reached." << std::endl;
                    collision_risk_ = false;
                    state = SM_CALCULATE_PATH;
                }
                else if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Simple move reported path aborted. Trying again..." << std::endl;
                    state = SM_CALCULATE_PATH;
                }
                break;
            }


            case SM_CORRECT_FINAL_ANGLE:
            {
                std::cout << "MotionPlanner.-> Correcting final angle." << std::endl;
                get_robot_position();
                error = atan2(global_goal_.orientation.z, global_goal_.orientation.w)*2 - robot_t_;

                if(error  >  M_PI) error -= 2*M_PI;
                if(error <= -M_PI) error += 2*M_PI;

                msg_goal_dist_angle.data[0] = 0;
                msg_goal_dist_angle.data[1] = error;
                pub_goal_dist_angle_->publish(msg_goal_dist_angle);

                state = SM_WAIT_FOR_ANGLE_CORRECTED;
                break;
            }


            case SM_WAIT_FOR_ANGLE_CORRECTED:
            {
                if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED && 
                                                      simple_move_status_id_ == -1)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Final angle corrected succesfully." << std::endl;
                    state = SM_FINAL ;
                }
                else if(simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED)
                {
                    simple_move_goal_status_.status = 0;
                    std::cout << "MotionPlanner.-> Simple move reported move aborted. Trying again..." << std::endl;
                    state = SM_CALCULATE_PATH;
                }
                break;
            }


            case SM_FINAL:
            {
                std::cout << "MotionPlanner.-> TASK FINISHED." << std::endl;
                current_status = publish_status(
                    actionlib_msgs::msg::GoalStatus::SUCCEEDED, 
                    goal_id, 
                    "Global goal point reached"
                );
                state = SM_INIT;
                break;
            }



            default:
                std::cout<<"MotionPlanner.-> A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE :'(" << std::endl;
                return;
            }

        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Error in MotionPlanner processing: %s", e.what());
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlannerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
