#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "PathPlanner.h"  

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        // Declare and get parameters
        this->declare_parameter<float>("smooth_alpha",  0.1f);
        this->declare_parameter<float>("smooth_beta",   0.9f);
        this->declare_parameter<bool>("diagonal_paths", false);

        this->get_parameter("smooth_alpha",     smooth_alpha_);
        this->get_parameter("smooth_beta",      smooth_beta_);
        this->get_parameter("diagonal_paths",   diagonal_paths_);

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Smooth Alpha: %.2f, Beta: %.2f, Diagonal: %s",
                    smooth_alpha_, smooth_beta_, diagonal_paths_ ? "true" : "false");

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PathPlannerNode::on_parameter_change, this, std::placeholders::_1));
        
        // Initialize service clients (non-blocking)
        init_service_clients();

        // Advertise planning services
        srv_plan_static_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/path_planner/plan_path_with_static",
            std::bind(&PathPlannerNode::callback_a_star_with_static_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_plan_augmented_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/path_planner/plan_path_with_augmented",
            std::bind(&PathPlannerNode::callback_a_star_with_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> PathPlannerNode is ready.");
    }

private:
    // Parameters
    float smooth_alpha_;
    float smooth_beta_;
    bool diagonal_paths_;

    // Service clients
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_cost_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_augmented_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_augmented_cost_map_;

    // Service servers
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_static_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_augmented_;

    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // map services
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::OccupancyGrid cost_map_;

    // Timer and readiness flag
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;

    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "PathPlanner.-> Parameters updated successfully.";

        for (const auto &param : params)
        {
            if (param.get_name()      == "smooth_alpha")    smooth_alpha_   = param.as_double();
            else if (param.get_name() == "smooth_beta")     smooth_beta_    = param.as_double();
            else if (param.get_name() == "diagonal_paths")  diagonal_paths_ = param.as_bool();

            else
            {
                result.successful = false;
                result.reason = "PathPlanner.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    //############
    // Initialize service clients (non-blocking)
    void init_service_clients()
    {
        clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>("/map_augmenter/get_static_map");
        clt_get_static_cost_map_ = this->create_client<nav_msgs::srv::GetMap>("/map_augmenter/get_static_cost_map");
        clt_get_augmented_map_ = this->create_client<nav_msgs::srv::GetMap>("/map_augmenter/get_augmented_map");
        clt_get_augmented_cost_map_ = this->create_client<nav_msgs::srv::GetMap>("/map_augmenter/get_augmented_cost_map");

        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (clt_get_static_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_static_cost_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_augmented_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_augmented_cost_map_->wait_for_service(std::chrono::seconds(0)))
                {
                    RCLCPP_INFO(this->get_logger(), "PathPlanner.-> All map services are now available.");
                    services_ready_ = true;
                    service_check_timer_->cancel();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Waiting for map services to become available...");
                }
            });
    }

    //############
    // Callback functions
    void callback_a_star_with_static_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_) {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle augmented map request.");
            response->plan.poses.clear(); 
            return;
        }

        auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(static_map_req,
            [this, request, response](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static)
            {
                try {
                    map_ = future_static.get()->map;
                    RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Got static map with size %d x %d", 
                                map_.info.width, map_.info.height);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get static map: %s", e.what());
                    response->plan.poses.clear();
                    return;
                }

                auto static_cost_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
                clt_get_static_cost_map_->async_send_request(static_cost_map_req,
                    [this, request, response](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static_cost)
                    {
                        try {
                            cost_map_ = future_static_cost.get()->map;
                            RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Got static_cost map with size %d x %d", 
                                        cost_map_.info.width, cost_map_.info.height);
                        } catch (const std::exception &e) {
                            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get augmented_cost map: %s", e.what());
                            response->plan.poses.clear();
                            return;
                        }

                        nav_msgs::msg::Path path;
                        bool success = PathPlanner::AStar(map_, cost_map_,
                            request->start.pose, request->goal.pose,
                            diagonal_paths_, path);

                        if (success) {
                            response->plan = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
                            RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path with static_map planned successfully.");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                            response->plan.poses.clear();
                        }
                    });
            });
    }

    void callback_a_star_with_augmented_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_) {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle augmented map request.");
            response->plan.poses.clear(); 
            return;
        }

        auto augmented_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_augmented_map_->async_send_request(augmented_map_req,
            [this, request, response](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_augmented)
            {
                try {
                    map_ = future_augmented.get()->map;
                    RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Got augmented map with size %d x %d", 
                                map_.info.width, map_.info.height);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get augmented map: %s", e.what());
                    response->plan.poses.clear();
                    return;
                }

                auto augmented_cost_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
                clt_get_augmented_cost_map_->async_send_request(augmented_cost_map_req,
                    [this, request, response](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_augmented_cost)
                    {
                        try {
                            cost_map_ = future_augmented_cost.get()->map;
                            RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Got augmented_cost map with size %d x %d", 
                                        cost_map_.info.width, cost_map_.info.height);
                        } catch (const std::exception &e) {
                            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get augmented_cost map: %s", e.what());
                            response->plan.poses.clear();
                            return;
                        }

                        nav_msgs::msg::Path path;
                        bool success = PathPlanner::AStar(map_, cost_map_,
                            request->start.pose, request->goal.pose,
                            diagonal_paths_, path);

                        if (success) {
                            response->plan = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
                            RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path with augmented_map planned successfully."); 
                        } else {
                            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                            response->plan.poses.clear();
                        }
                    });
            });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
