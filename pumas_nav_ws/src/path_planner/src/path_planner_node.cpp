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
    nav_msgs::msg::OccupancyGrid static_map_;
    nav_msgs::msg::OccupancyGrid static_cost_map_;
    nav_msgs::msg::OccupancyGrid augmented_map_;
    nav_msgs::msg::OccupancyGrid augmented_cost_map_;

    // To hold the incoming request/response
    std::shared_ptr<nav_msgs::srv::GetPlan::Request> pending_request_;
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> pending_response_;

    // Timer and readiness flag
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;
    bool is_static_map_ = false;
    bool is_static_cost_map_ = false;
    bool is_augmented_map_ = false;
    bool is_augmented_cost_map_ = false;

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
    void process_a_star()
    {
        if (is_static_map_ && is_static_cost_map_ && pending_request_ && pending_response_)
        {
            const auto &req = pending_request_;
            auto &res = pending_response_;

            bool success = PathPlanner::AStar(static_map_, static_cost_map_,
                                            req->start.pose, req->goal.pose,
                                            diagonal_paths_, res->plan);
            if (success)
            {
                res->plan = PathPlanner::SmoothPath(res->plan, smooth_alpha_, smooth_beta_);
                RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Static path planned successfully.");
            }
            else
            {
                res->plan.poses.clear();
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan static path.");
            }

            // Clear state
            pending_request_.reset();
            pending_response_.reset();
            is_static_map_ = false;
            is_static_cost_map_ = false;
        }
    }

    void callback_a_star_with_static_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle static map request.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Received planning request (static map).");
        pending_request_ = request;
        pending_response_ = response;

        auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(static_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static) {
                try {
                    this->static_map_ = future_static.get()->map;
                    is_static_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static map with size %d x %d", 
                                static_map_.info.width, static_map_.info.height);
                    process_a_star();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static map: %s", e.what());
                }
            });

        auto static_cost_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_cost_map_->async_send_request(static_cost_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static_cost) {
                try {
                    this->static_cost_map_ = future_static_cost.get()->map;
                    is_static_cost_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static_cost map with size %d x %d", 
                                static_cost_map_.info.width, static_cost_map_.info.height);
                    process_a_star();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static_cost map: %s", e.what());
                }
            });
    }

    void callback_a_star_with_augmented_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        if (!services_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Services not ready. Cannot handle augmented map request.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Received planning request (augmented map).");

        pending_request_ = request;
        pending_response_ = response;

        auto augmented_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_augmented_map_->async_send_request(augmented_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_augmented) {
                try {
                    this->augmented_map_ = future_augmented.get()->map;
                    is_augmented_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got augmented map with size %d x %d", 
                                augmented_map_.info.width, augmented_map_.info.height);
                    process_a_star();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get augmented map: %s", e.what());
                }
            });

        auto augmented_cost_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_augmented_cost_map_->async_send_request(augmented_cost_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_augmented_cost) {
                try {
                    this->augmented_cost_map_ = future_augmented_cost.get()->map;
                    is_augmented_cost_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got augmented_cost map with size %d x %d", 
                                augmented_cost_map_.info.width, augmented_cost_map_.info.height);
                    process_a_star();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get augmented_cost map: %s", e.what());
                }
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
