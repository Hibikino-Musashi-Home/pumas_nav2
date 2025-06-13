#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/srv/get_map.hpp"
#include "std_srvs/srv/trigger.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/utils.h"
#include "Eigen/Geometry"

#include "message_filters/subscriber.h"

#include <cmath>

class MapAugmenterNode : public rclcpp::Node
{
public:
    MapAugmenterNode() : Node("map_augmenter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters with default values
        this->declare_parameter("use_lidar",              false);
        this->declare_parameter("use_point_cloud",        false);
        this->declare_parameter("use_point_cloud2",       false);
        this->declare_parameter("use_online",             false);
        this->declare_parameter("min_x",                  -10.0);
        this->declare_parameter("max_x",                  10.0);
        this->declare_parameter("min_y",                  -10.0);
        this->declare_parameter("max_y",                  10.0);
        this->declare_parameter("min_z",                  -1.0);
        this->declare_parameter("max_z",                  2.0);
        this->declare_parameter("decay_factor",           10);
        this->declare_parameter("inflation_radius",       0.25);
        this->declare_parameter("cost_radius",            0.25);
        this->declare_parameter("cloud_downsampling",     1);
        this->declare_parameter("cloud_downsampling2",    1);
        this->declare_parameter("lidar_downsampling",     1);
        this->declare_parameter("point_cloud_topic",      "/point_cloud");
        this->declare_parameter("point_cloud_topic2",     "/point_cloud2");
        this->declare_parameter("laser_scan_topic",       "/scan");
        this->declare_parameter("static_map_server",      "/static_map_server/map");
        this->declare_parameter("prohibition_map_server", "/prohibition_map_server/map");
        this->declare_parameter("base_link_name",         "base_footprint");

        // Initialize internal variables from declared parameters
        this->get_parameter("use_lidar",              use_lidar_);
        this->get_parameter("use_point_cloud",        use_cloud_);
        this->get_parameter("use_point_cloud2",       use_cloud2_);
        this->get_parameter("use_online",             use_online_);
        this->get_parameter("min_x",                  min_x_);
        this->get_parameter("max_x",                  max_x_);
        this->get_parameter("min_y",                  min_y_);
        this->get_parameter("max_y",                  max_y_);
        this->get_parameter("min_z",                  min_z_);
        this->get_parameter("max_z",                  max_z_);
        this->get_parameter("decay_factor",           decay_factor_);
        this->get_parameter("inflation_radius",       inflation_radius_);
        this->get_parameter("cost_radius",            cost_radius_);
        this->get_parameter("cloud_downsampling",     cloud_downsampling_);
        this->get_parameter("cloud_downsampling2",    cloud_downsampling2_);
        this->get_parameter("lidar_downsampling",     lidar_downsampling_);
        this->get_parameter("point_cloud_topic",      point_cloud_topic_);
        this->get_parameter("point_cloud_topic2",     point_cloud_topic2_);
        this->get_parameter("laser_scan_topic",       laser_scan_topic_);
        this->get_parameter("static_map_server",      static_map_server_);
        this->get_parameter("prohibition_map_server", prohibition_map_server_);
        this->get_parameter("base_link_name",         base_link_name_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MapAugmenterNode::on_parameter_change, this, std::placeholders::_1));

        // Initialize service clients (non-blocking)
        init_service_clients();

        //############
        // Publishers
        pub_augmented_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/augmented_map", rclcpp::QoS(10).transient_local());

        //############
        // Subscribers
        sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", rclcpp::SensorDataQoS(), 
            std::bind(&MapAugmenterNode::callback_point_obstacle, this, std::placeholders::_1));

        sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(point_cloud_mutex_);
                latest_point_cloud_ = msg;
                point_cloud_received_ = true;
            });

        sub_point_cloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic2_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(point_cloud2_mutex_);
                latest_point_cloud2_ = msg;
                point_cloud2_received_ = true;
            });

        sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(laser_scan_mutex_);
                latest_laser_scan_ = msg;
                laser_scan_received_ = true;
            });

        //############
        // Advertise augmenter services
        srv_static_map_ = this->create_service<nav_msgs::srv::GetMap>(
            "/map_augmenter/get_static_map", 
            std::bind(&MapAugmenterNode::callback_static_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_static_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
            "/map_augmenter/get_static_cost_map", 
            std::bind(&MapAugmenterNode::callback_static_cost_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_augmented_map_ = this->create_service<nav_msgs::srv::GetMap>(
            "/map_augmenter/get_augmented_map", 
            std::bind(&MapAugmenterNode::callback_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_augmented_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
            "/map_augmenter/get_augmented_cost_map", 
            std::bind(&MapAugmenterNode::callback_augmented_cost_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_are_there_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
            "/map_augmenter/are_there_obstacles", 
            std::bind(&MapAugmenterNode::callback_are_there_obstacles, this, std::placeholders::_1, std::placeholders::_2));

        srv_is_inside_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
            "/map_augmenter/is_inside_obstacles", 
            std::bind(&MapAugmenterNode::callback_is_inside_obstacles, this, std::placeholders::_1, std::placeholders::_2));

        //############
        // Map Augmenter main processing
        get_first_maps();

        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100 ms = 10 Hz
            std::bind(&MapAugmenterNode::map_augmenter_processing, this));

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> MapAugmenterNode is ready.");
    }

private:
    //############
    // State variables
    int counter_ = 0;
    bool are_there_obstacles_;

    nav_msgs::msg::OccupancyGrid static_map_;
    nav_msgs::msg::OccupancyGrid prohibition_map_;
    nav_msgs::msg::OccupancyGrid static_cost_map_;
    nav_msgs::msg::OccupancyGrid obstacles_map_;
    nav_msgs::msg::OccupancyGrid obstacles_inflated_map_;
    nav_msgs::msg::OccupancyGrid augmented_map_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Internal parameter values
    bool use_lidar_;
    bool use_cloud_;
    bool use_cloud2_;
    bool use_online_;

    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;

    int decay_factor_;
    int cloud_downsampling_;
    int cloud_downsampling2_;
    int lidar_downsampling_;

    double inflation_radius_;
    double cost_radius_;

    std::string point_cloud_topic_;
    std::string point_cloud_topic2_;
    std::string laser_scan_topic_;
    std::string static_map_server_;
    std::string prohibition_map_server_;

    std::string base_link_name_;

    //############
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_augmented_map_;

    //############
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;

    // PointCloud
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;
    std::mutex point_cloud_mutex_;
    std::atomic<bool> point_cloud_received_{false};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud2_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud2_;
    std::mutex point_cloud2_mutex_;
    std::atomic<bool> point_cloud2_received_{false};

    // LaserScan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_laser_scan_;
    std::mutex laser_scan_mutex_;
    std::atomic<bool> laser_scan_received_{false};

    //############
    // Service clients
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_prohibition_map_;

    // Service servers
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_map_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_cost_map_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_map_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_cost_map_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_are_there_obstacles_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_is_inside_obstacles_;

    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Timer and readiness flag
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;
    bool is_static_map_ = false;
    bool is_prohibition_map_ = false;

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
            if (param.get_name()      == "use_lidar")              use_lidar_              = param.as_bool();
            else if (param.get_name() == "use_point_cloud")        use_cloud_              = param.as_bool();
            else if (param.get_name() == "use_point_cloud2")       use_cloud2_             = param.as_bool();
            else if (param.get_name() == "use_online")             use_online_             = param.as_bool();

            else if (param.get_name() == "min_x")                  min_x_                  = param.as_double();
            else if (param.get_name() == "max_x")                  max_x_                  = param.as_double();
            else if (param.get_name() == "min_y")                  min_y_                  = param.as_double();
            else if (param.get_name() == "max_y")                  max_y_                  = param.as_double();
            else if (param.get_name() == "min_z")                  min_z_                  = param.as_double();
            else if (param.get_name() == "max_z")                  max_z_                  = param.as_double();

            else if (param.get_name() == "decay_factor")           decay_factor_           = param.as_int();
            else if (param.get_name() == "cloud_downsampling")     cloud_downsampling_     = param.as_int();
            else if (param.get_name() == "cloud_downsampling2")    cloud_downsampling2_    = param.as_int();
            else if (param.get_name() == "lidar_downsampling")     lidar_downsampling_     = param.as_int();
            else if (param.get_name() == "inflation_radius")       inflation_radius_       = param.as_double();
            else if (param.get_name() == "cost_radius")            cost_radius_            = param.as_double();

            else if (param.get_name() == "point_cloud_topic")      point_cloud_topic_      = param.as_string();
            else if (param.get_name() == "point_cloud_topic2")     point_cloud_topic2_     = param.as_string();
            else if (param.get_name() == "laser_scan_topic")       laser_scan_topic_       = param.as_string();
            else if (param.get_name() == "static_map_server")      static_map_server_      = param.as_string();
            else if (param.get_name() == "prohibition_map_server") prohibition_map_server_ = param.as_string();

            else if (param.get_name() == "base_link_name")         base_link_name_         = param.as_string();

            else {
                result.successful = false;
                result.reason = "MapAugmenter.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    //############
    // Initialize service clients (non-blocking)
    void init_service_clients()
    {
        clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>(static_map_server_);
        clt_get_prohibition_map_ = this->create_client<nav_msgs::srv::GetMap>(prohibition_map_server_);

        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                bool is_static_map = clt_get_static_map_->wait_for_service(std::chrono::seconds(0));
                bool is_prohibition_map = clt_get_prohibition_map_->wait_for_service(std::chrono::seconds(0));
                if (is_static_map && is_prohibition_map)
                {
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> All map services are now available.");
                    services_ready_ = true;
                    service_check_timer_->cancel();
                }
                else
                {
                    if (!is_static_map)
                        RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> Waiting for static_map service to become available...");

                    if (!is_prohibition_map)
                        RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> Waiting for prohibition_map service to become available...");
                }
            });
    }

    // Wait for transforms 
    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "MapAugmenter.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

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
                                    "MapAugmenter.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "MapAugmenter.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }

    void get_first_maps() 
    {
        rclcpp::sleep_for(std::chrono::seconds(1));

        auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(static_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static) {
                try {
                    this->static_map_ = future_static.get()->map;
                    is_static_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static map with size %d x %d", 
                                static_map_.info.width, static_map_.info.height);
                    process_maps();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static map: %s", e.what());
                }
            });

        auto prohibition_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_prohibition_map_->async_send_request(prohibition_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_ptohibition) {
                try {
                    this->prohibition_map_ = future_ptohibition.get()->map;
                    is_prohibition_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got prohibition map with size %d x %d", 
                                prohibition_map_.info.width, prohibition_map_.info.height);
                    process_maps();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get prohibition map: %s", e.what());
                }
            });
    }

    void process_maps()
    {
        if (is_static_map_ && is_prohibition_map_)
        {
            RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Updating static map with prohibition layer and static cost map...");
            
            static_map_ = merge_maps(static_map_, prohibition_map_);
            static_map_ = inflate_map(static_map_, inflation_radius_);

            static_cost_map_ = get_cost_map(static_map_, cost_radius_);
            obstacles_map_ = this->static_map_;
            for (size_t i = 0; i < obstacles_map_.data.size(); ++i)
                obstacles_map_.data[i] = 0;
            
            is_static_map_ = false;
            is_prohibition_map_ = false;
            RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Statics maps have been updated.");
        }
    }

    //############
    //Map Augmenter additional functions
    //get_robot_position() = get_relative_position("map", base_link_name_)
    //get_camera_position() = get_relative_position(base_link_name_, point_cloud_frame_)
    //get_camera_position2() = get_relative_position(base_link_name_, point_cloud_frame2_)
    //get_lidar_position() =  get_relative_position(base_link_name_, laser_scan_frame_)
    Eigen::Affine3d get_relative_position(const std::string &target_frame, const std::string &source_frame)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame, 
                    source_frame, 
                    tf2::TimePointZero
                );

            // Convert to Eigen
            Eigen::Affine3d e = tf2::transformToEigen(transform.transform);
            return e;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> TF Exception: %s", ex.what());
            return Eigen::Affine3d::Identity();
        }
    }
    
    //get_robot_position(x, y, t) = get_relative_position("map", base_link_name_, x, y, t)
    void get_relative_position(const std::string &target_frame, const std::string &source_frame, 
                               float &robot_x, float &robot_y, float &robot_t)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame, 
                    source_frame, 
                    tf2::TimePointZero
                );

            robot_x = static_cast<float>(transform.transform.translation.x);
            robot_y = static_cast<float>(transform.transform.translation.y);

            tf2::Quaternion q;
            tf2::fromMsg(transform.transform.rotation, q);
            robot_t = static_cast<float>(tf2::getYaw(q));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
            robot_x = 0.0f;
            robot_y = 0.0f;
            robot_t = 0.0f;
        }
    }

    nav_msgs::msg::OccupancyGrid merge_maps(const nav_msgs::msg::OccupancyGrid &a,
                                            const nav_msgs::msg::OccupancyGrid &b)
    {
        if (a.info.width != b.info.width || a.info.height != b.info.height)
        {
            RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> WARNING!!! Cannot merge maps of different sizes!");
            return a;
        }

        nav_msgs::msg::OccupancyGrid c = a;
        for (size_t i = 0; i < c.data.size(); ++i)
        {
            c.data[i] = static_cast<int8_t>(
                std::max(static_cast<uint8_t>(a.data[i]), static_cast<uint8_t>(b.data[i])));
        }

        return c;
    }

    nav_msgs::msg::OccupancyGrid inflate_map(const nav_msgs::msg::OccupancyGrid &map, float inflation)
    {
        /*
        * WARNING!!! It is assumed that map borders (borders with at least 'inflation' thickness)
        * are occupied or unkwnon. Map must be big enough to fulfill this assumption.
        */
        if (inflation <= 0)
            return map;

        nav_msgs::msg::OccupancyGrid new_map = map;
        int n = static_cast<int>(inflation / map.info.resolution);
        int lower_limit = n * map.info.width + n;
        int upper_limit = static_cast<int>(map.data.size()) - n * map.info.width - n;

        for (int k = lower_limit; k < upper_limit; ++k) {
            if (map.data[k] > 0) {
                for (int i = -n; i <= n; ++i) {
                    for (int j = -n; j <= n; ++j) {
                        int idx = k + j * map.info.width + i;
                        if (idx >= 0 && idx < static_cast<int>(new_map.data.size())) {
                            new_map.data[idx] = map.data[k];
                        }
                    }
                }
            }
        }

        return new_map;
    }

    nav_msgs::msg::OccupancyGrid get_cost_map(const nav_msgs::msg::OccupancyGrid &map, float cost_radius)
    {
        if (cost_radius < 0)
            return map;

        nav_msgs::msg::OccupancyGrid cost_map = map;
        int steps = static_cast<int>(cost_radius / map.info.resolution);
        int box_size = (steps * 2 + 1) * (steps * 2 + 1);
        std::vector<int> cell_costs(box_size);
        std::vector<int> neighbors(box_size);

        int counter = 0;
        for (int i = -steps; i <= steps; ++i) {
            for (int j = -steps; j <= steps; ++j) {
                neighbors[counter] = i * map.info.width + j;
                cell_costs[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1) * 2;
                ++counter;
            }
        }

        int start_idx = steps * map.info.width + steps;
        int end_idx = static_cast<int>(map.data.size()) - steps * map.info.width - steps;

        for (int i = start_idx; i < end_idx; ++i) {
            if (map.data[i] > 0) {
                for (int j = 0; j < box_size; ++j) {
                    int neighbor_idx = i + neighbors[j];
                    if (neighbor_idx >= 0 && neighbor_idx < static_cast<int>(cost_map.data.size())) {
                        if (cost_map.data[neighbor_idx] < cell_costs[j]) {
                            cost_map.data[neighbor_idx] = static_cast<int8_t>(cell_costs[j]);
                        }
                    }
                }
            }
        }

        return cost_map;
    }

    bool obstacles_map_with_cloud()
    {
        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Trying to get point cloud from topic: %s", point_cloud_topic_.c_str());
        
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            if (!point_cloud_received_ || !latest_point_cloud_) {
                RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> No new point cloud available.");
                return false;
            }
            msg = latest_point_cloud_;
            point_cloud_received_ = false;
        }

        const unsigned char* p = msg->data.data();
        int cell_x = 0;
        int cell_y = 0;
        int cell   = 0;

        Eigen::Affine3d cam_to_robot = get_relative_position(base_link_name_, msg->header.frame_id);
        Eigen::Affine3d robot_to_map = get_relative_position("map", base_link_name_);

        for (size_t i = 0; i < msg->width * msg->height; i += cloud_downsampling_)
        {
            Eigen::Vector3d v(
                *reinterpret_cast<const float*>(p),
                *reinterpret_cast<const float*>(p + 4),
                *reinterpret_cast<const float*>(p + 8));

            v = cam_to_robot * v;

            if (v.x() > min_x_ && v.x() < max_x_ &&
                v.y() > min_y_ && v.y() < max_y_ &&
                v.z() > min_z_ && v.z() < max_z_)
            {
                v = robot_to_map * v;

                cell_x = static_cast<int>((v.x() - obstacles_map_.info.origin.position.x) / obstacles_map_.info.resolution);
                cell_y = static_cast<int>((v.y() - obstacles_map_.info.origin.position.y) / obstacles_map_.info.resolution);
                cell   = cell_y * obstacles_map_.info.width + cell_x;

                if (cell >= 0 && cell < static_cast<int>(obstacles_map_.data.size())) {
                    obstacles_map_.data[cell] = 100;
                    are_there_obstacles_ = true;
                }
            }

            p += static_cast<std::size_t>(cloud_downsampling_ * msg->point_step);
        }

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> obstacles_map_with_cloud() done");

        return true;
    }

    bool obstacles_map_with_cloud2()
    {
        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Trying to get point cloud2 from topic: %s", point_cloud_topic2_.c_str());

        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(point_cloud2_mutex_);
            if (!point_cloud2_received_ || !latest_point_cloud2_) {
                RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> No new point cloud2 available.");
                return false;
            }
            msg = latest_point_cloud2_;
            point_cloud2_received_ = false;
        }

        const unsigned char* p = msg->data.data();
        int cell_x = 0;
        int cell_y = 0;
        int cell   = 0;

        Eigen::Affine3d cam_to_robot = get_relative_position(base_link_name_, msg->header.frame_id);
        Eigen::Affine3d robot_to_map = get_relative_position("map", base_link_name_);

        for (size_t i = 0; i < msg->width * msg->height; i += cloud_downsampling2_)
        {
            Eigen::Vector3d v(
                *reinterpret_cast<const float*>(p),
                *reinterpret_cast<const float*>(p + 4),
                *reinterpret_cast<const float*>(p + 8));

            v = cam_to_robot * v;

            if (v.x() > min_x_ && v.x() < max_x_ &&
                v.y() > min_y_ && v.y() < max_y_ &&
                v.z() > min_z_ && v.z() < max_z_)
            {
                v = robot_to_map * v;

                cell_x = static_cast<int>((v.x() - obstacles_map_.info.origin.position.x) / obstacles_map_.info.resolution);
                cell_y = static_cast<int>((v.y() - obstacles_map_.info.origin.position.y) / obstacles_map_.info.resolution);
                cell   = cell_y * obstacles_map_.info.width + cell_x;

                if (cell >= 0 && cell < static_cast<int>(obstacles_map_.data.size())) {
                    obstacles_map_.data[cell] = 100;
                    are_there_obstacles_ = true;
                }
            }

            p += static_cast<std::size_t>(cloud_downsampling_ * msg->point_step);
        }

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> obstacles_map_with_cloud2() done");
        return true;
    }

    bool obstacles_map_with_lidar()
    {
        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Trying to get laser scan from topic: %s", laser_scan_topic_.c_str());

        sensor_msgs::msg::LaserScan::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(laser_scan_mutex_);
            if (!laser_scan_received_ || !latest_laser_scan_) {
                RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> No new LaserScan available.");
                return false;
            }
            msg = latest_laser_scan_;
            laser_scan_received_ = false;
        }

        int cell_x = 0;
        int cell_y = 0;
        int cell   = 0;

        Eigen::Affine3d lidar_to_robot = get_relative_position(base_link_name_, msg->header.frame_id);
        Eigen::Affine3d robot_to_map = get_relative_position("map", base_link_name_);

        for (size_t i = 0; i < msg->ranges.size(); i += lidar_downsampling_)
        {
            float range = msg->ranges[i];
            float angle = msg->angle_min + i * msg->angle_increment;
            Eigen::Vector3d v(range * std::cos(angle), range * std::sin(angle), 0.0);
            v = lidar_to_robot * v;

            if (v.x() > min_x_ && v.x() < max_x_ &&
                v.y() > min_y_ && v.y() < max_y_ &&
                v.z() > min_z_ && v.z() < max_z_)
            {
                v = robot_to_map * v;
                cell_x = static_cast<int>((v.x() - obstacles_map_.info.origin.position.x) / obstacles_map_.info.resolution);
                cell_y = static_cast<int>((v.y() - obstacles_map_.info.origin.position.y) / obstacles_map_.info.resolution);
                cell   = cell_y * obstacles_map_.info.width + cell_x;

                if (cell >= 0 && cell < (int)obstacles_map_.data.size())
                {
                    obstacles_map_.data[cell] = 100;
                    are_there_obstacles_ = true;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> obstacles_map_with_lidar() done");
        return true;
    }

    bool decay_map_and_check_if_obstacles(nav_msgs::msg::OccupancyGrid& map, int decay_factor)
    {
        bool obstacles = false;
        for (size_t i = 0; i < map.data.size(); i++)
        {
            map.data[i] -= decay_factor;
            if (map.data[i] < 0)
                map.data[i] = 0;
            obstacles |= map.data[i] > 0;
        }
        return obstacles;
    }

    //############
    //Map Augmenter subscribers callbacks
    void callback_point_obstacle(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
    {    
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> new PointStamped received...");

        auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(static_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static) {
                try {
                    this->static_map_ = future_static.get()->map;
                    is_static_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static map with size %d x %d", 
                                static_map_.info.width, static_map_.info.height);
                    process_maps();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static map: %s", e.what());
                }
            });

        auto prohibition_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_prohibition_map_->async_send_request(prohibition_map_req,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_ptohibition) {
                try {
                    this->prohibition_map_ = future_ptohibition.get()->map;
                    is_prohibition_map_ = true;
                    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got prohibition map with size %d x %d", 
                                prohibition_map_.info.width, prohibition_map_.info.height);
                    process_maps();
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get prohibition map: %s", e.what());
                }
            });
    }

    //############
    //Map Augmenter services callbacks
    void callback_static_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                             std::shared_ptr<nav_msgs::srv::GetMap::Response> response) 
    {
        (void)request; // If unused
        response->map = static_map_;
    }

    void callback_static_cost_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                             std::shared_ptr<nav_msgs::srv::GetMap::Response> response) 
    {
        (void)request; // unused
        response->map = static_cost_map_;
    }

    void callback_augmented_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                             std::shared_ptr<nav_msgs::srv::GetMap::Response> response) 
    {
        if (!services_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Services not ready. Cannot handle static map request.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.->Augmenting map using: %s%s%s",
                    use_lidar_ ? "lidar " : "",
                    use_cloud_ ? "point_cloud " : "",
                    use_cloud2_ ? "point_cloud2" : "");

        if (use_online_) {
            auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
            clt_get_static_map_->async_send_request(static_map_req,
                [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static) {
                    try {
                        this->static_map_ = future_static.get()->map;
                        is_static_map_ = true;
                        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static map with size %d x %d", 
                                    static_map_.info.width, static_map_.info.height);
                        process_maps();
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static map: %s", e.what());
                    }
                });

            auto prohibition_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
            clt_get_prohibition_map_->async_send_request(prohibition_map_req,
                [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_ptohibition) {
                    try {
                        this->prohibition_map_ = future_ptohibition.get()->map;
                        is_prohibition_map_ = true;
                        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got prohibition map with size %d x %d", 
                                    prohibition_map_.info.width, prohibition_map_.info.height);
                        process_maps();
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get prohibition map: %s", e.what());
                    }
                });
        }

        if ((use_lidar_  && !obstacles_map_with_lidar()) ||
            (use_cloud_  && !obstacles_map_with_cloud())  ||
            (use_cloud2_ && !obstacles_map_with_cloud2()))
        {
            RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to add obstacles with sensors.");
            return;
        }

        obstacles_inflated_map_ = inflate_map(obstacles_map_, inflation_radius_);
        augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
        response->map = augmented_map_;

        RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Augmented map response has been sent with size %d x %d", 
                    static_map_.info.width, static_map_.info.height);
    }

    void callback_augmented_cost_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                             std::shared_ptr<nav_msgs::srv::GetMap::Response> response) 
    {
        (void)request;  // unused

        auto obs_cost_map = get_cost_map(obstacles_inflated_map_, cost_radius_);
        response->map = merge_maps(static_cost_map_, obs_cost_map);
    }

    void callback_are_there_obstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        (void)request; // unused
        response->success = are_there_obstacles_;
    }

    void callback_is_inside_obstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        (void)request;  // unused

        float robot_x, robot_y, robot_a;
        get_relative_position("map", base_link_name_, robot_x, robot_y, robot_a);

        int cell_x = static_cast<int>(
            (robot_x - augmented_map_.info.origin.position.x) / augmented_map_.info.resolution);
        int cell_y = static_cast<int>(
            (robot_y - augmented_map_.info.origin.position.y) / augmented_map_.info.resolution);
        int cell = cell_y * augmented_map_.info.width + cell_x;

        response->success = (cell >= 0 &&
                            static_cast<size_t>(cell) < augmented_map_.data.size() &&
                            augmented_map_.data[cell] > 0);
    }

    //############
    //Map Augmenter main processing
    void map_augmenter_processing() 
    {
        if (!services_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Services not ready. Cannot handle static map request.");
            return;
        }

        /*RCLCPP_INFO(this->get_logger(), 
            "MapAugmenter.-> Parameters: min_x=%.2f  max_x=%.2f  min_y=%.2f  max_y=%.2f  min_z=%.2f  max_z=%.2f",
            min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);*/

        /*RCLCPP_INFO(this->get_logger(),
            "MapAugmenter.-> Parameters: decay_factor=%.2f  inflation=%.2f  cost_radius=%.2f",
            decay_factor_, inflation_radius_, cost_radius_);*/

        /*RCLCPP_INFO(this->get_logger(),
            "MapAugmenter.-> Parameters: cloud_downsampling=%.2f  cloud_downsampling2=%.2f  lidar_downsampling=%.2f  base_link_name=%s",
            cloud_downsampling_, cloud_downsampling2_, lidar_downsampling_, base_link_name_.c_str());*/

        static int counter = 0;

        if (++counter > 10)
        {
            counter = 0;

            are_there_obstacles_ = decay_map_and_check_if_obstacles(obstacles_map_, decay_factor_);
            obstacles_inflated_map_ = inflate_map(obstacles_map_, inflation_radius_);
            augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
            pub_augmented_map_->publish(augmented_map_);
        }

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapAugmenterNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}
