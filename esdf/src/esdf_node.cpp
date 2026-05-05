#include "esdf/feasible_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "custom_msgs/msg/vehicle_state.hpp"
#include "custom_msgs/msg/map_data.hpp"
#include <memory>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

constexpr double mid360z2vehiclestate = 0.01;

class ESDFNode : public rclcpp::Node {
public:
    ESDFNode() : Node("esdf_node"), initialized_(false) {
        
        this->declare_parameter<bool>("use_offline_pcd", true);
        this->declare_parameter<std::string>("pcd_path", "/home/hyl/new_nav/esdf/PCD/1.pcd");
        this->declare_parameter<bool>("debug_mode", true);

        bool use_offline;
        std::string pcd_path;
        this->get_parameter("use_offline_pcd", use_offline);
        this->get_parameter("pcd_path", pcd_path);
        this->get_parameter("debug_mode", debug_mode_);

        map_.InitMap(800, 800);

        rclcpp::QoS map_qos(1);
        map_qos.transient_local(); 
        map_pub_ = this->create_publisher<custom_msgs::msg::MapData>("esdf/map", map_qos);

        if (debug_mode_) {
            pub_global_occ_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug/global_occupancy", map_qos);
            pub_global_esdf_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug/global_esdf", map_qos);
            pub_local_occ_   = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug/local_occupancy", 10);
            pub_local_esdf_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug/local_esdf", 10);
        }

        if (use_offline) {
            RCLCPP_INFO(this->get_logger(), "Mode 1: Loading offline PCD as prior map from %s...", pcd_path.c_str());
            try {
                map_.InitFromPCD(pcd_path);
                map_.PreMapDownSample();
                initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Offline map loaded & ESDF initialized.");
                
                PublishMapData(); 
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", e.what());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Mode 2: Waiting for accumulated global pointcloud topic...");
            global_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "cloud_registered", 1, 
                std::bind(&ESDFNode::globalCloudCallback, this, std::placeholders::_1));
        }

        local_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,
            std::bind(&ESDFNode::localCloudCallback, this, std::placeholders::_1));
            
        state_sub_ = this->create_subscription<custom_msgs::msg::VehicleState>(
            "relocation/state", 10,
            std::bind(&ESDFNode::stateCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1s, std::bind(&ESDFNode::timerCallback, this));
    }

private:
    void timerCallback() {
        if (initialized_) {
            PublishMapData();
        }
    }

    void globalCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (!initialized_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *cloud_pcl);
            map_.Init(cloud_pcl);
            map_.PreMapDownSample();
            initialized_ = true;
            PublishMapData();
        }
    }

    void stateCallback(const custom_msgs::msg::VehicleState::ConstSharedPtr msg) {
        latest_state_ = msg;
    }

    void localCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (!initialized_ || !latest_state_ || !latest_state_->is_valid) return;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *local_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        global_cloud->points.reserve(local_cloud->points.size());

        double tx = latest_state_->pos_x;
        double ty = latest_state_->pos_y;
        double yaw = latest_state_->yaw;
        double tz = mid360z2vehiclestate;
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);

        for(const auto& pt : local_cloud->points) {
            pcl::PointXYZ g_pt;
            g_pt.x = cos_yaw * pt.x - sin_yaw * pt.y + tx;
            g_pt.y = sin_yaw * pt.x + cos_yaw * pt.y + ty;
            g_pt.z = pt.z + tz;
            global_cloud->points.push_back(g_pt);
        }

        if (map_.Update(global_cloud, tx, ty)) {
            PublishMapData();
        }
    }

    void PublishMapData() {
        custom_msgs::msg::MapData map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.is_valid = true;

        map_msg.width = map_.getMapWidth();
        map_msg.height = map_.getMapHeight();
        map_msg.res_x = esdf_map::leafX;
        map_msg.res_y = esdf_map::leafY;
        map_msg.origin_x = esdf_map::minX;
        map_msg.origin_y = esdf_map::minY;

        const auto& global_esdf = map_.getGlobalESDF();
        const auto& global_map_state = map_.getGlobalMap();

        int size = map_msg.width * map_msg.height;
        map_msg.occupancy_array.resize(size);
        map_msg.esdf_array.resize(size);

        for (int i = 0; i < size; ++i) {
            bool is_occupied = (global_map_state[i].state == esdf_map::grip_state::occupy);
            map_msg.occupancy_array[i] = is_occupied ? 1 : 0;
            double raw_val = global_esdf[i];
            map_msg.esdf_array[i] = (raw_val < 0) ? -std::sqrt(-raw_val) * map_msg.res_x : std::sqrt(raw_val) * map_msg.res_x;
        }

        map_pub_->publish(map_msg);

        if (debug_mode_) {
            PublishDebugMaps(global_map_state, map_msg.esdf_array, map_msg.width, map_msg.height, 
                             map_msg.res_x, map_msg.res_y, map_msg.origin_x, map_msg.origin_y);
        }
    }

    void PublishDebugMaps(const auto& global_map_state, const auto& global_esdf_meters, 
                          int width, int height, double res_x, double res_y, 
                          double origin_x, double origin_y) {
        
        nav_msgs::msg::OccupancyGrid global_occ_msg;
        nav_msgs::msg::OccupancyGrid global_esdf_msg;
        auto stamp = this->now();
        
        global_occ_msg.header.stamp = stamp;
        global_occ_msg.header.frame_id = "map"; 
        global_occ_msg.info.resolution = res_x;
        global_occ_msg.info.width = width;
        global_occ_msg.info.height = height;
        global_occ_msg.info.origin.position.x = origin_x;
        global_occ_msg.info.origin.position.y = origin_y;
        global_occ_msg.data.resize(width * height, 0);

        global_esdf_msg = global_occ_msg; 
        const double max_esdf_dist = 0.8; 

        for (int i = 0; i < width * height; ++i) {
            bool is_occupied = (global_map_state[i].state == esdf_map::grip_state::occupy);
            global_occ_msg.data[i] = is_occupied ? 100 : 0;

            double dist_meters = global_esdf_meters[i]; 
            
            if (dist_meters <= 0.0) {
                global_esdf_msg.data[i] = 100;
            } else if (dist_meters >= max_esdf_dist) {
                global_esdf_msg.data[i] = 0;
            } else {
                global_esdf_msg.data[i] = static_cast<int8_t>((1.0 - (dist_meters / max_esdf_dist)) * 100.0);
            }
        }

        pub_global_occ_->publish(global_occ_msg);
        pub_global_esdf_->publish(global_esdf_msg);

        if (!latest_state_ || !latest_state_->is_valid) return;

        double local_w_pixels = esdf_map::local_W;
        double local_h_pixels = esdf_map::local_H;

        int cx = static_cast<int>((latest_state_->pos_x - origin_x) / res_x);
        int cy = static_cast<int>((latest_state_->pos_y - origin_y) / res_y);

        int start_x = std::max(0, cx - esdf_map::windowX);
        int start_y = std::max(0, cy - esdf_map::windowY);
        int end_x = std::min(width, start_x + static_cast<int>(local_w_pixels));
        int end_y = std::min(height, start_y + static_cast<int>(local_h_pixels));

        if (end_x - start_x <= 0 || end_y - start_y <= 0) return;

        nav_msgs::msg::OccupancyGrid local_occ_msg, local_esdf_msg;
        local_occ_msg.header.stamp = stamp;
        local_occ_msg.header.frame_id = "map";
        local_occ_msg.info.resolution = res_x;
        local_occ_msg.info.width = end_x - start_x;
        local_occ_msg.info.height = end_y - start_y;
        local_occ_msg.info.origin.position.x = origin_x + start_x * res_x;
        local_occ_msg.info.origin.position.y = origin_y + start_y * res_y;
        
        local_esdf_msg = local_occ_msg;
        local_occ_msg.data.resize((end_x - start_x) * (end_y - start_y));
        local_esdf_msg.data.resize((end_x - start_x) * (end_y - start_y));

        for (int y = 0; y < end_y - start_y; ++y) {
            for (int x = 0; x < end_x - start_x; ++x) {
                int global_idx = (start_y + y) * width + (start_x + x);
                int local_idx = y * (end_x - start_x) + x;
                
                local_occ_msg.data[local_idx] = global_occ_msg.data[global_idx];
                local_esdf_msg.data[local_idx] = global_esdf_msg.data[global_idx];
            }
        }

        pub_local_occ_->publish(local_occ_msg);
        pub_local_esdf_->publish(local_esdf_msg);
    }

    esdf_map::feasible_map map_;
    bool initialized_;
    bool debug_mode_; 

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_sub_;
    rclcpp::Subscription<custom_msgs::msg::VehicleState>::SharedPtr state_sub_;
    
    rclcpp::Publisher<custom_msgs::msg::MapData>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_occ_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_esdf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_occ_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_esdf_;
    
    rclcpp::TimerBase::SharedPtr timer_; 
    
    custom_msgs::msg::VehicleState::ConstSharedPtr latest_state_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESDFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}