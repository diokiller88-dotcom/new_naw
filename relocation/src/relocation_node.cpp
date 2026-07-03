#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> 
#include <visualization_msgs/msg/marker_array.hpp> 

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <chrono>

#include "relocation/location.hpp" 
#include "custom_msgs/msg/chassis_info.hpp" 
#include "custom_msgs/msg/vehicle_state.hpp" 

using namespace std::chrono_literals;

namespace relocation {

constexpr const char* node_name = "relocation_node";
constexpr const char* default_pcd_path = "/home/hyl/new_nav/relocation/PCD/1.pcd";
constexpr const char* default_db_path = "history_db.txt";
constexpr const char* topic_odom = "aft_mapped_to_init";
constexpr const char* topic_cloud = "cloud_registered";
constexpr const char* topic_trigger = "serial/chassis";
constexpr const char* topic_state_pub = "relocation/state";
constexpr const char* frame_id_map = "map";

constexpr int default_accumulate_frames = 50;
constexpr int pub_queue_size = 10;
constexpr int sync_queue_size = 10;

class RelocationNode : public rclcpp::Node
{
public:
    RelocationNode() : Node(node_name), is_relocated_(false), trigger_reloc_(true), map_initialized_(false), has_cached_grid_(false), accumulated_frames_(0), initial_gimbal_yaw_(0.0)
    {
        this->declare_parameter<std::string>("pcd_path", default_pcd_path);
        this->declare_parameter<std::string>("db_path", default_db_path);
        this->declare_parameter<int>("accumulate_frames", default_accumulate_frames);
        
        this->declare_parameter<double>("offset_x", 0.0);
        this->declare_parameter<double>("offset_y", 0.4);
        this->declare_parameter<bool>("sim_mode", false); 
        
        this->declare_parameter<double>("sim_odom_offset_x", 0.5);
        this->declare_parameter<double>("sim_odom_offset_y", -0.8);

        this->declare_parameter<double>("init_pose_x", 10.4);
        this->declare_parameter<double>("init_pose_y", -0.4);
        this->declare_parameter<double>("init_pose_yaw", 0.0); 

        std::string pcd_path = this->get_parameter("pcd_path").as_string();
        std::string db_path = this->get_parameter("db_path").as_string();
        accumulate_frames_max_ = this->get_parameter("accumulate_frames").as_int();
        
        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();
        sim_mode_ = this->get_parameter("sim_mode").as_bool();
        sim_odom_offset_x_ = this->get_parameter("sim_odom_offset_x").as_double();
        sim_odom_offset_y_ = this->get_parameter("sim_odom_offset_y").as_double();
        
        init_pose_x_ = this->get_parameter("init_pose_x").as_double();
        init_pose_y_ = this->get_parameter("init_pose_y").as_double();
        init_pose_yaw_ = this->get_parameter("init_pose_yaw").as_double();

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        state_pub_ = this->create_publisher<custom_msgs::msg::VehicleState>(topic_state_pub, pub_queue_size);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("relocation/current_pose", pub_queue_size);
        matched_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("relocation/matched_cloud_2d", pub_queue_size);
        
        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("relocation/map_2d", map_qos);
        
        rclcpp::QoS default_qos(10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("relocation/status_marker", default_qos);

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&RelocationNode::InitialPoseCallback, this, std::placeholders::_1));

        if (!pcd_path.empty() && pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *global_map_) != -1) {
            RCLCPP_INFO(this->get_logger(), "成功加载先验 PCD 文件, 点数: %zu", global_map_->size());
            if (loc_system_.Init(db_path, global_map_)) {
                map_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "重定位模块初始化成功！");
                Publish2DMap();
            } else {
                RCLCPP_ERROR(this->get_logger(), "重定位模块初始化失败，请检查 DB 文件路径！");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未找到或无法加载先验 PCD 文件。将自动累积里程计点云作为初始地图...");
        }

        if (sim_mode_) {
            RCLCPP_INFO(this->get_logger(), "=======================================================");
            RCLCPP_INFO(this->get_logger(), ">>> 仿真模式 (SIM MODE) 已开启 <<<");
            RCLCPP_INFO(this->get_logger(), "请在 RVIZ2 中使用 '2D Pose Estimate' 工具点击地图进行测试。");
            RCLCPP_INFO(this->get_logger(), "=======================================================");
        } else {
            RCLCPP_INFO(this->get_logger(), ">>> 实际运行模式 (REAL MODE) 已开启 <<<");
            RCLCPP_INFO(this->get_logger(), "预存先验启动点设定为: X:%.2f, Y:%.2f, Yaw:%.2f", init_pose_x_, init_pose_y_, init_pose_yaw_);
            
            auto qos = rclcpp::SensorDataQoS();
            odom_sub_.subscribe(this, topic_odom, qos.get_rmw_qos_profile());
            cloud_sub_.subscribe(this, topic_cloud, qos.get_rmw_qos_profile());
            chassis_sub_.subscribe(this, topic_trigger, qos.get_rmw_qos_profile());

            sync_.reset(new Sync(SyncPolicy(sync_queue_size), odom_sub_, cloud_sub_, chassis_sub_));
            sync_->registerCallback(std::bind(&RelocationNode::SyncedCallback, this, 
                                              std::placeholders::_1, 
                                              std::placeholders::_2, 
                                              std::placeholders::_3));
        }

        T_map_lio_ = Eigen::Isometry3d::Identity();
        vis_timer_ = this->create_wall_timer(1s, std::bind(&RelocationNode::VisTimerCallback, this));
    }

private:
    void VisTimerCallback() {
        static int tick = 0;
        if (map_initialized_ && (tick % 2 == 0)) {
            Publish2DMap();
        }
        if (!sim_mode_) {
            if (!is_relocated_) {
                PublishTargetMarker(init_pose_x_, init_pose_y_, init_pose_yaw_, true);
            } else {
                PublishTargetMarker(last_marker_x_, last_marker_y_, last_marker_yaw_, false);
            }
        }
        tick++;
    }

    void PublishTargetMarker(double x, double y, double yaw, bool is_prior) {
        visualization_msgs::msg::MarkerArray marker_array;
        builtin_interfaces::msg::Time stamp = this->now();

        visualization_msgs::msg::Marker cylinder;
        cylinder.header.frame_id = frame_id_map;
        cylinder.header.stamp = stamp;
        cylinder.ns = "relocation_target";
        cylinder.id = 0;
        cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder.action = visualization_msgs::msg::Marker::ADD;
        
        cylinder.pose.position.x = x;
        cylinder.pose.position.y = y;
        cylinder.pose.position.z = 0.5; 
        cylinder.pose.orientation.w = 1.0;
        
        cylinder.scale.x = 0.8; 
        cylinder.scale.y = 0.8;
        cylinder.scale.z = 0.2; 
        
        if (is_prior) {
            cylinder.color.r = 1.0f; cylinder.color.g = 1.0f; cylinder.color.b = 0.0f; cylinder.color.a = 1.0f; 
        } else {
            cylinder.color.r = 0.0f; cylinder.color.g = 1.0f; cylinder.color.b = 0.0f; cylinder.color.a = 1.0f; 
        }

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_map;
        arrow.header.stamp = stamp;
        arrow.ns = "relocation_target";
        arrow.id = 1;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = x;
        arrow.pose.position.y = y;
        arrow.pose.position.z = 0.7; 

        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        arrow.pose.orientation.w = q.w();
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();

        arrow.scale.x = 1.2;  
        arrow.scale.y = 0.15; 
        arrow.scale.z = 0.15; 

        if (is_prior) {
            arrow.color.r = 1.0f; arrow.color.g = 0.5f; arrow.color.b = 0.0f; arrow.color.a = 1.0f; 
        } else {
            arrow.color.r = 1.0f; arrow.color.g = 0.0f; arrow.color.b = 0.0f; arrow.color.a = 1.0f; 
        }

        marker_array.markers.push_back(cylinder);
        marker_array.markers.push_back(arrow);

        marker_pub_->publish(marker_array);
    }

    void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (!map_initialized_) {
            RCLCPP_WARN(this->get_logger(), "地图尚未初始化，请等待！");
            return;
        }

        double gt_x = msg->pose.pose.position.x;
        double gt_y = msg->pose.pose.position.y;
        
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        double yaw_deg = yaw * 180.0 / M_PI;

        if (sim_mode_) {
            double odom_x = gt_x + sim_odom_offset_x_;
            double odom_y = gt_y + sim_odom_offset_y_;
            RCLCPP_INFO(this->get_logger(), "[SIM] 收到GT输入: X:%.2f, Y:%.2f | 模拟Odom: X:%.2f, Y:%.2f", gt_x, gt_y, odom_x, odom_y);

            auto local_cloud = ExtractLocalCloud(gt_x, gt_y, yaw_deg);
            if (local_cloud->empty()) return;

            Eigen::Isometry3d T_odom_body = Eigen::Isometry3d::Identity();
            T_odom_body.translation() << odom_x, odom_y, 0.0;
            T_odom_body.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*local_cloud, *world_cloud, T_odom_body.matrix());

            auto dummy_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*world_cloud, *dummy_cloud);
            dummy_cloud->header.stamp = this->now();
            dummy_cloud->header.frame_id = frame_id_map;

            auto dummy_odom = std::make_shared<nav_msgs::msg::Odometry>();
            dummy_odom->header.stamp = dummy_cloud->header.stamp;
            dummy_odom->header.frame_id = frame_id_map;
            dummy_odom->pose.pose.position.x = odom_x;
            dummy_odom->pose.pose.position.y = odom_y;
            dummy_odom->pose.pose.position.z = 0.0;
            dummy_odom->pose.pose.orientation = msg->pose.pose.orientation;

            auto dummy_chassis = std::make_shared<custom_msgs::msg::ChassisInfo>();
            dummy_chassis->header.stamp = dummy_cloud->header.stamp;
            dummy_chassis->is_valid = true;
            dummy_chassis->trigger_relocation = true;
            dummy_chassis->trigger_target = false;
            dummy_chassis->gimbal_yaw = yaw;          
            dummy_chassis->speed = 0.0;
            dummy_chassis->target_x = 0.0;
            dummy_chassis->target_y = 0.0;

            SyncedCallback(dummy_odom, dummy_cloud, dummy_chassis);

        } else {
            init_pose_x_ = gt_x;
            init_pose_y_ = gt_y;
            init_pose_yaw_ = yaw; 

            is_relocated_ = false; 
            trigger_reloc_ = true;
            
            PublishTargetMarker(init_pose_x_, init_pose_y_, init_pose_yaw_, true);
            RCLCPP_INFO(this->get_logger(), "[REAL] RVIZ2 手动更新先验启动点: X:%.2f, Y:%.2f, Yaw:%.2f", init_pose_x_, init_pose_y_, init_pose_yaw_);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractLocalCloud(float cx, float cy, float yaw_deg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
        float yaw_rad = yaw_deg * M_PI / 180.0f;
        float cos_y = cos(yaw_rad);
        float sin_y = sin(yaw_rad);

        constexpr int yaw_bins = 3600; 
        constexpr int z_bins = 50;     
        constexpr float thickness_tolerance = 0.2f; 
        constexpr float test_max_distance = 10.0f;
        constexpr float test_z_min = 0.2f;  
        constexpr float test_z_max = 2.5f;

        std::vector<std::vector<float>> depth_buffer(yaw_bins, std::vector<float>(z_bins, test_max_distance));
        
        for (const auto& pt : global_map_->points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            float dist = std::hypot(dx, dy);
            if (dist <= test_max_distance) {
                float angle = std::atan2(dy, dx);
                if (angle < 0) angle += 2 * M_PI; 
                int y_idx = static_cast<int>((angle / (2 * M_PI)) * yaw_bins) % yaw_bins;
                float z_norm = (pt.z - test_z_min) / (test_z_max - test_z_min);
                int z_idx = std::max(0, std::min(z_bins - 1, static_cast<int>(z_norm * z_bins)));
                if (dist < depth_buffer[y_idx][z_idx]) {
                    depth_buffer[y_idx][z_idx] = dist;
                }
            }
        }

        for (const auto& pt : global_map_->points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            float dist = std::hypot(dx, dy);
            if (dist <= test_max_distance) {
                float angle = std::atan2(dy, dx);
                if (angle < 0) angle += 2 * M_PI;
                int y_idx = static_cast<int>((angle / (2 * M_PI)) * yaw_bins) % yaw_bins;
                float z_norm = (pt.z - test_z_min) / (test_z_max - test_z_min);
                int z_idx = std::max(0, std::min(z_bins - 1, static_cast<int>(z_norm * z_bins)));
                if (dist <= depth_buffer[y_idx][z_idx] + thickness_tolerance) {
                    pcl::PointXYZ p_local;
                    p_local.x = dx * cos_y + dy * sin_y;
                    p_local.y = -dx * sin_y + dy * cos_y;
                    p_local.z = pt.z;
                    local->push_back(p_local);
                }
            }
        }
        return local;
    }

    void Publish2DMap() {
        if (!global_map_ || global_map_->empty()) return;

        if (has_cached_grid_) {
            cached_grid_msg_.header.stamp = this->now();
            grid_pub_->publish(cached_grid_msg_);
            return;
        }

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();

        for (const auto& pt : global_map_->points) {
            if (pt.x < min_x) min_x = pt.x;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.y > max_y) max_y = pt.y;
        }

        float res = 0.05f; 
        int width = static_cast<int>(std::ceil((max_x - min_x) / res));
        int height = static_cast<int>(std::ceil((max_y - min_y) / res));

        cached_grid_msg_.header.stamp = this->now();
        cached_grid_msg_.header.frame_id = frame_id_map;
        cached_grid_msg_.info.resolution = res;
        cached_grid_msg_.info.width = width;
        cached_grid_msg_.info.height = height;
        cached_grid_msg_.info.origin.position.x = min_x;
        cached_grid_msg_.info.origin.position.y = min_y;
        cached_grid_msg_.info.origin.position.z = 0.0;
        cached_grid_msg_.info.origin.orientation.w = 1.0;
        
        cached_grid_msg_.data.assign(width * height, 0); 

        for (const auto& pt : global_map_->points) {
            if (pt.z >= 0.0f && pt.z <= 0.40f) {
                int gx = static_cast<int>((pt.x - min_x) / res);
                int gy = static_cast<int>((pt.y - min_y) / res);
                if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                    cached_grid_msg_.data[gy * width + gx] = 100;
                }
            }
        }
        has_cached_grid_ = true;
        grid_pub_->publish(cached_grid_msg_);
    }

    void SyncedCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg, 
                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                        const custom_msgs::msg::ChassisInfo::ConstSharedPtr& chassis_msg)
    {
        if (!chassis_msg->is_valid) return;

        if (chassis_msg->trigger_relocation) {
            trigger_reloc_ = true;
            RCLCPP_INFO(this->get_logger(), "收到重定位触发信号，正在进行粗精匹配...");
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lio_world(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud_lio_world);

        if (!map_initialized_) {
            *global_map_ += *cloud_lio_world;
            accumulated_frames_++;
            if (accumulated_frames_ >= accumulate_frames_max_) {
                std::string db_path = this->get_parameter("db_path").as_string();
                if (loc_system_.Init(db_path, global_map_)) {
                    map_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "累积建图完成 (帧数: %d), 重定位初始化成功！", accumulated_frames_);
                    Publish2DMap();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "累积地图初始化重定位失败,DB 可能缺失！");
                }
            }
            return; 
        }

        Eigen::Isometry3d T_lio_world_body = Eigen::Isometry3d::Identity();
        T_lio_world_body.translation() << odom_msg->pose.pose.position.x,
                                          odom_msg->pose.pose.position.y,
                                          odom_msg->pose.pose.position.z;
        Eigen::Quaterniond q_lio(odom_msg->pose.pose.orientation.w,
                                 odom_msg->pose.pose.orientation.x,
                                 odom_msg->pose.pose.orientation.y,
                                 odom_msg->pose.pose.orientation.z);
        T_lio_world_body.linear() = q_lio.toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_lio_world, *cloud_body, T_lio_world_body.inverse().matrix());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : cloud_body->points) {
            if (pt.z >= 0.2f && pt.z <= 2.5f) {
                cloud_body_filtered->push_back(pt);
            }
        }
        if (cloud_body_filtered->size() > 20) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_body_filtered);
            sor.setMeanK(20);
            sor.setStddevMulThresh(1.5);
            sor.filter(*cloud_body_filtered);
        }

        Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();

        if (!is_relocated_ || trigger_reloc_) {
            double rough_x, rough_y, rough_yaw_deg;
            
            double prior_x, prior_y;
            if (sim_mode_) {
                prior_x = odom_msg->pose.pose.position.x;
                prior_y = odom_msg->pose.pose.position.y;
            } else {
                if (!is_relocated_) {
                    prior_x = init_pose_x_;
                    prior_y = init_pose_y_;
                } else {
                    Eigen::Vector3d current_guess = (T_map_lio_ * T_lio_world_body).translation();
                    prior_x = current_guess.x();
                    prior_y = current_guess.y();
                }
            }
            
            if (loc_system_.SetRoughPoseWithPrePose(cloud_body_filtered, prior_x, prior_y, 0.0, rough_x, rough_y, rough_yaw_deg)) {
                double rough_yaw_rad = rough_yaw_deg * M_PI / 180.0;
                
                Eigen::Isometry3d T_rough = Eigen::Isometry3d::Identity();
                T_rough.translation() << rough_x, rough_y, odom_msg->pose.pose.position.z; 
                T_rough.linear() = Eigen::AngleAxisd(rough_yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                Eigen::Matrix3d R_prec = T_rough.linear();
                Eigen::Vector3d T_prec = T_rough.translation();

                if (loc_system_.SetPrecisePose(cloud_body_filtered, R_prec, T_prec)) {
                    is_relocated_ = true;
                    trigger_reloc_ = false; 
                    initial_gimbal_yaw_ = chassis_msg->gimbal_yaw;
                    
                    T_map_body.linear() = R_prec;
                    T_map_body.translation() = T_prec;
                    T_map_lio_ = T_map_body * T_lio_world_body.inverse();

                    last_marker_x_ = T_prec.x();
                    last_marker_y_ = T_prec.y();
                    last_marker_yaw_ = std::atan2(R_prec(1, 0), R_prec(0, 0));

                    RCLCPP_INFO(this->get_logger(), "重定位成功! Map系下位姿: X:%.2f, Y:%.2f", T_prec.x(), T_prec.y());

                    PublishVehicleState(T_map_body, chassis_msg->speed, chassis_msg->gimbal_yaw, chassis_msg->header.stamp);
                    PublishPoseForRViz(T_map_body, chassis_msg->header.stamp);
                    PublishMatchedCloud2D(cloud_body_filtered, T_map_body, chassis_msg->header.stamp);
                } else {
                    RCLCPP_WARN(this->get_logger(), "粗定位成功，但 ICP 精定位匹配失败。");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "粗定位匹配失败，继续尝试...");
            }
        } else {
            Eigen::Isometry3d T_map_body_guess = T_map_lio_ * T_lio_world_body;
            Eigen::Matrix3d R_guess = T_map_body_guess.linear();
            Eigen::Vector3d T_guess = T_map_body_guess.translation();

            if (loc_system_.SetPrecisePose(cloud_body_filtered, R_guess, T_guess)) {
                T_map_body.linear() = R_guess;
                T_map_body.translation() = T_guess;
                T_map_lio_ = T_map_body * T_lio_world_body.inverse();

                last_marker_x_ = T_guess.x();
                last_marker_y_ = T_guess.y();
                last_marker_yaw_ = std::atan2(R_guess(1, 0), R_guess(0, 0));
                PublishVehicleState(T_map_body, chassis_msg->speed, chassis_msg->gimbal_yaw, chassis_msg->header.stamp);
                PublishPoseForRViz(T_map_body, chassis_msg->header.stamp);
                PublishMatchedCloud2D(cloud_body_filtered, T_map_body, chassis_msg->header.stamp);
            } else {
                RCLCPP_WARN(this->get_logger(), "追踪过程中精定位失败，等待下一帧...");
            }
        }
    }

    void PublishMatchedCloud2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, const Eigen::Isometry3d& T_map_body, const builtin_interfaces::msg::Time& stamp)
    {
        if (local_cloud->empty()) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*local_cloud, *cloud_map, T_map_body.matrix());
        for (auto& pt : cloud_map->points) pt.z = 0.05f; 

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_map, cloud_msg);
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = frame_id_map;
        matched_cloud_pub_->publish(cloud_msg);
    }

    void PublishVehicleState(const Eigen::Isometry3d& T_map_body, double synchronized_speed, double current_gimbal_yaw, const builtin_interfaces::msg::Time& stamp)
    {
        custom_msgs::msg::VehicleState state_msg;
        state_msg.header.stamp = stamp;
        state_msg.header.frame_id = frame_id_map;

        double map_yaw = std::atan2(T_map_body.linear()(1, 0), T_map_body.linear()(0, 0));
        double yaw_diff = current_gimbal_yaw - initial_gimbal_yaw_;
        double chassis_yaw = map_yaw - yaw_diff;

        while (chassis_yaw > M_PI) chassis_yaw -= 2.0 * M_PI;
        while (chassis_yaw < -M_PI) chassis_yaw += 2.0 * M_PI;

        state_msg.pos_x = T_map_body.translation().x() - (offset_x_ * std::cos(map_yaw) - offset_y_ * std::sin(map_yaw));
        state_msg.pos_y = T_map_body.translation().y() - (offset_x_ * std::sin(map_yaw) + offset_y_ * std::cos(map_yaw));
        state_msg.yaw = chassis_yaw;
        state_msg.vel_x = synchronized_speed * std::cos(chassis_yaw);
        state_msg.vel_y = synchronized_speed * std::sin(chassis_yaw);
        state_msg.is_valid = true;
        state_pub_->publish(state_msg);
    }

    void PublishPoseForRViz(const Eigen::Isometry3d& T_map_body, const builtin_interfaces::msg::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_map;
        pose_msg.pose.position.x = T_map_body.translation().x();
        pose_msg.pose.position.y = T_map_body.translation().y();
        pose_msg.pose.position.z = T_map_body.translation().z();
        Eigen::Quaterniond q(T_map_body.linear());
        pose_msg.pose.orientation.w = q.w();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_pub_->publish(pose_msg);
    }

    location loc_system_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    nav_msgs::msg::OccupancyGrid cached_grid_msg_;
    
    bool map_initialized_;
    bool has_cached_grid_;
    bool sim_mode_;
    int accumulate_frames_max_;
    int accumulated_frames_;
    bool is_relocated_;
    bool trigger_reloc_;

    double offset_x_;
    double offset_y_;
    double sim_odom_offset_x_;
    double sim_odom_offset_y_;
    double init_pose_x_; 
    double init_pose_y_; 
    double init_pose_yaw_; 
    double initial_gimbal_yaw_;
    
    double last_marker_x_{0.0}, last_marker_y_{0.0}, last_marker_yaw_{0.0};

    Eigen::Isometry3d T_map_lio_;

    rclcpp::TimerBase::SharedPtr vis_timer_; 
    
    rclcpp::Publisher<custom_msgs::msg::VehicleState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr matched_cloud_pub_; 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_; 
    
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<custom_msgs::msg::ChassisInfo> chassis_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry, 
        sensor_msgs::msg::PointCloud2, 
        custom_msgs::msg::ChassisInfo> SyncPolicy;
        
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};

} // namespace relocation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<relocation::RelocationNode>());
    rclcpp::shutdown();
    return 0;
}
