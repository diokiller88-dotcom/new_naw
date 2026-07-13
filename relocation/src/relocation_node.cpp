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
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "relocation/location.hpp" 
#include "custom_msgs/msg/chassis_info.hpp" 
#include "custom_msgs/msg/vehicle_state.hpp" 

using namespace std::chrono_literals;

namespace relocation {

constexpr const char* node_name = "relocation_node";
constexpr const char* default_pcd_path = "PCD/1.pcd";
constexpr const char* default_db_path = "history_db.txt";
constexpr const char* topic_odom = "aft_mapped_to_init";
constexpr const char* topic_cloud = "cloud_registered";
constexpr const char* topic_trigger = "serial/chassis";
constexpr const char* topic_state_pub = "relocation/state";
constexpr const char* frame_id_map = "map";

constexpr int default_accumulate_frames = 50;
constexpr int pub_queue_size = 10;
constexpr int sync_queue_size = 10;
constexpr double reloc_min_rough_score_gap = 0.02;
constexpr double reloc_min_rough_score_ratio = 1.10;
constexpr double reloc_min_gicp_error_gap = 0.05;
constexpr double reloc_min_gicp_error_ratio = 1.08;
constexpr int reloc_min_gicp_valid_count = 30;
constexpr double reloc_min_gicp_valid_ratio = 0.02;
constexpr size_t reloc_max_gicp_candidates = 3;
constexpr double reloc_max_rough_score_ratio_for_gicp = 1.35;
constexpr double reloc_early_accept_gicp_error = 0.25;
constexpr int reloc_candidate_gicp_max_iterations = 10;
constexpr float reloc_candidate_gicp_voxel_leaf_size = 0.35f;

struct GicpCandidateResult {
    RoughPoseCandidate rough;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T = Eigen::Vector3d::Zero();
    double gicp_error = std::numeric_limits<double>::max();
    int valid_count = 0;
    int source_point_count = 0;
    bool xicp_triggered = false;
};

std::string StripPackagePrefix(const std::filesystem::path& path)
{
    auto iter = path.begin();
    if (iter != path.end() && *iter == "relocation") {
        std::filesystem::path stripped;
        ++iter;
        for (; iter != path.end(); ++iter) {
            stripped /= *iter;
        }
        return stripped.string();
    }
    return path.string();
}

std::string ResolveReadablePath(const std::string& path)
{
    namespace fs = std::filesystem;
    if (path.empty()) return path;

    fs::path input(path);
    if (input.is_absolute()) return path;

    fs::path cwd_candidate = fs::current_path() / input;
    if (fs::exists(cwd_candidate)) return cwd_candidate.string();

    try {
        fs::path share_dir(ament_index_cpp::get_package_share_directory("relocation"));
        fs::path share_candidate = share_dir / input;
        if (fs::exists(share_candidate)) return share_candidate.string();

        fs::path stripped_candidate = share_dir / StripPackagePrefix(input);
        if (fs::exists(stripped_candidate)) return stripped_candidate.string();
    } catch (const std::exception&) {
    }

    return path;
}

bool HasEnoughScoreSeparation(double best, double second, double min_gap, double min_ratio)
{
    double gap = second - best;
    double ratio = second / std::max(best, 1e-6);
    return gap >= min_gap || ratio >= min_ratio;
}

class RelocationNode : public rclcpp::Node
{
public:
    RelocationNode()
        : Node(node_name),
          map_initialized_(false),
          has_cached_grid_(false),
          sim_mode_(false),
          accumulate_frames_max_(default_accumulate_frames),
          accumulated_frames_(0),
          is_relocated_(false),
          trigger_reloc_(true),
          initial_gimbal_yaw_(0.0)
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

        std::string pcd_path = ResolveReadablePath(this->get_parameter("pcd_path").as_string());
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
            double prior_x, prior_y, prior_yaw;
            if (sim_mode_) {
                prior_x = odom_msg->pose.pose.position.x;
                prior_y = odom_msg->pose.pose.position.y;
                prior_yaw = std::atan2(T_lio_world_body.linear()(1, 0), T_lio_world_body.linear()(0, 0));
            } else {
                if (!is_relocated_) {
                    prior_x = init_pose_x_;
                    prior_y = init_pose_y_;
                    prior_yaw = init_pose_yaw_;
                } else {
                    Eigen::Isometry3d T_map_body_guess = T_map_lio_ * T_lio_world_body;
                    Eigen::Vector3d current_guess = T_map_body_guess.translation();
                    prior_x = current_guess.x();
                    prior_y = current_guess.y();
                    prior_yaw = std::atan2(T_map_body_guess.linear()(1, 0), T_map_body_guess.linear()(0, 0));
                }
            }

            std::vector<RoughPoseCandidate> rough_candidates;
            if (loc_system_.GetRoughPoseCandidatesWithPrePose(cloud_body_filtered, prior_x, prior_y, prior_yaw, rough_candidates)) {
                std::vector<GicpCandidateResult> gicp_results;
                gicp_results.reserve(rough_candidates.size());
                double best_rough_score = rough_candidates.front().rough_score;
                size_t attempted_gicp_count = 0;

                for (const auto& candidate : rough_candidates) {
                    if (attempted_gicp_count >= reloc_max_gicp_candidates) break;
                    double rough_ratio = candidate.rough_score / std::max(best_rough_score, 1e-6);
                    if (attempted_gicp_count > 0 && rough_ratio > reloc_max_rough_score_ratio_for_gicp) {
                        break;
                    }

                    attempted_gicp_count++;
                    double rough_yaw_rad = candidate.yaw_deg * M_PI / 180.0;
                    Eigen::Matrix3d R_prec = Eigen::AngleAxisd(rough_yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    Eigen::Vector3d T_prec(candidate.x, candidate.y, odom_msg->pose.pose.position.z);
                    double gicp_error = std::numeric_limits<double>::max();
                    int valid_count = 0;

                    if (!loc_system_.SetPrecisePose(cloud_body_filtered, R_prec, T_prec, gicp_error, valid_count,
                                                    reloc_candidate_gicp_max_iterations,
                                                    reloc_candidate_gicp_voxel_leaf_size)) {
                        continue;
                    }

                    GicpCandidateResult result;
                    result.rough = candidate;
                    result.R = R_prec;
                    result.T = T_prec;
                    result.gicp_error = gicp_error;
                    result.valid_count = valid_count;
                    result.source_point_count = loc_system_.GetLastSourcePointCount();
                    result.xicp_triggered = loc_system_.WasLastXicpTriggered();
                    gicp_results.push_back(result);

                    double valid_ratio = result.source_point_count <= 0
                                             ? 0.0
                                             : static_cast<double>(valid_count) / static_cast<double>(result.source_point_count);
                    bool enough_inliers = valid_count >= reloc_min_gicp_valid_count &&
                                          valid_ratio >= reloc_min_gicp_valid_ratio;
                    bool rough_unique_now = rough_candidates.size() < 2 ||
                                            HasEnoughScoreSeparation(rough_candidates[0].rough_score,
                                                                     rough_candidates[1].rough_score,
                                                                     reloc_min_rough_score_gap,
                                                                     reloc_min_rough_score_ratio);
                    if (rough_unique_now && enough_inliers && gicp_error <= reloc_early_accept_gicp_error) {
                        break;
                    }
                }

                if (!gicp_results.empty()) {
                    std::sort(gicp_results.begin(), gicp_results.end(), [](const auto& lhs, const auto& rhs) {
                        if (std::abs(lhs.gicp_error - rhs.gicp_error) > 1e-4) return lhs.gicp_error < rhs.gicp_error;
                        if (lhs.valid_count != rhs.valid_count) return lhs.valid_count > rhs.valid_count;
                        return lhs.rough.rough_score < rhs.rough.rough_score;
                    });

                    const auto& best = gicp_results.front();
                    bool rough_unique = rough_candidates.size() < 2 ||
                                        HasEnoughScoreSeparation(rough_candidates[0].rough_score,
                                                                 rough_candidates[1].rough_score,
                                                                 reloc_min_rough_score_gap,
                                                                 reloc_min_rough_score_ratio);
                    bool gicp_unique = gicp_results.size() < 2 ||
                                       HasEnoughScoreSeparation(best.gicp_error,
                                                                gicp_results[1].gicp_error,
                                                                reloc_min_gicp_error_gap,
                                                                reloc_min_gicp_error_ratio);
                    double valid_ratio = best.source_point_count <= 0
                                             ? 0.0
                                             : static_cast<double>(best.valid_count) / static_cast<double>(best.source_point_count);
                    bool enough_inliers = best.valid_count >= reloc_min_gicp_valid_count &&
                                          valid_ratio >= reloc_min_gicp_valid_ratio;

                    if (!enough_inliers) {
                        RCLCPP_WARN(this->get_logger(),
                                    "拒绝重定位：GICP有效匹配不足。Candidates:%zu, GicpTried:%zu, GicpOK:%zu, Valid:%d, Source:%d, Ratio:%.3f, Error:%.4f",
                                    rough_candidates.size(), attempted_gicp_count, gicp_results.size(), best.valid_count,
                                    best.source_point_count, valid_ratio, best.gicp_error);
                        return;
                    }

                    if (!rough_unique && !gicp_unique) {
                        double rough_gap = rough_candidates[1].rough_score - rough_candidates[0].rough_score;
                        double gicp_gap = gicp_results.size() >= 2 ? gicp_results[1].gicp_error - best.gicp_error : 0.0;
                        RCLCPP_WARN(this->get_logger(),
                                    "拒绝重定位：粗匹配和GICP均低置信。Candidates:%zu, GicpTried:%zu, GicpOK:%zu, RoughGap:%.4f, GicpGap:%.4f, BestHist:%d, BestError:%.4f",
                                    rough_candidates.size(), attempted_gicp_count, gicp_results.size(), rough_gap, gicp_gap, best.rough.hist_index, best.gicp_error);
                        return;
                    }

                    Eigen::Matrix3d final_R = best.R;
                    Eigen::Vector3d final_T = best.T;
                    double final_gicp_error = std::numeric_limits<double>::max();
                    int final_valid_count = 0;
                    if (!loc_system_.SetPrecisePose(cloud_body_filtered, final_R, final_T,
                                                    final_gicp_error, final_valid_count)) {
                        RCLCPP_WARN(this->get_logger(),
                                    "拒绝重定位：最佳候选通过粗GICP筛选，但最终精GICP失败。Candidates:%zu, GicpTried:%zu, GicpOK:%zu, BestHist:%d",
                                    rough_candidates.size(), attempted_gicp_count, gicp_results.size(), best.rough.hist_index);
                        return;
                    }
                    const bool final_xicp_triggered = loc_system_.WasLastXicpTriggered();

                    is_relocated_ = true;
                    trigger_reloc_ = false; 
                    initial_gimbal_yaw_ = chassis_msg->gimbal_yaw;
                    
                    T_map_body.linear() = final_R;
                    T_map_body.translation() = final_T;
                    T_map_lio_ = T_map_body * T_lio_world_body.inverse();

                    last_marker_x_ = final_T.x();
                    last_marker_y_ = final_T.y();
                    last_marker_yaw_ = std::atan2(final_R(1, 0), final_R(0, 0));

                    RCLCPP_INFO(this->get_logger(),
                                "重定位成功! 候选数:%zu, GicpTried:%zu, GicpOK:%zu, Hist:%d, RoughScore:%.4f, CoarseGICP:%.4f, FinalGICP:%.4f, FinalValid:%d, CoarseSource:%d, Ratio:%.3f, RoughUnique:%d, GicpUnique:%d, CoarseXICP:%d, FinalXICP:%d, Map系下位姿: X:%.2f, Y:%.2f",
                                rough_candidates.size(), attempted_gicp_count, gicp_results.size(), best.rough.hist_index, best.rough.rough_score,
                                best.gicp_error, final_gicp_error, final_valid_count, best.source_point_count, valid_ratio, rough_unique, gicp_unique,
                                best.xicp_triggered, final_xicp_triggered, final_T.x(), final_T.y());

                    PublishVehicleState(T_map_body, chassis_msg->speed, chassis_msg->gimbal_yaw, chassis_msg->header.stamp);
                    PublishPoseForRViz(T_map_body, chassis_msg->header.stamp);
                    PublishMatchedCloud2D(cloud_body_filtered, T_map_body, chassis_msg->header.stamp);
                } else {
                    RCLCPP_WARN(this->get_logger(), "粗定位候选数: %zu, GicpTried:%zu，但全部 GICP 精定位失败。", rough_candidates.size(), attempted_gicp_count);
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
                if (loc_system_.WasLastXicpTriggered()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "局部定位 GICP 触发 X-ICP 退化约束。");
                }

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
