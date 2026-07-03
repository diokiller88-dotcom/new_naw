#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <cmath>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> 
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "planner_2d/replanner.hpp"
#include "custom_msgs/msg/map_data.hpp"      
#include "custom_msgs/msg/vehicle_state.hpp" 
#include "custom_msgs/msg/chassis_info.hpp"          
#include "custom_msgs/msg/result.hpp"        

namespace planner_2d {

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node"), m_HasMap(false), m_HasPose(false), m_HasGoal(false), m_IsInitialPlanned(false),
                    m_HasRVizGoal(false), m_HasChassisGoal(false) {
        
        this->declare_parameter<bool>("use_offline_map", false);
        this->declare_parameter<std::string>("occupancy_file", "esdf_occupancy.txt");
        this->declare_parameter<std::string>("esdf_file", "esdf_distance.txt");
        this->declare_parameter<std::string>("map_info_file", "map_info.txt");
        this->declare_parameter<double>("init_pose_x", 10.4);
        this->declare_parameter<double>("init_pose_y", -0.4);
        this->declare_parameter<double>("init_pose_yaw", 0.0);

        bool use_offline_map = false; 
        this->get_parameter("use_offline_map", use_offline_map);

        double init_x = this->get_parameter("init_pose_x").as_double();
        double init_y = this->get_parameter("init_pose_y").as_double();
        m_CurYaw = this->get_parameter("init_pose_yaw").as_double();
        
        m_CurPose << init_x, init_y;
        m_CurVel << 0.0, 0.0; 

        m_ResultPub = this->create_publisher<custom_msgs::msg::Result>("planner/result", 10);
        m_PathPub = this->create_publisher<nav_msgs::msg::Path>("planner/trajectory", 10);
        m_GoalMarkerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("planner/goal_markers", 10);

        m_InitialPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&PlannerNode::InitialPoseCallback, this, std::placeholders::_1));

        if (!use_offline_map) {
            m_GlobalMapSub = this->create_subscription<custom_msgs::msg::MapData>(
                "esdf/map", 1, std::bind(&PlannerNode::MapCallback, this, std::placeholders::_1));
        } else {
            LoadOfflineMap();
        }

        m_StateSubSync.subscribe(this, "relocation/state");
        m_LocalMapSubSync.subscribe(this, "esdf/local_map");

        m_Sync.reset(new Sync(SyncPolicy(10), m_StateSubSync, m_LocalMapSubSync));
        m_Sync->registerCallback(std::bind(&PlannerNode::SyncedStateMapCallback, this, 
                                           std::placeholders::_1, std::placeholders::_2));

        m_ChassisInfoSub = this->create_subscription<custom_msgs::msg::ChassisInfo>(
            "serial/chassis", 1, std::bind(&PlannerNode::ChassisInfoCallback, this, std::placeholders::_1));

        m_GoalSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PlannerNode::RVizGoalCallback, this, std::placeholders::_1));

        m_LastTargetGoal << 1e9, 1e9; 
    }

private:
    void MaybePlan(bool has_local_map = false, const custom_msgs::msg::MapData::ConstSharedPtr& local_map_msg = nullptr,
                   const builtin_interfaces::msg::Time* stamp = nullptr) {
        if (!m_HasMap || !m_HasPose || !m_HasGoal) return;

        if (!m_IsInitialPlanned) {
            if (m_Replanner.InitPoint(m_CurPose.x(), m_CurPose.y(), m_TargetGoal.x(), m_TargetGoal.y())) {
                m_IsInitialPlanned = true;
                m_LastTargetGoal = m_TargetGoal;
            } else {
                return;
            }
        } else if ((m_TargetGoal - m_LastTargetGoal).norm() > 0.05) {
            Eigen::Vector2d zero_velocity(0.0, 0.0);
            if (!m_Replanner.UpdateGoal(m_CurPose, m_TargetGoal, zero_velocity)) return;
            m_LastTargetGoal = m_TargetGoal;
        } else if (has_local_map && local_map_msg) {
            if (!m_Replanner.Update(m_CurPose, m_CurVel,
                                    local_map_msg->occupancy_array, local_map_msg->esdf_array,
                                    local_map_msg->width, local_map_msg->height,
                                    local_map_msg->origin_x, local_map_msg->origin_y)) {
                return;
            }
        } else {
            if (!m_Replanner.Update(m_CurPose, m_CurVel)) return;
        }

        PublishTrajectory();
        if (stamp) PublishResult(*stamp);
        else PublishResult(this->now());

        if (m_Replanner.WasLastPlanTriggered()) {
            if (m_Replanner.GetLastPlanType() == "global") {
                RCLCPP_INFO(this->get_logger(), "Replanning mode: %s, total time: %.2f ms, jps time: %.2f ms",
                            m_Replanner.GetLastPlanType().c_str(), m_Replanner.GetLastPlanDurationMs(),
                            m_Replanner.GetLastJpsDurationMs());
            } else {
                RCLCPP_INFO(this->get_logger(), "Replanning mode: %s, time: %.2f ms",
                            m_Replanner.GetLastPlanType().c_str(), m_Replanner.GetLastPlanDurationMs());
            }
        }
    }

    void PublishGoalMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        builtin_interfaces::msg::Time stamp = this->now();

        if (m_HasRVizGoal) {
            visualization_msgs::msg::Marker rviz_marker;
            rviz_marker.header.frame_id = "map";
            rviz_marker.header.stamp = stamp;
            rviz_marker.ns = "rviz_target";
            rviz_marker.id = 0;
            rviz_marker.type = visualization_msgs::msg::Marker::SPHERE;
            rviz_marker.action = visualization_msgs::msg::Marker::ADD;
            rviz_marker.pose.position.x = m_RVizGoal.x();
            rviz_marker.pose.position.y = m_RVizGoal.y();
            rviz_marker.pose.position.z = 0.2; 
            rviz_marker.pose.orientation.w = 1.0;
            rviz_marker.scale.x = 0.4; rviz_marker.scale.y = 0.4; rviz_marker.scale.z = 0.4;
            rviz_marker.color.r = 1.0f; rviz_marker.color.g = 0.0f; rviz_marker.color.b = 1.0f; rviz_marker.color.a = 0.8f;
            marker_array.markers.push_back(rviz_marker);
        }

        if (m_HasChassisGoal) {
            visualization_msgs::msg::Marker chassis_marker;
            chassis_marker.header.frame_id = "map";
            chassis_marker.header.stamp = stamp;
            chassis_marker.ns = "chassis_target";
            chassis_marker.id = 1;
            chassis_marker.type = visualization_msgs::msg::Marker::SPHERE;
            chassis_marker.action = visualization_msgs::msg::Marker::ADD;
            chassis_marker.pose.position.x = m_ChassisGoal.x();
            chassis_marker.pose.position.y = m_ChassisGoal.y();
            chassis_marker.pose.position.z = 0.2; 
            chassis_marker.pose.orientation.w = 1.0;
            chassis_marker.scale.x = 0.4; chassis_marker.scale.y = 0.4; chassis_marker.scale.z = 0.4;
            chassis_marker.color.r = 0.0f; chassis_marker.color.g = 1.0f; chassis_marker.color.b = 1.0f; chassis_marker.color.a = 0.8f;
            marker_array.markers.push_back(chassis_marker);
        }
        if (!marker_array.markers.empty()) m_GoalMarkerPub->publish(marker_array);
    }

    void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        m_CurPose.x() = msg->pose.pose.position.x;
        m_CurPose.y() = msg->pose.pose.position.y;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        m_CurYaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        m_HasPose = true;
        MaybePlan();
    }

    void LoadOfflineMap() {
        std::string occ_path, esdf_path, info_path;
        this->get_parameter("occupancy_file", occ_path);
        this->get_parameter("esdf_file", esdf_path);
        this->get_parameter("map_info_file", info_path);

        int width = 0, height = 0;
        double res_x = 0.05, res_y = 0.05, origin_x = 0.0, origin_y = 0.0; 
        
        std::ifstream ifs_info(info_path);
        if (ifs_info.is_open() && (ifs_info >> width >> height >> res_x >> res_y)) {
            if (!(ifs_info >> origin_x >> origin_y)) { origin_x = 0.0; origin_y = 0.0; }
        } else { return; }

        std::vector<int> occupancy(width * height);
        std::ifstream ifs_occ(occ_path);
        for (int i = 0; i < width * height; ++i) ifs_occ >> occupancy[i];

        std::vector<double> esdf_raw(width * height);
        std::ifstream ifs_esdf(esdf_path);
        for (int i = 0; i < width * height; ++i) ifs_esdf >> esdf_raw[i];

        std::vector<double> esdf_dist(width * height);
        for (size_t i = 0; i < esdf_raw.size(); ++i) {
            esdf_dist[i] = (esdf_raw[i] > 0 ? 1.0 : -1.0) * std::sqrt(std::abs(esdf_raw[i])) * res_x;
        }

        if (m_Replanner.InitMapWithESDF(occupancy, esdf_dist, width, height, res_x, res_y, origin_x, origin_y)) {
            m_HasMap = true;
            MaybePlan();
        }
    }

    void MapCallback(const custom_msgs::msg::MapData::SharedPtr msg) {
        if (!msg->is_valid) return;
        if (m_Replanner.InitMapWithESDF(msg->occupancy_array, msg->esdf_array, msg->width, msg->height, 
                                        msg->res_x, msg->res_y, msg->origin_x, msg->origin_y)) {
            m_HasMap = true;
            MaybePlan();
        }
    }

    void ChassisInfoCallback(const custom_msgs::msg::ChassisInfo::SharedPtr msg) {
        if (!msg->is_valid) return;
        m_ChassisGoal << msg->target_x, msg->target_y;
        m_HasChassisGoal = true;

        if (msg->trigger_target) {
            m_TargetGoal = m_ChassisGoal;
            m_HasGoal = true;
            MaybePlan();
        }
        PublishGoalMarkers();
    }

    void RVizGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        m_RVizGoal << msg->pose.position.x, msg->pose.position.y;
        m_HasRVizGoal = true;
        m_TargetGoal = m_RVizGoal;
        m_HasGoal = true;
        MaybePlan();
        PublishGoalMarkers();
    }

    void SyncedStateMapCallback(const custom_msgs::msg::VehicleState::ConstSharedPtr& state_msg, 
                                const custom_msgs::msg::MapData::ConstSharedPtr& local_map_msg) {
        if (!state_msg->is_valid || !local_map_msg->is_valid) return;

        m_CurPose << state_msg->pos_x, state_msg->pos_y;
        m_CurVel << state_msg->vel_x, state_msg->vel_y;
        m_CurYaw = state_msg->yaw;
        m_HasPose = true;

        MaybePlan(true, local_map_msg, &state_msg->header.stamp);
    }

    void PublishResult(const builtin_interfaces::msg::Time& stamp) {
        if (m_Replanner.IsGetRes()) {
            Eigen::Vector2d res_pose = m_Replanner.GetResPose();
            Eigen::Vector2d res_vel = m_Replanner.GetResVec();

            double cos_y = std::cos(m_CurYaw);
            double sin_y = std::sin(m_CurYaw);

            double dx_map = res_pose.x() - m_CurPose.x();
            double dy_map = res_pose.y() - m_CurPose.y();
            
            double local_res_x = dx_map * cos_y + dy_map * sin_y;
            double local_res_y = -dx_map * sin_y + dy_map * cos_y;

            double local_vel_x = res_vel.x() * cos_y + res_vel.y() * sin_y;
            double local_vel_y = -res_vel.x() * sin_y + res_vel.y() * cos_y;

            double target_yaw = std::atan2(res_vel.y(), res_vel.x());
            double yaw_diff = target_yaw - m_CurYaw;
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
            
            double kp_yaw = 2.0; 
            double vyaw = kp_yaw * yaw_diff;

            custom_msgs::msg::Result out_msg;
            out_msg.header.stamp = stamp;
            out_msg.header.frame_id = "map"; 

            out_msg.res_pose_x = local_res_x;
            out_msg.res_pose_y = local_res_y;
            out_msg.res_vel_x = local_vel_x;
            out_msg.res_vel_y = local_vel_y;
            out_msg.yaw_diff = yaw_diff;
            out_msg.vyaw = vyaw;
            out_msg.is_valid = true; 
            
            m_ResultPub->publish(out_msg);
        }
    }

    void PublishTrajectory() {
        if (!m_IsInitialPlanned) return;
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map"; 

        const auto& traj = m_Replanner.GetTrajectory();
        Eigen::VectorXd times = traj.GetTimes();
        if (times.size() == 0) return;

        double start_time = m_HasPose ? m_Replanner.GetTrajStartTime(m_CurPose) : 0.0;
        double current_t = 0.0;

        for (int i = 0; i < times.size(); ++i) {
            double T = times(i);
            for (double t = 0; t <= T + 1e-4; t += 0.05) { 
                if (current_t + t >= start_time - 0.05) {
                    Eigen::Vector2d pos = traj.GetPosition(i, t);
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;
                    pose.pose.position.x = pos.x(); pose.pose.position.y = pos.y(); pose.pose.position.z = 0.1; 
                    pose.pose.orientation.w = 1.0;
                    path_msg.poses.push_back(pose);
                }
            }
            current_t += T;
        }
        m_PathPub->publish(path_msg);
    }

    replanner m_Replanner;
    bool m_HasMap, m_HasPose, m_HasGoal, m_IsInitialPlanned;
    Eigen::Vector2d m_TargetGoal, m_LastTargetGoal, m_CurPose, m_CurVel;
    double m_CurYaw; 

    Eigen::Vector2d m_RVizGoal, m_ChassisGoal;
    bool m_HasRVizGoal, m_HasChassisGoal;

    rclcpp::Publisher<custom_msgs::msg::Result>::SharedPtr m_ResultPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_PathPub;            
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_GoalMarkerPub;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_InitialPoseSub; 
    rclcpp::Subscription<custom_msgs::msg::MapData>::SharedPtr m_GlobalMapSub;
    rclcpp::Subscription<custom_msgs::msg::ChassisInfo>::SharedPtr m_ChassisInfoSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_GoalSub; 

    message_filters::Subscriber<custom_msgs::msg::VehicleState> m_StateSubSync;
    message_filters::Subscriber<custom_msgs::msg::MapData> m_LocalMapSubSync;
    
    typedef message_filters::sync_policies::ApproximateTime<custom_msgs::msg::VehicleState, custom_msgs::msg::MapData> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> m_Sync;
};

} // namespace planner_2d

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planner_2d::PlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
