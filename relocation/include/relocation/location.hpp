#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "relocation/iris.hpp"
#include "relocation/ann_kdtree.hpp"
#include "relocation/gicp.hpp"
#include "relocation/scan_context.hpp"

namespace relocation {

    constexpr int loc_rough_k_candidates = 80;
    constexpr int loc_sc_rough_k_candidates = 20;
    constexpr float loc_prior_radius_limit = 15.0f;
    constexpr float loc_prior_penalty_scale = 0.2f;
    constexpr float loc_prior_yaw_limit_deg = 75.0f;
    constexpr float loc_prior_yaw_penalty_scale = 0.5f;
    constexpr int loc_max_rough_pose_candidates = 8;
    constexpr double loc_fusion_base_score = 0.05;
    constexpr double loc_fusion_iris_weight = 0.60;
    constexpr double loc_fusion_sc_weight = 0.35;
    constexpr double loc_fusion_yaw_weight = 0.05;
    constexpr double loc_fusion_sc_rejection_cost =
        loc_fusion_sc_weight + loc_fusion_yaw_weight;
    constexpr double loc_fusion_yaw_mean_limit_deg = 10.0;
    constexpr double loc_fusion_yaw_consistency_limit_deg = 30.0;
    
    constexpr float loc_icp_crop_z_min = 0.2f;
    constexpr float loc_icp_crop_z_max = 2.5f;

    struct HistNode {
        float x, y, yaw;
        std::vector<uint8_t> binary_vec;
        iris::FeatureDesc desc;
    };

    struct RoughPoseCandidate {
        double x = 0.0;
        double y = 0.0;
        double yaw_deg = 0.0;
        double rough_score = 0.0;
        double iris_score = 1.0;
        double sc_score = 1.0;
        double iris_yaw_deg = 0.0;
        double sc_yaw_deg = 0.0;
        double descriptor_yaw_diff_deg = 180.0;
        double sc_lateral_shift = 0.0;
        double dist_to_prior = 0.0;
        double yaw_diff_deg = 0.0;
        bool iris_retrieved = false;
        bool sc_retrieved = false;
        bool sc_available = false;
        bool sc_matched = false;
        bool fusion_active = false;
        ScanContextVariant sc_variant = ScanContextVariant::Original;
        int hist_index = -1;
    };

    class location {
    public:
        location() = default;
        ~location() = default;

        bool Init(const std::string& db_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& global_map);
        void GetPrePose(const double& x, const double& y, const double& yaw);
        bool SetRoughPose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                          double& out_x, double& out_y, double& out_yaw);
        bool SetRoughPoseWithPrePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                     const double& pre_x, const double& pre_y, const double& pre_yaw, 
                                     double& out_x, double& out_y, double& out_yaw);
        bool GetRoughPoseCandidatesWithPrePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                                               const double& pre_x, const double& pre_y, const double& pre_yaw,
                                               std::vector<RoughPoseCandidate>& candidates);
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                            Eigen::Matrix3d& R, Eigen::Vector3d& T);
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                            Eigen::Matrix3d& R, Eigen::Vector3d& T,
                            double& out_match_error, int& out_valid_count);
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                            Eigen::Matrix3d& R, Eigen::Vector3d& T,
                            double& out_match_error, int& out_valid_count,
                            int max_iterations, float voxel_leaf_size,
                            int correspondence_k = gicp_correspondence_k);
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud);
        int GetLastSourcePointCount() const { return m_last_source_point_count; }
        int GetLastTargetPointCount() const { return m_last_target_point_count; }
        int GetLastGicpIterations() const { return m_last_gicp_iterations; }
        double GetLastGicpCropTimeMs() const { return m_last_gicp_crop_time_ms; }
        double GetLastGicpInitTimeMs() const { return m_last_gicp_init_time_ms; }
        double GetLastGicpSolveTimeMs() const { return m_last_gicp_solve_time_ms; }
        float GetLastGicpEffectiveVoxelLeafSize() const {
            return m_last_gicp_effective_voxel_leaf_size;
        }
        bool WasLastXicpTriggered() const { return m_last_xicp_triggered; }
        bool IsScanContextEnabled() const { return m_scan_context != nullptr; }

    private:
        struct GicpSourceCacheEntry {
            float voxel_leaf_size = 0.0f;
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr input;
            GICPPreparedSource::ConstPtr prepared;
        };

        bool LoadDatabase(const std::string& filename);
        GICPPreparedSource::ConstPtr GetPreparedGicpSource(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
            float voxel_leaf_size);

        pcl::PointCloud<pcl::PointXYZ>::Ptr m_global_map;
        GICPPreparedTarget::ConstPtr m_gicp_target;
        std::vector<GicpSourceCacheEntry> m_gicp_source_cache;
        std::vector<HistNode> m_history_db;
        std::unique_ptr<ann_kdtree> m_iris_tree;
        std::unique_ptr<ScanContextPlusPlus> m_scan_context;

        bool m_has_pre_pose = false;
        double m_pre_x = 0.0;
        double m_pre_y = 0.0;
        double m_pre_yaw = 0.0;
        int m_last_source_point_count = 0;
        int m_last_target_point_count = 0;
        int m_last_gicp_iterations = 0;
        double m_last_gicp_crop_time_ms = 0.0;
        double m_last_gicp_init_time_ms = 0.0;
        double m_last_gicp_solve_time_ms = 0.0;
        float m_last_gicp_effective_voxel_leaf_size = 0.0f;
        bool m_last_xicp_triggered = false;
    };

}
