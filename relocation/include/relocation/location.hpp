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
#include "relocation/icp.hpp"

namespace relocation {

    constexpr int loc_rough_k_candidates = 20;
    constexpr float loc_prior_radius_limit = 15.0f;
    constexpr float loc_prior_penalty_scale = 0.2f;
    
    constexpr float loc_icp_crop_z_min = 0.2f;
    constexpr float loc_icp_crop_z_max = 2.5f;
    constexpr float loc_icp_crop_xy = 20.0f;

    struct HistNode {
        float x, y, yaw;
        std::vector<uint8_t> binary_vec;
        iris::FeatureDesc desc;
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
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                            Eigen::Matrix3d& R, Eigen::Vector3d& T);
        bool SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud);

    private:
        bool LoadDatabase(const std::string& filename);

        pcl::PointCloud<pcl::PointXYZ>::Ptr m_global_map;
        std::vector<HistNode> m_history_db;
        std::unique_ptr<ann_kdtree> m_iris_tree;

        bool m_has_pre_pose = false;
        double m_pre_x = 0.0;
        double m_pre_y = 0.0;
        double m_pre_yaw = 0.0;
    };

}