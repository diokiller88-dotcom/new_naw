#pragma once
#include "kd_tree.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace relocation {

    constexpr int icp_p2p_max_iterations = 30;
    constexpr float icp_p2p_max_corr_dist = 2.0f;
    constexpr float icp_p2p_voxel_leaf_size = 0.2f;

    constexpr int icp_p2plane_max_iterations = 10;
    constexpr float icp_p2plane_max_corr_dist = 1.0f;
    constexpr float icp_p2plane_voxel_leaf_size = 0.1f;

    class P2PointICP_SVD {
    public:
        P2PointICP_SVD() : m_MaxIterations(icp_p2p_max_iterations), m_MaxCorrespondenceDistance(icp_p2p_max_corr_dist), m_VoxelLeafSize(icp_p2p_voxel_leaf_size) {}
        void SetMaxIterations(int iterations) { m_MaxIterations = iterations; }
        void SetMaxCorrespondenceDistance(float dist) { m_MaxCorrespondenceDistance = dist; }
        void SetVoxelLeafSize(float size) { m_VoxelLeafSize = size; }

        bool Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_, 
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_);
        bool Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_);
        float GetLastError() const { return m_LastError; }
        int GetLastValidCount() const { return m_LastValidCount; }

    private:
        int m_MaxIterations;
        float m_MaxCorrespondenceDistance; 
        float m_VoxelLeafSize;
        
        std::vector<Eigen::Vector3f> m_SourcePC;
        std::vector<Eigen::Vector3f> m_TargetPC;
        KDTree m_TargetKDTree; 
        
        Eigen::Matrix3f m_RotatedMatrix;
        Eigen::Vector3f m_TransVector;
        float m_LastError = std::numeric_limits<float>::max();
        int m_LastValidCount = 0;
    };

    class P2PlaneICP_SVD {
    public:
        P2PlaneICP_SVD() : m_MaxIterations(icp_p2plane_max_iterations), m_MaxCorrespondenceDistance(icp_p2plane_max_corr_dist), m_VoxelLeafSize(icp_p2plane_voxel_leaf_size) {}
        void SetMaxIterations(int iterations) { m_MaxIterations = iterations; }
        void SetMaxCorrespondenceDistance(float dist) { m_MaxCorrespondenceDistance = dist; }
        void SetVoxelLeafSize(float size) { m_VoxelLeafSize = size; }

        bool Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_, 
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
                  const std::vector<Eigen::Vector3f>& targetNormals_);
        bool Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_);
        float GetLastError() const { return m_LastError; }
        int GetLastValidCount() const { return m_LastValidCount; }

    private:
        int m_MaxIterations;
        float m_MaxCorrespondenceDistance;
        float m_VoxelLeafSize;

        std::vector<Eigen::Vector3f> m_SourcePC;
        std::vector<Eigen::Vector3f> m_TargetPC;
        std::vector<Eigen::Vector3f> m_TargetNormals;
        KDTree m_TargetKDTree; 
        
        Eigen::Matrix3f m_RotatedMatrix;
        Eigen::Vector3f m_TransVector;
        float m_LastError = std::numeric_limits<float>::max();
        int m_LastValidCount = 0;
    };

}
