#pragma once
#include "kd_tree.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace relocation {

    constexpr int gicp_max_iterations = 20;
    constexpr float gicp_max_corr_dist = 2.0f;
    constexpr float gicp_voxel_leaf_size = 0.05f;
    constexpr int gicp_covariance_k = 20;
    constexpr int gicp_correspondence_k = 8;
    constexpr float gicp_covariance_regularization = 1e-3f;
    constexpr float gicp_plane_eigenvalue = 1.0f;
    constexpr float gicp_normal_eigenvalue = 1e-3f;
    constexpr float gicp_hessian_min_eigenvalue = 1e-6f;
    constexpr float gicp_hessian_max_condition = 1e6f;
    constexpr float gicp_hessian_damping = 1e-3f;

    class GICP {
    public:
        GICP()
            : m_MaxIterations(gicp_max_iterations),
              m_MaxCorrespondenceDistance(gicp_max_corr_dist),
              m_VoxelLeafSize(gicp_voxel_leaf_size),
              m_CovarianceK(gicp_covariance_k),
              m_CorrespondenceK(gicp_correspondence_k),
              m_CovarianceRegularization(gicp_covariance_regularization) {}

        void SetMaxIterations(int iterations) { m_MaxIterations = iterations; }
        void SetMaxCorrespondenceDistance(float dist) { m_MaxCorrespondenceDistance = dist; }
        void SetVoxelLeafSize(float size) { m_VoxelLeafSize = size; }
        void SetCovarianceK(int k) { m_CovarianceK = k; }
        void SetCorrespondenceK(int k) { m_CorrespondenceK = k; }
        void SetCovarianceRegularization(float reg) { m_CovarianceRegularization = reg; }

        bool Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_);
        bool InitWithTargetCovariances(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
                                       const std::vector<Eigen::Matrix3f>& target_covariances);
        bool Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_);
        float GetLastError() const { return m_LastError; }
        int GetLastValidCount() const { return m_LastValidCount; }
        std::vector<Eigen::Matrix3f> EstimateCovariances(const std::vector<Eigen::Vector3f>& cloud);

    private:
        static Eigen::Matrix3f Skew(const Eigen::Vector3f& v);
        static Eigen::Matrix3f ExpSO3(const Eigen::Vector3f& w);

        Eigen::Matrix3f ComputeCovariance(const std::vector<Eigen::Vector3f>& cloud,
                                          const std::vector<int>& indices) const;

        int m_MaxIterations;
        float m_MaxCorrespondenceDistance;
        float m_VoxelLeafSize;
        int m_CovarianceK;
        int m_CorrespondenceK;
        float m_CovarianceRegularization;

        std::vector<Eigen::Vector3f> m_SourcePC;
        std::vector<Eigen::Vector3f> m_TargetPC;
        std::vector<Eigen::Matrix3f> m_SourceCov;
        std::vector<Eigen::Matrix3f> m_TargetCov;
        KDTree m_TargetKDTree;

        Eigen::Matrix3f m_RotatedMatrix;
        Eigen::Vector3f m_TransVector;
        float m_LastError = std::numeric_limits<float>::max();
        int m_LastValidCount = 0;
    };

}
