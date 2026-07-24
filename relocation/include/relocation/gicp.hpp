#pragma once
#include "kd_tree.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace relocation {

    constexpr int gicp_max_iterations = 15;
    constexpr float gicp_max_corr_dist = 2.0f;
    constexpr float gicp_voxel_leaf_size = 0.2f;
    constexpr int gicp_covariance_k = 20;
    constexpr int gicp_correspondence_k = 8;
    constexpr float gicp_covariance_regularization = 1e-3f;
    constexpr float gicp_plane_eigenvalue = 1.0f;
    constexpr float gicp_normal_eigenvalue = 1e-3f;
    constexpr float gicp_hessian_min_eigenvalue = 1e-6f;
    constexpr float gicp_hessian_max_condition = 1e6f;
    constexpr float gicp_hessian_damping = 1e-3f;
    constexpr float gicp_xicp_unobservable_eigen_ratio = 1e-6f;
    constexpr float gicp_xicp_partial_eigen_ratio = 1e-4f;
    constexpr float gicp_xicp_partial_min_scale = 0.2f;
    constexpr bool gicp_use_aa = true;
    constexpr int gicp_aa_history_size = 5;
    constexpr int gicp_aa_start_iteration = 2;
    constexpr float gicp_aa_max_translation_step = 0.5f;
    constexpr float gicp_aa_max_rotation_step_deg = 5.0f;
    constexpr float gicp_aa_error_reject_ratio = 1.0f;
    constexpr bool gicp_aa_verbose = false;
    constexpr int gicp_max_source_points = 7000;
    constexpr int gicp_max_parallel_threads = 24;

    struct GICPPreparedSource {
        using Ptr = std::shared_ptr<GICPPreparedSource>;
        using ConstPtr = std::shared_ptr<const GICPPreparedSource>;

        std::vector<Eigen::Vector3f> points;
        std::vector<Eigen::Matrix3f> covariances;
        float effective_voxel_leaf_size = 0.0f;
    };

    struct GICPPreparedTarget {
        using Ptr = std::shared_ptr<GICPPreparedTarget>;
        using ConstPtr = std::shared_ptr<const GICPPreparedTarget>;

        std::vector<Eigen::Vector3f> points;
        std::vector<Eigen::Matrix3f> covariances;
        KDTree kdtree;
    };

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
        void SetUseAA(bool use_aa) { m_UseAA = use_aa; }

        bool Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_);
        bool InitWithTargetCovariances(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
                                       const std::vector<Eigen::Matrix3f>& target_covariances);
        GICPPreparedSource::Ptr PrepareSource(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_);
        GICPPreparedTarget::Ptr PrepareTargetWithCovariances(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
            const std::vector<Eigen::Matrix3f>& target_covariances);
        bool InitPrepared(const GICPPreparedSource::ConstPtr& source,
                          const GICPPreparedTarget::ConstPtr& target,
                          const Eigen::Matrix3f& initial_R = Eigen::Matrix3f::Identity(),
                          const Eigen::Vector3f& initial_T = Eigen::Vector3f::Zero());
        bool Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_);
        float GetLastError() const { return m_LastError; }
        int GetLastValidCount() const { return m_LastValidCount; }
        int GetSourcePointCount() const {
            return m_PreparedSource ? static_cast<int>(m_PreparedSource->points.size()) : 0;
        }
        int GetTargetPointCount() const {
            return m_PreparedTarget ? static_cast<int>(m_PreparedTarget->points.size()) : 0;
        }
        int GetLastIterations() const { return m_LastIterations; }
        double GetLastInitTimeMs() const { return m_LastInitTimeMs; }
        double GetLastSolveTimeMs() const { return m_LastSolveTimeMs; }
        float GetLastEffectiveVoxelLeafSize() const { return m_LastEffectiveVoxelLeafSize; }
        bool WasLastXicpTriggered() const { return m_LastXicpTriggered; }
        std::vector<Eigen::Matrix3f> EstimateCovariances(const std::vector<Eigen::Vector3f>& cloud);

    private:
        static Eigen::Matrix3f Skew(const Eigen::Vector3f& v);
        static Eigen::Matrix3f ExpSO3(const Eigen::Vector3f& w);
        static Eigen::Vector3f LogSO3(const Eigen::Matrix3f& R);
        static Eigen::Matrix<float, 6, 1> PoseToDelta(const Eigen::Matrix3f& R, const Eigen::Vector3f& T);
        static void DeltaToPose(const Eigen::Matrix<float, 6, 1>& delta,
                                Eigen::Matrix3f& R,
                                Eigen::Vector3f& T);
        static bool IsStepWithinAALimit(const Eigen::Matrix<float, 6, 1>& step);
        static Eigen::Matrix<float, 6, 1> ApplyXicpConstraint(
            const Eigen::Matrix<float, 6, 1>& dx,
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>>& h_solver,
            float max_eval,
            bool hessian_degenerate);

        Eigen::Matrix3f ComputeCovariance(const std::vector<Eigen::Vector3f>& cloud,
                                          const std::vector<int>& indices) const;
        void UpdateTransformedSource(const Eigen::Matrix3f& R, const Eigen::Vector3f& T);
        float EvaluatePoseErrorWithFixedCorrespondences(
            const Eigen::Matrix3f& R,
            const Eigen::Vector3f& T,
            const std::vector<int>& nn_indices,
            const std::vector<std::uint8_t>& valid_flag,
            int& valid_count) const;

        int m_MaxIterations;
        float m_MaxCorrespondenceDistance;
        float m_VoxelLeafSize;
        int m_CovarianceK;
        int m_CorrespondenceK;
        float m_CovarianceRegularization;
        bool m_UseAA = gicp_use_aa;

        std::vector<Eigen::Vector3f> m_SourcePC;
        GICPPreparedSource::ConstPtr m_PreparedSource;
        GICPPreparedTarget::ConstPtr m_PreparedTarget;

        Eigen::Matrix3f m_RotatedMatrix;
        Eigen::Vector3f m_TransVector;
        float m_LastError = std::numeric_limits<float>::max();
        int m_LastValidCount = 0;
        int m_LastIterations = 0;
        double m_LastInitTimeMs = 0.0;
        double m_LastSolveTimeMs = 0.0;
        float m_LastEffectiveVoxelLeafSize = 0.0f;
        bool m_LastXicpTriggered = false;
    };

}
