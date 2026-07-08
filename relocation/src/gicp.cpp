#include "relocation/gicp.hpp"
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <omp.h>

namespace relocation {

    constexpr int gicp_min_valid_correspondences = 10;
    constexpr float gicp_min_step_norm = 1e-5f;
    constexpr float gicp_min_error_delta = 1e-5f;

    Eigen::Matrix3f GICP::Skew(const Eigen::Vector3f& v) {
        Eigen::Matrix3f mat;
        mat << 0.0f, -v.z(), v.y(),
               v.z(), 0.0f, -v.x(),
              -v.y(), v.x(), 0.0f;
        return mat;
    }

    Eigen::Matrix3f GICP::ExpSO3(const Eigen::Vector3f& w) {
        const float theta = w.norm();
        const Eigen::Matrix3f W = Skew(w);
        if (theta < 1e-6f) {
            return Eigen::Matrix3f::Identity() + W;
        }
        const float a = std::sin(theta) / theta;
        const float b = (1.0f - std::cos(theta)) / (theta * theta);
        return Eigen::Matrix3f::Identity() + a * W + b * W * W;
    }

    bool GICP::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_) {
        if (!sourcePC_ || !targetPC_ || sourcePC_->empty() || targetPC_->empty()) return false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_target(new pcl::PointCloud<pcl::PointXYZ>());

        if (m_VoxelLeafSize > 0.0f) {
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setLeafSize(m_VoxelLeafSize, m_VoxelLeafSize, m_VoxelLeafSize);
            filter.setInputCloud(sourcePC_);
            filter.filter(*processed_source);
            filter.setInputCloud(targetPC_);
            filter.filter(*processed_target);
        } else {
            processed_source = sourcePC_;
            processed_target = targetPC_;
        }

        if (processed_source->empty() || processed_target->empty()) return false;

        m_SourcePC.resize(processed_source->size());
        for (size_t i = 0; i < processed_source->size(); i++) {
            m_SourcePC[i] = processed_source->points[i].getVector3fMap();
        }

        m_TargetPC.resize(processed_target->size());
        for (size_t i = 0; i < processed_target->size(); i++) {
            m_TargetPC[i] = processed_target->points[i].getVector3fMap();
        }

        m_SourceCov = EstimateCovariances(m_SourcePC);
        m_TargetCov = EstimateCovariances(m_TargetPC);
        m_TargetKDTree.Build(m_TargetPC);

        m_RotatedMatrix = Eigen::Matrix3f::Identity();
        m_TransVector = Eigen::Vector3f::Zero();
        m_LastError = std::numeric_limits<float>::max();
        m_LastValidCount = 0;
        return true;
    }

    std::vector<Eigen::Matrix3f> GICP::EstimateCovariances(const std::vector<Eigen::Vector3f>& cloud) {
        std::vector<Eigen::Matrix3f> covariances(cloud.size(), Eigen::Matrix3f::Identity());
        if (cloud.empty()) return covariances;

        KDTree tree;
        tree.Build(cloud);
        const int k = std::max(3, std::min(m_CovarianceK, static_cast<int>(cloud.size())));

        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(cloud.size()); i++) {
            std::vector<int> indices;
            std::vector<double> dists;
            tree.SearchKNearest(cloud[i], k, indices, dists);
            covariances[i] = ComputeCovariance(cloud, indices);
        }
        return covariances;
    }

    Eigen::Matrix3f GICP::ComputeCovariance(const std::vector<Eigen::Vector3f>& cloud,
                                            const std::vector<int>& indices) const {
        if (indices.size() < 3) {
            return m_CovarianceRegularization * Eigen::Matrix3f::Identity();
        }

        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (int idx : indices) {
            mean += cloud[idx];
        }
        mean /= static_cast<float>(indices.size());

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (int idx : indices) {
            const Eigen::Vector3f diff = cloud[idx] - mean;
            cov += diff * diff.transpose();
        }
        cov /= static_cast<float>(indices.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        if (solver.info() != Eigen::Success) {
            return m_CovarianceRegularization * Eigen::Matrix3f::Identity();
        }

        Eigen::Vector3f evals = solver.eigenvalues();
        for (int i = 0; i < 3; i++) {
            evals[i] = std::max(evals[i], m_CovarianceRegularization);
        }
        return solver.eigenvectors() * evals.asDiagonal() * solver.eigenvectors().transpose();
    }

    bool GICP::Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_) {
        const int N = static_cast<int>(m_SourcePC.size());
        if (N == 0 || m_TargetPC.empty()) return false;

        float prev_objective_error = std::numeric_limits<float>::max();
        float final_error = std::numeric_limits<float>::max();
        int final_valid_count = 0;
        bool has_valid_solution = false;

        std::vector<int> nn_indices(N, -1);
        std::vector<float> dists(N, 0.0f);
        std::vector<bool> valid_flag(N, false);

        for (int iter = 0; iter < m_MaxIterations; iter++) {
            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                int nn_idx = m_TargetKDTree.GetNearestIdx(m_SourcePC[i]);
                nn_indices[i] = nn_idx;
                valid_flag[i] = false;

                if (nn_idx >= 0) {
                    float dist = (m_SourcePC[i] - m_TargetPC[nn_idx]).norm();
                    dists[i] = dist;
                    if (dist <= m_MaxCorrespondenceDistance) {
                        valid_flag[i] = true;
                    }
                }
            }

            Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();
            float current_objective_error = 0.0f;
            float current_euclidean_error = 0.0f;
            int valid_count = 0;

            for (int i = 0; i < N; i++) {
                if (!valid_flag[i]) continue;

                const int nn_idx = nn_indices[i];
                const Eigen::Vector3f residual = m_SourcePC[i] - m_TargetPC[nn_idx];
                Eigen::Matrix3f cov = m_TargetCov[nn_idx] +
                    m_RotatedMatrix * m_SourceCov[i] * m_RotatedMatrix.transpose();
                cov += m_CovarianceRegularization * Eigen::Matrix3f::Identity();
                Eigen::Matrix3f info = cov.inverse();

                Eigen::Matrix<float, 3, 6> J;
                J.block<3, 3>(0, 0) = -Skew(m_SourcePC[i]);
                J.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();

                H += J.transpose() * info * J;
                b += J.transpose() * info * residual;
                const float mahalanobis_error = residual.dot(info * residual);
                current_objective_error += std::sqrt(std::max(0.0f, mahalanobis_error));
                current_euclidean_error += residual.norm();
                valid_count++;
            }

            if (valid_count < gicp_min_valid_correspondences) {
                std::cerr << "[GICP] Too few correspondences: " << valid_count << std::endl;
                break;
            }

            Eigen::Matrix<float, 6, 1> dx = -H.ldlt().solve(b);
            if (!dx.allFinite()) {
                std::cerr << "[GICP] Linear solve failed" << std::endl;
                break;
            }

            const Eigen::Vector3f delta_rot = dx.head<3>();
            const Eigen::Vector3f delta_trans = dx.tail<3>();
            const Eigen::Matrix3f dR = ExpSO3(delta_rot);

            m_RotatedMatrix = dR * m_RotatedMatrix;
            m_TransVector = dR * m_TransVector + delta_trans;

            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                m_SourcePC[i] = dR * m_SourcePC[i] + delta_trans;
            }

            current_objective_error /= static_cast<float>(valid_count);
            current_euclidean_error /= static_cast<float>(valid_count);
            final_error = current_euclidean_error;
            final_valid_count = valid_count;
            has_valid_solution = true;

            if (dx.norm() < gicp_min_step_norm ||
                std::abs(prev_objective_error - current_objective_error) < gicp_min_error_delta) {
                break;
            }
            prev_objective_error = current_objective_error;
        }

        if (!has_valid_solution) {
            std::cerr << "[GICP] Solve failed, no valid iteration." << std::endl;
            return false;
        }

        if (final_error > m_MaxCorrespondenceDistance) {
            std::cerr << "[GICP] Solve rejected, error: " << final_error
                      << ", valid correspondences: " << final_valid_count << std::endl;
            return false;
        }

        m_LastError = final_error;
        m_LastValidCount = final_valid_count;
        R_result_ = m_RotatedMatrix.cast<double>();
        T_result_ = m_TransVector.cast<double>();
        return true;
    }

}
