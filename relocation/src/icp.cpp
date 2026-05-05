#include "relocation/icp.hpp"
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <limits>
#include <omp.h> 

namespace relocation {

    bool P2PointICP_SVD::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_, 
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

        m_TargetKDTree.Build(m_TargetPC);
        m_RotatedMatrix = Eigen::Matrix3f::Identity();
        m_TransVector = Eigen::Vector3f::Zero();
        return true;
    }

    bool P2PointICP_SVD::Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_) {
        const int N = m_SourcePC.size();
        Eigen::Matrix3Xf S_matched(3, N);
        Eigen::Matrix3Xf T_matched(3, N);
        float prev_error = std::numeric_limits<float>::max();
        std::vector<int> nn_indices(N, -1);
        std::vector<float> dists(N, 0.0f);
        std::vector<bool> valid_flag(N, false);

        for (int iter = 0; iter < m_MaxIterations; iter++) {
            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                Eigen::Vector3f query_pt = m_SourcePC[i];
                int nn_idx = m_TargetKDTree.GetNearestIdx(query_pt);
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

            int valid_count = 0;
            float current_error = 0.0f;
            for (int i = 0; i < N; i++) {
                if (valid_flag[i]) {
                    S_matched.col(valid_count) = m_SourcePC[i];
                    T_matched.col(valid_count) = m_TargetPC[nn_indices[i]];
                    current_error += dists[i];
                    valid_count++;
                }
            }

            if (valid_count < 3) break;
            auto S_valid = S_matched.leftCols(valid_count);
            auto T_valid = T_matched.leftCols(valid_count);
            Eigen::Vector3f mean_S = S_valid.rowwise().mean();
            Eigen::Vector3f mean_T = T_valid.rowwise().mean();
            S_valid.colwise() -= mean_S;
            T_valid.colwise() -= mean_T;

            Eigen::Matrix3f H = S_valid * T_valid.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3f U = svd.matrixU();
            Eigen::Matrix3f V = svd.matrixV();
            
            Eigen::Matrix3f R = V * U.transpose();
            if (R.determinant() < 0) {
                V.col(2) *= -1;
                R = V * U.transpose();
            }
            Eigen::Vector3f T = mean_T - R * mean_S;

            m_RotatedMatrix = R * m_RotatedMatrix;
            m_TransVector = R * m_TransVector + T;

            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                m_SourcePC[i] = R * m_SourcePC[i] + T;
            }

            current_error /= valid_count;
            if (std::abs(prev_error - current_error) < 1e-5) break;
            prev_error = current_error;
        }

        R_result_ = m_RotatedMatrix.cast<double>();
        T_result_ = m_TransVector.cast<double>();
        return true;
    }

    bool P2PlaneICP_SVD::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_, 
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
                              const std::vector<Eigen::Vector3f>& targetNormals_) {
        if (!sourcePC_ || !targetPC_ || targetPC_->size() != targetNormals_.size()) return false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_source(new pcl::PointCloud<pcl::PointXYZ>());
        if (m_VoxelLeafSize > 0.0f) {
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setLeafSize(m_VoxelLeafSize, m_VoxelLeafSize, m_VoxelLeafSize);
            filter.setInputCloud(sourcePC_);
            filter.filter(*processed_source);
        } else {
            processed_source = sourcePC_;
        }
        m_SourcePC.resize(processed_source->size());
        for (size_t i = 0; i < processed_source->size(); i++) {
            m_SourcePC[i] = processed_source->points[i].getVector3fMap();
        }
        m_TargetPC.resize(targetPC_->size());
        m_TargetNormals = targetNormals_;
        for (size_t i = 0; i < targetPC_->size(); i++) {
            m_TargetPC[i] = targetPC_->points[i].getVector3fMap();
        }
        m_TargetKDTree.Build(m_TargetPC);
        m_RotatedMatrix = Eigen::Matrix3f::Identity();
        m_TransVector = Eigen::Vector3f::Zero();
        return true;
    }

    bool P2PlaneICP_SVD::Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_) {
        const int N = m_SourcePC.size();
        Eigen::Matrix3Xf S_matched(3, N);
        Eigen::Matrix3Xf T_proj_matched(3, N);
        float prev_error = std::numeric_limits<float>::max();
        std::vector<int> nn_indices(N, -1);
        std::vector<float> dists_to_plane(N, 0.0f);
        std::vector<Eigen::Vector3f> projected_points(N);
        std::vector<bool> valid_flag(N, false);

        for (int iter = 0; iter < m_MaxIterations; iter++) {
            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                Eigen::Vector3f query_pt = m_SourcePC[i];
                int nn_idx = m_TargetKDTree.GetNearestIdx(query_pt);
                nn_indices[i] = nn_idx;
                valid_flag[i] = false;

                if (nn_idx >= 0) {
                    Eigen::Vector3f target_p = m_TargetPC[nn_idx];
                    Eigen::Vector3f target_n = m_TargetNormals[nn_idx];
                    float dist_to_point = (m_SourcePC[i] - target_p).norm();
                    if (dist_to_point <= m_MaxCorrespondenceDistance) {
                        float dist_plane = (m_SourcePC[i] - target_p).dot(target_n);
                        dists_to_plane[i] = std::abs(dist_plane);
                        projected_points[i] = m_SourcePC[i] - dist_plane * target_n;
                        valid_flag[i] = true;
                    }
                }
            }

            int valid_count = 0;
            float current_error = 0.0f;
            for (int i = 0; i < N; i++) {
                if (valid_flag[i]) {
                    S_matched.col(valid_count) = m_SourcePC[i];
                    T_proj_matched.col(valid_count) = projected_points[i];
                    current_error += dists_to_plane[i];
                    valid_count++;
                }
            }

            if (valid_count < 3) break;
            auto S_valid = S_matched.leftCols(valid_count);
            auto T_valid = T_proj_matched.leftCols(valid_count);
            Eigen::Vector3f mean_S = S_valid.rowwise().mean();
            Eigen::Vector3f mean_T = T_valid.rowwise().mean();
            S_valid.colwise() -= mean_S;
            T_valid.colwise() -= mean_T;

            Eigen::Matrix3f H = S_valid * T_valid.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3f U = svd.matrixU();
            Eigen::Matrix3f V = svd.matrixV();
            
            Eigen::Matrix3f R = V * U.transpose();
            if (R.determinant() < 0) {
                V.col(2) *= -1;
                R = V * U.transpose();
            }
            Eigen::Vector3f T = mean_T - R * mean_S;

            m_RotatedMatrix = R * m_RotatedMatrix;
            m_TransVector = R * m_TransVector + T;

            #pragma omp parallel for
            for (int i = 0; i < N; i++) {
                m_SourcePC[i] = R * m_SourcePC[i] + T;
            }

            current_error /= valid_count;
            if (std::abs(prev_error - current_error) < 1e-5) break;
            prev_error = current_error;
        }

        R_result_ = m_RotatedMatrix.cast<double>();
        T_result_ = m_TransVector.cast<double>();
        return true;
    }
}