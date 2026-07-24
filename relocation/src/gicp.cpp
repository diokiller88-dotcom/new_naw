#include "relocation/gicp.hpp"
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <omp.h>

namespace relocation {

    namespace {
        float FilterSourceCloud(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
            float requested_leaf_size,
            pcl::PointCloud<pcl::PointXYZ>& output)
        {
            if (requested_leaf_size <= 0.0f) {
                output = *input;
                return 0.0f;
            }

            float effective_leaf_size = requested_leaf_size;
            for (int attempt = 0; attempt < 5; ++attempt) {
                pcl::VoxelGrid<pcl::PointXYZ> filter;
                filter.setLeafSize(
                    effective_leaf_size, effective_leaf_size, effective_leaf_size);
                filter.setInputCloud(input);
                filter.filter(output);
                if (output.size() <= static_cast<std::size_t>(gicp_max_source_points)) {
                    return effective_leaf_size;
                }

                const float point_ratio = static_cast<float>(output.size()) /
                                          static_cast<float>(gicp_max_source_points);
                effective_leaf_size *= std::max(1.10f, std::cbrt(point_ratio));
            }

            if (output.size() > static_cast<std::size_t>(gicp_max_source_points)) {
                pcl::PointCloud<pcl::PointXYZ> limited;
                limited.reserve(gicp_max_source_points);
                const double stride = static_cast<double>(output.size()) /
                                      static_cast<double>(gicp_max_source_points);
                for (int i = 0; i < gicp_max_source_points; ++i) {
                    const std::size_t index = std::min(
                        static_cast<std::size_t>(i * stride), output.size() - 1);
                    limited.push_back(output[index]);
                }
                output.swap(limited);
            }
            return effective_leaf_size;
        }
    }

    constexpr int gicp_min_valid_correspondences = 10;
    constexpr float gicp_min_step_norm = 1e-5f;
    constexpr float gicp_min_error_delta = 1e-5f;

    class AndersonAcceleration6 {
    public:
        using Vec6 = Eigen::Matrix<float, 6, 1>;

        explicit AndersonAcceleration6(int history_size)
            : history_size_(std::max(1, history_size)),
              prev_dF_(6, history_size_),
              prev_dG_(6, history_size_),
              scales_(history_size_) {
            prev_dF_.setZero();
            prev_dG_.setZero();
            scales_.setZero();
        }

        void Init(const Vec6& u0) {
            current_u_ = u0;
            prev_dF_.setZero();
            prev_dG_.setZero();
            scales_.setZero();
            iter_ = 0;
            col_idx_ = 0;
        }

        void SetCurrent(const Vec6& u) {
            current_u_ = u;
        }

        Vec6 Compute(const Vec6& g) {
            constexpr float eps = 1e-8f;
            const Vec6 current_F = g - current_u_;

            if (iter_ == 0) {
                prev_dF_.col(0) = -current_F;
                prev_dG_.col(0) = -g;
                current_u_ = g;
                iter_++;
                return current_u_;
            }

            prev_dF_.col(col_idx_) += current_F;
            prev_dG_.col(col_idx_) += g;

            const float scale = std::max(eps, static_cast<float>(prev_dF_.col(col_idx_).norm()));
            scales_(col_idx_) = scale;
            prev_dF_.col(col_idx_) /= scale;

            const int m_k = std::min(history_size_, iter_);
            Eigen::VectorXf theta = Eigen::VectorXf::Zero(m_k);
            if (m_k == 1) {
                const float dF_sqrnorm = static_cast<float>(prev_dF_.col(0).squaredNorm());
                if (dF_sqrnorm > eps) {
                    theta(0) = prev_dF_.col(0).dot(current_F) / dF_sqrnorm;
                }
            } else {
                const Eigen::MatrixXf dF = prev_dF_.leftCols(m_k);
                const Eigen::MatrixXf M = dF.transpose() * dF;
                theta = M.completeOrthogonalDecomposition().solve(dF.transpose() * current_F);
            }

            Eigen::VectorXf coeff = theta;
            for (int i = 0; i < m_k; ++i) {
                coeff(i) /= std::max(eps, scales_(i));
            }
            current_u_ = g - prev_dG_.leftCols(m_k) * coeff;

            col_idx_ = (col_idx_ + 1) % history_size_;
            prev_dF_.col(col_idx_) = -current_F;
            prev_dG_.col(col_idx_) = -g;
            iter_++;
            return current_u_;
        }

    private:
        int history_size_ = 1;
        int iter_ = 0;
        int col_idx_ = 0;
        Vec6 current_u_ = Vec6::Zero();
        Eigen::MatrixXf prev_dF_;
        Eigen::MatrixXf prev_dG_;
        Eigen::VectorXf scales_;
    };

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

    Eigen::Vector3f GICP::LogSO3(const Eigen::Matrix3f& R) {
        const float cos_theta = std::max(-1.0f, std::min(1.0f, (R.trace() - 1.0f) * 0.5f));
        const float theta = std::acos(cos_theta);
        Eigen::Vector3f w;
        w << R(2, 1) - R(1, 2),
             R(0, 2) - R(2, 0),
             R(1, 0) - R(0, 1);
        if (theta < 1e-6f) {
            return 0.5f * w;
        }
        const float sin_theta = std::sin(theta);
        if (std::abs(sin_theta) < 1e-6f) {
            return 0.5f * theta * w.normalized();
        }
        return theta / (2.0f * sin_theta) * w;
    }

    Eigen::Matrix<float, 6, 1> GICP::PoseToDelta(const Eigen::Matrix3f& R, const Eigen::Vector3f& T) {
        Eigen::Matrix<float, 6, 1> delta;
        delta.head<3>() = LogSO3(R);
        delta.tail<3>() = T;
        return delta;
    }

    void GICP::DeltaToPose(const Eigen::Matrix<float, 6, 1>& delta,
                           Eigen::Matrix3f& R,
                           Eigen::Vector3f& T) {
        R = ExpSO3(delta.head<3>());
        T = delta.tail<3>();
    }

    bool GICP::IsStepWithinAALimit(const Eigen::Matrix<float, 6, 1>& step) {
        const float rot_deg = step.head<3>().norm() * 180.0f / static_cast<float>(M_PI);
        const float trans = step.tail<3>().norm();
        return trans <= gicp_aa_max_translation_step && rot_deg <= gicp_aa_max_rotation_step_deg;
    }

    Eigen::Matrix<float, 6, 1> GICP::ApplyXicpConstraint(
        const Eigen::Matrix<float, 6, 1>& dx,
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>>& h_solver,
        float max_eval,
        bool hessian_degenerate) {
        if (!hessian_degenerate || max_eval <= gicp_hessian_min_eigenvalue || !dx.allFinite()) {
            return dx;
        }

        const Eigen::Matrix<float, 6, 6> eig_vecs = h_solver.eigenvectors();
        const Eigen::Matrix<float, 6, 1> eig_vals = h_solver.eigenvalues();
        Eigen::Matrix<float, 6, 1> dx_eigen = eig_vecs.transpose() * dx;
        Eigen::Matrix<float, 6, 1> scales = Eigen::Matrix<float, 6, 1>::Ones();
        bool constrained = false;

        for (int i = 0; i < 6; ++i) {
            const float eval = std::max(0.0f, eig_vals[i]);
            const float ratio = eval / max_eval;
            if (eval < gicp_hessian_min_eigenvalue || ratio < gicp_xicp_unobservable_eigen_ratio) {
                scales[i] = 0.0f;
            } else if (ratio < gicp_xicp_partial_eigen_ratio) {
                const float linear_scale = ratio / gicp_xicp_partial_eigen_ratio;
                scales[i] = std::max(gicp_xicp_partial_min_scale, std::min(1.0f, linear_scale));
            }

            if (scales[i] < 1.0f) {
                constrained = true;
                dx_eigen[i] *= scales[i];
            }
        }

        if (constrained) {
            std::cerr << "[GICP-XICP] Triggered localizability constrained update, eigen scales: "
                      << scales.transpose() << std::endl;
        }

        return eig_vecs * dx_eigen;
    }

    bool GICP::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_) {
        const auto init_start = std::chrono::steady_clock::now();
        m_LastInitTimeMs = 0.0;
        if (!sourcePC_ || !targetPC_ || sourcePC_->empty() || targetPC_->empty()) return false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_target(new pcl::PointCloud<pcl::PointXYZ>());
        if (m_VoxelLeafSize > 0.0f) {
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setLeafSize(m_VoxelLeafSize, m_VoxelLeafSize, m_VoxelLeafSize);
            filter.setInputCloud(targetPC_);
            filter.filter(*processed_target);
        } else {
            processed_target = targetPC_;
        }

        if (processed_target->empty()) return false;
        std::vector<Eigen::Vector3f> target_points(processed_target->size());
        for (size_t i = 0; i < processed_target->size(); i++) {
            target_points[i] = processed_target->points[i].getVector3fMap();
        }

        auto source = PrepareSource(sourcePC_);
        const auto target_covariances = EstimateCovariances(target_points);
        auto target = PrepareTargetWithCovariances(processed_target, target_covariances);
        const bool initialized = InitPrepared(source, target);
        m_LastInitTimeMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - init_start).count();
        return initialized;
    }

    bool GICP::InitWithTargetCovariances(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_,
                                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
                                         const std::vector<Eigen::Matrix3f>& target_covariances) {
        const auto init_start = std::chrono::steady_clock::now();
        m_LastInitTimeMs = 0.0;
        if (!sourcePC_ || !targetPC_ || sourcePC_->empty() || targetPC_->empty()) return false;
        if (targetPC_->size() != target_covariances.size()) return false;

        auto source = PrepareSource(sourcePC_);
        auto target = PrepareTargetWithCovariances(targetPC_, target_covariances);
        const bool initialized = InitPrepared(source, target);
        m_LastInitTimeMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - init_start).count();
        return initialized;
    }

    GICPPreparedSource::Ptr GICP::PrepareSource(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcePC_) {
        if (!sourcePC_ || sourcePC_->empty()) return nullptr;

        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_source(new pcl::PointCloud<pcl::PointXYZ>());
        float effective_voxel_leaf_size = 0.0f;
        if (m_VoxelLeafSize > 0.0f) {
            effective_voxel_leaf_size = FilterSourceCloud(
                sourcePC_, m_VoxelLeafSize, *processed_source);
        } else {
            processed_source = sourcePC_;
        }

        if (processed_source->empty()) return nullptr;
        auto prepared = std::make_shared<GICPPreparedSource>();
        prepared->points.resize(processed_source->size());
        for (size_t i = 0; i < processed_source->size(); i++) {
            prepared->points[i] = processed_source->points[i].getVector3fMap();
        }
        prepared->covariances = EstimateCovariances(prepared->points);
        prepared->effective_voxel_leaf_size = effective_voxel_leaf_size;
        return prepared;
    }

    GICPPreparedTarget::Ptr GICP::PrepareTargetWithCovariances(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& targetPC_,
        const std::vector<Eigen::Matrix3f>& target_covariances) {
        if (!targetPC_ || targetPC_->empty() ||
            targetPC_->size() != target_covariances.size()) {
            return nullptr;
        }

        auto prepared = std::make_shared<GICPPreparedTarget>();
        prepared->points.resize(targetPC_->size());
        for (size_t i = 0; i < targetPC_->size(); i++) {
            prepared->points[i] = targetPC_->points[i].getVector3fMap();
        }
        prepared->covariances = target_covariances;
        if (!prepared->kdtree.Build(prepared->points)) return nullptr;
        return prepared;
    }

    bool GICP::InitPrepared(const GICPPreparedSource::ConstPtr& source,
                            const GICPPreparedTarget::ConstPtr& target,
                            const Eigen::Matrix3f& initial_R,
                            const Eigen::Vector3f& initial_T) {
        const auto init_start = std::chrono::steady_clock::now();
        m_LastInitTimeMs = 0.0;
        if (!source || !target || source->points.empty() || target->points.empty() ||
            source->points.size() != source->covariances.size() ||
            target->points.size() != target->covariances.size()) {
            return false;
        }

        m_PreparedSource = source;
        m_SourcePC.resize(m_PreparedSource->points.size());
        m_PreparedTarget = target;
        m_LastEffectiveVoxelLeafSize = source->effective_voxel_leaf_size;
        m_RotatedMatrix = initial_R;
        m_TransVector = initial_T;
        UpdateTransformedSource(m_RotatedMatrix, m_TransVector);
        m_LastError = std::numeric_limits<float>::max();
        m_LastValidCount = 0;
        m_LastIterations = 0;
        m_LastSolveTimeMs = 0.0;
        m_LastXicpTriggered = false;
        m_LastInitTimeMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - init_start).count();
        return true;
    }

    std::vector<Eigen::Matrix3f> GICP::EstimateCovariances(const std::vector<Eigen::Vector3f>& cloud) {
        std::vector<Eigen::Matrix3f> covariances(cloud.size(), Eigen::Matrix3f::Identity());
        if (cloud.empty()) return covariances;

        KDTree tree;
        tree.Build(cloud);
        const int k = std::max(3, std::min(m_CovarianceK, static_cast<int>(cloud.size())));
        const int thread_count = std::max(
            1, std::min(gicp_max_parallel_threads, omp_get_max_threads()));

        #pragma omp parallel num_threads(thread_count)
        {
            std::vector<int> indices;
            std::vector<double> dists;
            indices.reserve(k);
            dists.reserve(k);
            #pragma omp for
            for (int i = 0; i < static_cast<int>(cloud.size()); i++) {
                tree.SearchKNearest(cloud[i], k, indices, dists);
                covariances[i] = ComputeCovariance(cloud, indices);
            }
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

        Eigen::Vector3f evals;
        evals << std::max(gicp_normal_eigenvalue, m_CovarianceRegularization),
                 std::max(gicp_plane_eigenvalue, m_CovarianceRegularization),
                 std::max(gicp_plane_eigenvalue, m_CovarianceRegularization);
        return solver.eigenvectors() * evals.asDiagonal() * solver.eigenvectors().transpose();
    }

    void GICP::UpdateTransformedSource(const Eigen::Matrix3f& R, const Eigen::Vector3f& T) {
        const auto& source_points = m_PreparedSource->points;
        if (m_SourcePC.size() != source_points.size()) {
            m_SourcePC.resize(source_points.size());
        }

        const int thread_count = std::max(
            1, std::min(gicp_max_parallel_threads, omp_get_max_threads()));
        #pragma omp parallel for num_threads(thread_count)
        for (int i = 0; i < static_cast<int>(source_points.size()); i++) {
            m_SourcePC[i] = R * source_points[i] + T;
        }
    }

    float GICP::EvaluatePoseErrorWithFixedCorrespondences(
        const Eigen::Matrix3f& R,
        const Eigen::Vector3f& T,
        const std::vector<int>& nn_indices,
        const std::vector<std::uint8_t>& valid_flag,
        int& valid_count) const {
        valid_count = 0;
        if (!m_PreparedSource || m_PreparedSource->points.empty() || !m_PreparedTarget ||
            m_PreparedTarget->points.empty() ||
            nn_indices.size() != m_PreparedSource->points.size() ||
            valid_flag.size() != m_PreparedSource->points.size()) {
            return std::numeric_limits<float>::max();
        }
        const auto& source_points = m_PreparedSource->points;
        const auto& target_points = m_PreparedTarget->points;

        float euclidean_error = 0.0f;
        for (size_t i = 0; i < source_points.size(); ++i) {
            if (!valid_flag[i]) continue;
            const int nn_idx = nn_indices[i];
            if (nn_idx < 0 || nn_idx >= static_cast<int>(target_points.size())) continue;

            const Eigen::Vector3f source_pt = R * source_points[i] + T;
            const float dist = (source_pt - target_points[nn_idx]).norm();
            if (!std::isfinite(dist) || dist > m_MaxCorrespondenceDistance) {
                continue;
            }
            euclidean_error += dist;
            valid_count++;
        }

        if (valid_count == 0) {
            return std::numeric_limits<float>::max();
        }
        return euclidean_error / static_cast<float>(valid_count);
    }

    bool GICP::Solve(Eigen::Matrix3d& R_result_, Eigen::Vector3d& T_result_) {
        const auto solve_start = std::chrono::steady_clock::now();
        m_LastIterations = 0;
        m_LastSolveTimeMs = 0.0;
        const auto record_solve_time = [&]() {
            m_LastSolveTimeMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solve_start).count();
        };
        const int N = static_cast<int>(m_SourcePC.size());
        if (N == 0 || !m_PreparedSource || !m_PreparedTarget ||
            m_PreparedTarget->points.empty()) {
            record_solve_time();
            return false;
        }
        const auto& source_covariances = m_PreparedSource->covariances;
        const auto& target_points = m_PreparedTarget->points;
        const auto& target_covariances = m_PreparedTarget->covariances;
        auto& target_kdtree = m_PreparedTarget->kdtree;

        float prev_objective_error = std::numeric_limits<float>::max();
        float final_error = std::numeric_limits<float>::max();
        int final_valid_count = 0;
        bool has_valid_solution = false;

        std::vector<int> nn_indices(N, -1);
        std::vector<float> dists(N, 0.0f);
        std::vector<float> correspondence_metrics(N, std::numeric_limits<float>::max());
        std::vector<std::uint8_t> valid_flag(N, 0);
        AndersonAcceleration6 aa(gicp_aa_history_size);
        bool aa_initialized = false;
        const int thread_count = std::max(
            1, std::min(gicp_max_parallel_threads, omp_get_max_threads()));

        for (int iter = 0; iter < m_MaxIterations; iter++) {
            m_LastIterations = iter + 1;
            #pragma omp parallel for num_threads(thread_count)
            for (int i = 0; i < N; i++) {
                const Eigen::Matrix3f source_cov_map =
                    m_RotatedMatrix * source_covariances[i] * m_RotatedMatrix.transpose();
                double best_metric = std::numeric_limits<double>::max();
                double best_euclidean_dist = std::numeric_limits<double>::max();
                int nn_idx = target_kdtree.GetBestIdxWithMetric(
                    m_SourcePC[i],
                    std::max(1, m_CorrespondenceK),
                    [&](int target_idx) {
                        const Eigen::Vector3f residual = m_SourcePC[i] - target_points[target_idx];
                        Eigen::Matrix3f cov = target_covariances[target_idx] + source_cov_map;
                        cov += m_CovarianceRegularization * Eigen::Matrix3f::Identity();
                        Eigen::Matrix3f info = cov.inverse();
                        return static_cast<double>(residual.dot(info * residual));
                    },
                    m_MaxCorrespondenceDistance,
                    best_metric,
                    best_euclidean_dist);

                nn_indices[i] = nn_idx;
                valid_flag[i] = false;
                correspondence_metrics[i] = static_cast<float>(best_metric);

                if (nn_idx >= 0) {
                    float dist = static_cast<float>(best_euclidean_dist);
                    dists[i] = dist;
                    if (dist <= m_MaxCorrespondenceDistance && std::isfinite(best_metric)) {
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
                const Eigen::Vector3f residual = m_SourcePC[i] - target_points[nn_idx];
                Eigen::Matrix3f cov = target_covariances[nn_idx] +
                    m_RotatedMatrix * source_covariances[i] * m_RotatedMatrix.transpose();
                cov += m_CovarianceRegularization * Eigen::Matrix3f::Identity();
                Eigen::Matrix3f info = cov.inverse();

                Eigen::Matrix<float, 3, 6> J;
                J.block<3, 3>(0, 0) = -Skew(m_SourcePC[i]);
                J.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();

                H += J.transpose() * info * J;
                b += J.transpose() * info * residual;
                const float mahalanobis_error = correspondence_metrics[i];
                current_objective_error += std::sqrt(std::max(0.0f, mahalanobis_error));
                current_euclidean_error += residual.norm();
                valid_count++;
            }

            if (valid_count < gicp_min_valid_correspondences) {
                std::cerr << "[GICP] Too few correspondences: " << valid_count << std::endl;
                break;
            }

            Eigen::Matrix<float, 6, 6> H_solve = H;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> h_solver(H);
            if (h_solver.info() != Eigen::Success) {
                std::cerr << "[GICP] Hessian eigen decomposition failed" << std::endl;
                break;
            }
            const auto h_evals = h_solver.eigenvalues();
            const float min_eval = h_evals.minCoeff();
            const float max_eval = h_evals.maxCoeff();
            const float condition = max_eval / std::max(min_eval, gicp_hessian_min_eigenvalue);
            const bool hessian_degenerate =
                min_eval < gicp_hessian_min_eigenvalue || condition > gicp_hessian_max_condition;
            if (hessian_degenerate) {
                const float damping = std::max(gicp_hessian_damping, max_eval * 1e-6f);
                H_solve += damping * Eigen::Matrix<float, 6, 6>::Identity();
                std::cerr << "[GICP] Hessian degenerate, min_eval: " << min_eval
                          << ", max_eval: " << max_eval
                          << ", condition: " << condition
                          << ", damping: " << damping << std::endl;
            }

            Eigen::LDLT<Eigen::Matrix<float, 6, 6>> ldlt(H_solve);
            if (ldlt.info() != Eigen::Success) {
                std::cerr << "[GICP] Hessian LDLT failed" << std::endl;
                break;
            }
            Eigen::Matrix<float, 6, 1> dx = -ldlt.solve(b);
            if (!dx.allFinite()) {
                std::cerr << "[GICP] Linear solve failed" << std::endl;
                break;
            }
            dx = ApplyXicpConstraint(dx, h_solver, max_eval, hessian_degenerate);
            if (!dx.allFinite()) {
                std::cerr << "[GICP-XICP] Constrained update failed" << std::endl;
                break;
            }
            m_LastXicpTriggered = m_LastXicpTriggered || hessian_degenerate;

            const Eigen::Vector3f delta_rot = dx.head<3>();
            const Eigen::Vector3f delta_trans = dx.tail<3>();
            const Eigen::Matrix3f dR = ExpSO3(delta_rot);

            Eigen::Matrix3f plain_R = dR * m_RotatedMatrix;
            Eigen::Vector3f plain_T = dR * m_TransVector + delta_trans;
            int plain_valid_count = 0;
            float plain_error = EvaluatePoseErrorWithFixedCorrespondences(
                plain_R, plain_T, nn_indices, valid_flag, plain_valid_count);

            Eigen::Matrix3f accepted_R = plain_R;
            Eigen::Vector3f accepted_T = plain_T;
            float accepted_error = plain_error;
            int accepted_valid_count = plain_valid_count;
            Eigen::Matrix<float, 6, 1> accepted_step = dx;

            if (m_UseAA) {
                const Eigen::Matrix<float, 6, 1> plain_total_delta = PoseToDelta(plain_R, plain_T);
                if (!aa_initialized) {
                    aa.Init(plain_total_delta);
                    aa_initialized = true;
                } else if (iter < gicp_aa_start_iteration) {
                    aa.Compute(plain_total_delta);
                    aa.SetCurrent(plain_total_delta);
                } else if (aa_initialized && iter >= gicp_aa_start_iteration) {
                    const Eigen::Matrix<float, 6, 1> current_total_delta =
                        PoseToDelta(m_RotatedMatrix, m_TransVector);
                    Eigen::Matrix<float, 6, 1> aa_total_delta = aa.Compute(plain_total_delta);
                    Eigen::Matrix<float, 6, 1> aa_step = aa_total_delta - current_total_delta;

                    aa_step = ApplyXicpConstraint(aa_step, h_solver, max_eval, hessian_degenerate);
                    const bool aa_step_ok = aa_step.allFinite() && IsStepWithinAALimit(aa_step);
                    if (aa_step_ok) {
                        aa_total_delta = current_total_delta + aa_step;

                        Eigen::Matrix3f aa_R;
                        Eigen::Vector3f aa_T;
                        DeltaToPose(aa_total_delta, aa_R, aa_T);

                        int aa_valid_count = 0;
                        const float aa_error = EvaluatePoseErrorWithFixedCorrespondences(
                            aa_R, aa_T, nn_indices, valid_flag, aa_valid_count);
                        const bool enough_aa_inliers = aa_valid_count >= gicp_min_valid_correspondences;
                        const bool improves_or_matches =
                            std::isfinite(aa_error) && aa_error <= plain_error * gicp_aa_error_reject_ratio;

                        if (enough_aa_inliers && improves_or_matches) {
                            accepted_R = aa_R;
                            accepted_T = aa_T;
                            accepted_error = aa_error;
                            accepted_valid_count = aa_valid_count;
                            accepted_step = aa_step;
                            aa.SetCurrent(aa_total_delta);
                            if (gicp_aa_verbose) {
                                std::cerr << "[GICP-AA] Accepted AA update, plain_error: " << plain_error
                                          << ", aa_error: " << aa_error
                                          << ", valid: " << aa_valid_count << std::endl;
                            }
                        } else {
                            aa.SetCurrent(plain_total_delta);
                            if (gicp_aa_verbose) {
                                std::cerr << "[GICP-AA] Rejected AA update, plain_error: " << plain_error
                                          << ", aa_error: " << aa_error
                                          << ", aa_valid: " << aa_valid_count << std::endl;
                            }
                        }
                    } else {
                        aa.SetCurrent(plain_total_delta);
                        if (gicp_aa_verbose) {
                            std::cerr << "[GICP-AA] Rejected AA update by step limit, trans: "
                                      << aa_step.tail<3>().norm()
                                      << ", rot_deg: "
                                      << aa_step.head<3>().norm() * 180.0f / static_cast<float>(M_PI)
                                      << std::endl;
                        }
                    }
                }
            }

            m_RotatedMatrix = accepted_R;
            m_TransVector = accepted_T;
            UpdateTransformedSource(m_RotatedMatrix, m_TransVector);

            if (accepted_valid_count < gicp_min_valid_correspondences || !std::isfinite(accepted_error)) {
                std::cerr << "[GICP] Accepted update has too few correspondences: "
                          << accepted_valid_count << std::endl;
                break;
            }

            current_objective_error /= static_cast<float>(valid_count);
            current_euclidean_error /= static_cast<float>(valid_count);
            final_error = accepted_error;
            final_valid_count = accepted_valid_count;
            has_valid_solution = true;

            if (accepted_step.norm() < gicp_min_step_norm ||
                std::abs(prev_objective_error - current_objective_error) < gicp_min_error_delta) {
                break;
            }
            prev_objective_error = current_objective_error;
        }

        if (!has_valid_solution) {
            std::cerr << "[GICP] Solve failed, no valid iteration." << std::endl;
            record_solve_time();
            return false;
        }

        if (final_error > m_MaxCorrespondenceDistance) {
            std::cerr << "[GICP] Solve rejected, error: " << final_error
                      << ", valid correspondences: " << final_valid_count << std::endl;
            record_solve_time();
            return false;
        }

        m_LastError = final_error;
        m_LastValidCount = final_valid_count;
        R_result_ = m_RotatedMatrix.cast<double>();
        T_result_ = m_TransVector.cast<double>();
        record_solve_time();
        return true;
    }

}
