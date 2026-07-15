#include "relocation/location.hpp"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <pcl/common/transforms.h>

namespace relocation {

    namespace {
        double NormalizeAngleDeg(double angle) {
            angle = std::fmod(angle, 360.0);
            if (angle < 0.0) angle += 360.0;
            return angle;
        }

        double AngleDiffDeg(double a, double b) {
            double diff = std::abs(NormalizeAngleDeg(a) - NormalizeAngleDeg(b));
            return std::min(diff, 360.0 - diff);
        }

        double CircularMeanDeg(double first, double second, double first_weight) {
            const double second_weight = 1.0 - first_weight;
            const double first_rad = first * M_PI / 180.0;
            const double second_rad = second * M_PI / 180.0;
            const double sine = first_weight * std::sin(first_rad) +
                                second_weight * std::sin(second_rad);
            const double cosine = first_weight * std::cos(first_rad) +
                                  second_weight * std::cos(second_rad);
            return NormalizeAngleDeg(std::atan2(sine, cosine) * 180.0 / M_PI);
        }
    }

    bool location::LoadDatabase(const std::string& filename) {
        std::ifstream in(filename);
        if (!in.is_open()) return false;
        size_t size; 
        if (!(in >> size)) return false;
        m_history_db.resize(size);
        for (size_t i = 0; i < size; ++i) {
            in >> m_history_db[i].x >> m_history_db[i].y >> m_history_db[i].yaw;
            size_t vec_size; 
            in >> vec_size;
            m_history_db[i].binary_vec.resize(vec_size);
            for (size_t j = 0; j < vec_size; ++j) { 
                int val; in >> val; 
                m_history_db[i].binary_vec[j] = val; 
            }
            auto load_mat = [&](cv::Mat1b& mat) {
                int rows, cols; 
                in >> rows >> cols; 
                mat.create(rows, cols);
                for (int r = 0; r < rows; ++r) { 
                    for (int c = 0; c < cols; ++c) { 
                        int val; in >> val; 
                        mat(r, c) = val; 
                    } 
                }
            };
            load_mat(m_history_db[i].desc.img); 
            load_mat(m_history_db[i].desc.T); 
            load_mat(m_history_db[i].desc.M);
        }
        in.close(); 
        return true;
    }

    bool location::Init(const std::string& db_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& global_map) {
        if (!global_map || global_map->empty()) {
            std::cerr << "[location::Init] Map empty" << std::endl;
            return false;
        }
        m_global_map = global_map;
        if (!LoadDatabase(db_path)) {
            std::cerr << "[location::Init] Load failed" << std::endl;
            return false;
        }
        m_iris_tree = std::make_unique<ann_kdtree>();
        std::vector<std::vector<uint8_t>> kd_input;
        kd_input.reserve(m_history_db.size());
        for (const auto& node : m_history_db) {
            if (node.binary_vec.size() != iris_binary_vec_size) {
                std::cerr << "[location::Init] DB feature dimension mismatch. Expected "
                          << iris_binary_vec_size << ", got " << node.binary_vec.size()
                          << ". Please regenerate history_db.txt." << std::endl;
                return false;
            }
            kd_input.push_back(node.binary_vec);
        }
        if (!m_iris_tree->Build(kd_input)) {
            std::cerr << "[location::Init] KDTree failed" << std::endl;
            return false;
        }

        auto scan_context = std::make_unique<ScanContextPlusPlus>(ScanContextConfig::IrisPolar());
        const std::string sc_database_path = MakeScanContextDatabasePath(db_path);
        if (scan_context->LoadDatabase(sc_database_path) &&
            scan_context->PlaceCount() == m_history_db.size()) {
            m_scan_context = std::move(scan_context);
            std::cout << "[location::Init] SC++ database loaded: "
                      << sc_database_path << ", places=" << m_scan_context->PlaceCount()
                      << ", descriptors=" << m_scan_context->DescriptorCount() << std::endl;
        } else {
            m_scan_context.reset();
            std::cerr << "[location::Init] SC++ database unavailable or incompatible: "
                      << sc_database_path << ". Falling back to IRIS-only retrieval." << std::endl;
        }

        std::vector<Eigen::Vector3f> map_points;
        map_points.reserve(m_global_map->size());
        for (const auto& pt : m_global_map->points) {
            map_points.emplace_back(pt.x, pt.y, pt.z);
        }
        GICP covariance_builder;
        m_global_map_covariances = covariance_builder.EstimateCovariances(map_points);
        if (m_global_map_covariances.size() != m_global_map->size()) {
            std::cerr << "[location::Init] Global map covariance precompute failed" << std::endl;
            m_global_map_covariances.clear();
            return false;
        }
        return true;
    }

    void location::GetPrePose(const double& x, const double& y, const double& yaw) {
        m_pre_x = x;
        m_pre_y = y;
        m_pre_yaw = yaw;
        m_has_pre_pose = true;
    }

    bool location::SetRoughPose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                double& out_x, double& out_y, double& out_yaw) 
    {
        std::vector<RoughPoseCandidate> candidates;
        if (!GetRoughPoseCandidatesWithPrePose(local_cloud, 0.0, 0.0, 0.0, candidates)) {
            return false;
        }
        out_x = candidates.front().x;
        out_y = candidates.front().y;
        out_yaw = candidates.front().yaw_deg;
        return true;
    }

    bool location::SetRoughPoseWithPrePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                           const double& pre_x, const double& pre_y, const double& pre_yaw, 
                                           double& out_x, double& out_y, double& out_yaw) 
    {
        std::vector<RoughPoseCandidate> candidates;
        if (!GetRoughPoseCandidatesWithPrePose(local_cloud, pre_x, pre_y, pre_yaw, candidates)) return false;
        out_x = candidates.front().x;
        out_y = candidates.front().y;
        out_yaw = candidates.front().yaw_deg;
        return true;
    }

    bool location::GetRoughPoseCandidatesWithPrePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                                                     const double& pre_x, const double& pre_y, const double& pre_yaw,
                                                     std::vector<RoughPoseCandidate>& candidates)
    {
        candidates.clear();
        if (!local_cloud || local_cloud->empty()) return false;
        cv::Mat1b query_iris = iris::GetIris(*local_cloud);
        auto query_desc = iris::GetFeature(query_iris);
        auto query_binary = iris::IrisToBinaryVec(query_iris);

        std::vector<int> iris_indices;
        std::vector<int> iris_distances;
        m_iris_tree->SearchKNearest(
            query_binary, loc_rough_k_candidates, iris_indices, iris_distances);

        std::vector<int> candidate_indices;
        std::unordered_set<int> candidate_set;
        std::unordered_set<int> iris_retrieved;
        for (int idx : iris_indices) {
            if (idx < 0 || idx >= static_cast<int>(m_history_db.size())) continue;
            iris_retrieved.insert(idx);
            if (candidate_set.insert(idx).second) {
                candidate_indices.push_back(idx);
            }
        }

        ScanContextDescriptor query_sc;
        bool sc_query_valid = false;
        std::unordered_map<int, ScanContextMatch> sc_retrieved;
        if (m_scan_context) {
            try {
                query_sc = m_scan_context->MakeDescriptor(local_cloud);
                sc_query_valid = true;
                for (const auto& match : m_scan_context->QueryCandidates(
                         query_sc, loc_sc_rough_k_candidates)) {
                    if (match.place_id < 0 ||
                        match.place_id >= static_cast<int>(m_history_db.size()) ||
                        !match.matched ||
                        !std::isfinite(match.distance)) {
                        continue;
                    }
                    sc_retrieved[match.place_id] = match;
                    if (candidate_set.insert(match.place_id).second) {
                        candidate_indices.push_back(match.place_id);
                    }
                }
            } catch (const std::exception& exception) {
                std::cerr << "[location] SC++ query failed: " << exception.what()
                          << ". Using IRIS-only candidates." << std::endl;
                sc_query_valid = false;
            }
        }

        const double pre_yaw_deg = pre_yaw * 180.0 / M_PI;
        bool has_valid_sc_match = false;
        for (int idx : candidate_indices) {
            const auto& history = m_history_db[idx];
            int bias = 0;
            const float iris_score = iris::Compare(query_desc, history.desc, &bias);
            const double iris_yaw = NormalizeAngleDeg(history.yaw + bias);

            RoughPoseCandidate candidate;
            candidate.x = history.x;
            candidate.y = history.y;
            candidate.yaw_deg = iris_yaw;
            candidate.rough_score = iris_score;
            candidate.iris_score = iris_score;
            candidate.iris_yaw_deg = iris_yaw;
            candidate.iris_retrieved = iris_retrieved.count(idx) != 0;
            candidate.sc_retrieved = sc_retrieved.count(idx) != 0;
            candidate.hist_index = idx;

            if (sc_query_valid) {
                const auto retrieved_iter = sc_retrieved.find(idx);
                const ScanContextMatch sc_match = retrieved_iter != sc_retrieved.end()
                                                      ? retrieved_iter->second
                                                      : m_scan_context->ComparePlace(query_sc, idx);
                if (sc_match.place_id == idx && std::isfinite(sc_match.distance)) {
                    candidate.sc_available = true;
                    candidate.sc_score = sc_match.distance;

                    if (sc_match.matched) {
                        candidate.sc_matched = true;
                        has_valid_sc_match = true;
                        candidate.sc_lateral_shift = sc_match.relative_lateral_m;
                        candidate.sc_variant = sc_match.variant;
                        candidate.sc_yaw_deg = NormalizeAngleDeg(
                            history.yaw - sc_match.relative_yaw_rad * 180.0 / M_PI);
                        candidate.descriptor_yaw_diff_deg = AngleDiffDeg(
                            candidate.iris_yaw_deg, candidate.sc_yaw_deg);
                    }

                    if (candidate.sc_matched &&
                        candidate.descriptor_yaw_diff_deg <=
                            loc_fusion_yaw_consistency_limit_deg) {
                        const double history_yaw_rad = history.yaw * M_PI / 180.0;
                        candidate.x -= std::sin(history_yaw_rad) * candidate.sc_lateral_shift;
                        candidate.y += std::cos(history_yaw_rad) * candidate.sc_lateral_shift;
                    }
                }
            }
            candidates.push_back(candidate);
        }

        if (candidates.empty()) return false;

        if (has_valid_sc_match) {
            std::vector<std::size_t> iris_order(candidates.size());
            std::vector<std::size_t> sc_order;
            sc_order.reserve(candidates.size());
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                iris_order[i] = i;
                candidates[i].fusion_active = true;
                if (candidates[i].sc_matched) {
                    sc_order.push_back(i);
                }
            }
            std::sort(iris_order.begin(), iris_order.end(), [&](std::size_t lhs, std::size_t rhs) {
                return candidates[lhs].iris_score < candidates[rhs].iris_score;
            });
            std::sort(sc_order.begin(), sc_order.end(), [&](std::size_t lhs, std::size_t rhs) {
                return candidates[lhs].sc_score < candidates[rhs].sc_score;
            });

            std::vector<std::size_t> iris_rank(candidates.size());
            std::vector<std::size_t> sc_rank(candidates.size(), candidates.size());
            for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
                iris_rank[iris_order[rank]] = rank;
            }
            for (std::size_t rank = 0; rank < sc_order.size(); ++rank) {
                sc_rank[sc_order[rank]] = rank;
            }

            const double rank_denominator = std::max(1.0, static_cast<double>(candidates.size() - 1));
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                auto& candidate = candidates[i];
                const double iris_rank_cost = static_cast<double>(iris_rank[i]) / rank_denominator;
                if (candidate.sc_matched) {
                    const double sc_rank_cost =
                        static_cast<double>(sc_rank[i]) / rank_denominator;
                    const double yaw_penalty = std::min(
                        candidate.descriptor_yaw_diff_deg /
                            loc_fusion_yaw_consistency_limit_deg,
                        1.0);
                    candidate.rough_score = loc_fusion_base_score +
                                            loc_fusion_iris_weight * iris_rank_cost +
                                            loc_fusion_sc_weight * sc_rank_cost +
                                            loc_fusion_yaw_weight * yaw_penalty;

                    if (candidate.descriptor_yaw_diff_deg <=
                        loc_fusion_yaw_mean_limit_deg) {
                        candidate.yaw_deg = CircularMeanDeg(
                            candidate.iris_yaw_deg,
                            candidate.sc_yaw_deg,
                            loc_fusion_iris_weight);
                    } else if (candidate.descriptor_yaw_diff_deg <=
                                   loc_fusion_yaw_consistency_limit_deg &&
                               sc_rank[i] < iris_rank[i]) {
                        candidate.yaw_deg = candidate.sc_yaw_deg;
                    } else {
                        candidate.yaw_deg = candidate.iris_yaw_deg;
                    }
                } else {
                    // SC++ rejected this place. Keep its IRIS rank on the same
                    // [base, base + 1] scale without using the rejected SC score.
                    candidate.rough_score = loc_fusion_base_score + iris_rank_cost;
                    candidate.yaw_deg = candidate.iris_yaw_deg;
                }
                candidate.dist_to_prior = std::hypot(
                    candidate.x - pre_x, candidate.y - pre_y);
                candidate.yaw_diff_deg = AngleDiffDeg(candidate.yaw_deg, pre_yaw_deg);
            }
        } else {
            for (auto& candidate : candidates) {
                candidate.dist_to_prior = std::hypot(
                    candidate.x - pre_x, candidate.y - pre_y);
                candidate.yaw_diff_deg = AngleDiffDeg(candidate.yaw_deg, pre_yaw_deg);
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.rough_score != rhs.rough_score) return lhs.rough_score < rhs.rough_score;
            return lhs.iris_score < rhs.iris_score;
        });
        if (candidates.size() > loc_max_rough_pose_candidates) {
            candidates.resize(loc_max_rough_pose_candidates);
        }
        return !candidates.empty();
    }

    bool location::SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                  Eigen::Matrix3d& R, Eigen::Vector3d& T) 
    {
        double match_error = std::numeric_limits<double>::max();
        int valid_count = 0;
        return SetPrecisePose(local_cloud, R, T, match_error, valid_count);
    }

    bool location::SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                                  Eigen::Matrix3d& R, Eigen::Vector3d& T,
                                  double& out_match_error, int& out_valid_count)
    {
        return SetPrecisePose(local_cloud, R, T, out_match_error, out_valid_count,
                              gicp_max_iterations, gicp_voxel_leaf_size);
    }

    bool location::SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
                                  Eigen::Matrix3d& R, Eigen::Vector3d& T,
                                  double& out_match_error, int& out_valid_count,
                                  int max_iterations, float voxel_leaf_size,
                                  int correspondence_k)
    {
        m_last_source_point_count = 0;
        m_last_target_point_count = 0;
        m_last_gicp_iterations = 0;
        m_last_gicp_crop_time_ms = 0.0;
        m_last_gicp_init_time_ms = 0.0;
        m_last_gicp_solve_time_ms = 0.0;
        m_last_gicp_effective_voxel_leaf_size = 0.0f;
        m_last_xicp_triggered = false;
        if (!local_cloud || local_cloud->empty() || !m_global_map) return false;
        out_match_error = std::numeric_limits<double>::max();
        out_valid_count = 0;
        double coarse_x = T.x();
        double coarse_y = T.y();
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_map(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<Eigen::Matrix3f> target_covariances;
        target_covariances.reserve(m_global_map->size());
        const auto crop_start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < m_global_map->points.size(); ++i) {
            const auto& pt = m_global_map->points[i];
            if (pt.z < loc_icp_crop_z_min || pt.z > loc_icp_crop_z_max) continue;
            if (std::abs(pt.x - coarse_x) < loc_icp_crop_xy && std::abs(pt.y - coarse_y) < loc_icp_crop_xy) {
                target_map->push_back(pt);
                if (i < m_global_map_covariances.size()) {
                    target_covariances.push_back(m_global_map_covariances[i]);
                }
            }
        }
        if (target_map->empty()) {
            m_last_gicp_crop_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - crop_start).count();
            return false;
        }
        if (target_covariances.size() != target_map->size()) {
            target_covariances.clear();
        }
        if (voxel_leaf_size <= gicp_voxel_leaf_size &&
            target_map->size() >
            static_cast<std::size_t>(loc_gicp_max_target_points)) {
            pcl::PointCloud<pcl::PointXYZ> limited_target;
            limited_target.reserve(loc_gicp_max_target_points);
            std::vector<Eigen::Matrix3f> limited_covariances;
            if (!target_covariances.empty()) {
                limited_covariances.reserve(loc_gicp_max_target_points);
            }
            const double stride = static_cast<double>(target_map->size()) /
                                  static_cast<double>(loc_gicp_max_target_points);
            for (int i = 0; i < loc_gicp_max_target_points; ++i) {
                const std::size_t index = std::min(
                    static_cast<std::size_t>(i * stride), target_map->size() - 1);
                limited_target.push_back((*target_map)[index]);
                if (!target_covariances.empty()) {
                    limited_covariances.push_back(target_covariances[index]);
                }
            }
            target_map->swap(limited_target);
            if (!target_covariances.empty()) {
                target_covariances.swap(limited_covariances);
            }
        }
        m_last_gicp_crop_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - crop_start).count();
        Eigen::Matrix4f init_tf = Eigen::Matrix4f::Identity();
        init_tf.block<3,3>(0,0) = R.cast<float>();
        init_tf.block<3,1>(0,3) = T.cast<float>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*local_cloud, *source_aligned, init_tf);
        GICP gicp;
        gicp.SetMaxIterations(max_iterations);
        gicp.SetVoxelLeafSize(voxel_leaf_size);
        gicp.SetCorrespondenceK(correspondence_k);
        const bool init_ok = target_covariances.empty()
                                 ? gicp.Init(source_aligned, target_map)
                                 : gicp.InitWithTargetCovariances(source_aligned, target_map, target_covariances);
        m_last_source_point_count = gicp.GetSourcePointCount();
        m_last_target_point_count = gicp.GetTargetPointCount();
        m_last_gicp_init_time_ms = gicp.GetLastInitTimeMs();
        m_last_gicp_effective_voxel_leaf_size =
            gicp.GetLastEffectiveVoxelLeafSize();
        if (init_ok) {
            Eigen::Matrix3d R_gicp;
            Eigen::Vector3d T_gicp;
            const bool solve_ok = gicp.Solve(R_gicp, T_gicp);
            m_last_gicp_iterations = gicp.GetLastIterations();
            m_last_gicp_solve_time_ms = gicp.GetLastSolveTimeMs();
            if (solve_ok) {
                R = R_gicp * R;
                T = R_gicp * T + T_gicp;
                out_match_error = gicp.GetLastError();
                out_valid_count = gicp.GetLastValidCount();
                m_last_xicp_triggered = gicp.WasLastXicpTriggered();
                return true;
            }
        }
        return false;
    }

    bool location::SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud) {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d T = Eigen::Vector3d::Zero();
        return SetPrecisePose(local_cloud, R, T);
    }

}
