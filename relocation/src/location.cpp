#include "relocation/location.hpp"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

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

        bool HasMatchingScanContextPlaceIds(
            const ScanContextPlusPlus& scan_context,
            std::size_t expected_place_count) {
            if (scan_context.PlaceCount() != expected_place_count) {
                return false;
            }
            std::vector<bool> seen(expected_place_count, false);
            for (const auto& descriptor : scan_context.Descriptors()) {
                if (descriptor.place_id < 0 ||
                    descriptor.place_id >= static_cast<int>(expected_place_count)) {
                    return false;
                }
                seen[static_cast<std::size_t>(descriptor.place_id)] = true;
            }
            return std::all_of(seen.begin(), seen.end(), [](bool value) { return value; });
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
        ScanContextDatabaseIdentity expected_identity;
        const bool identity_ready = ComputeScanContextDatabaseIdentity(
            db_path, m_global_map, expected_identity);
        if (identity_ready &&
            scan_context->LoadDatabase(sc_database_path, expected_identity) &&
            HasMatchingScanContextPlaceIds(*scan_context, m_history_db.size())) {
            m_scan_context = std::move(scan_context);
            std::cout << "[location::Init] SC++ database loaded: "
                      << sc_database_path << ", places=" << m_scan_context->PlaceCount()
                      << ", descriptors=" << m_scan_context->DescriptorCount() << std::endl;
        } else {
            m_scan_context.reset();
            std::cerr << "[location::Init] SC++ database unavailable, incompatible, or not "
                         "paired with the current IRIS database and map: "
                      << sc_database_path << ". Falling back to IRIS-only retrieval." << std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr registration_target(
            new pcl::PointCloud<pcl::PointXYZ>());
        registration_target->reserve(m_global_map->size());
        for (const auto& pt : m_global_map->points) {
            if (pt.z >= loc_icp_crop_z_min && pt.z <= loc_icp_crop_z_max) {
                registration_target->push_back(pt);
            }
        }
        if (registration_target->empty()) {
            std::cerr << "[location::Init] No map points remain after GICP height filtering"
                      << std::endl;
            return false;
        }

        std::vector<Eigen::Vector3f> map_points;
        map_points.reserve(registration_target->size());
        for (const auto& pt : registration_target->points) {
            map_points.emplace_back(pt.x, pt.y, pt.z);
        }
        GICP covariance_builder;
        auto target_covariances = covariance_builder.EstimateCovariances(map_points);
        if (target_covariances.size() != registration_target->size()) {
            std::cerr << "[location::Init] Global map covariance precompute failed" << std::endl;
            return false;
        }
        m_gicp_target = covariance_builder.PrepareTargetWithCovariances(
            registration_target, target_covariances);
        if (!m_gicp_target) {
            std::cerr << "[location::Init] Global GICP target preparation failed" << std::endl;
            return false;
        }
        m_gicp_source_cache.clear();
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
        bool has_fusion_candidate = false;
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
                        candidate.sc_lateral_shift = sc_match.relative_lateral_m;
                        candidate.sc_variant = sc_match.variant;
                        candidate.sc_yaw_deg = NormalizeAngleDeg(
                            history.yaw - sc_match.relative_yaw_rad * 180.0 / M_PI);
                        candidate.descriptor_yaw_diff_deg = AngleDiffDeg(
                            candidate.iris_yaw_deg, candidate.sc_yaw_deg);
                    }

                    candidate.fusion_active = candidate.sc_matched &&
                        candidate.descriptor_yaw_diff_deg <=
                            loc_fusion_yaw_consistency_limit_deg;
                    if (candidate.fusion_active) {
                        has_fusion_candidate = true;
                        const double history_yaw_rad = history.yaw * M_PI / 180.0;
                        candidate.x -= std::sin(history_yaw_rad) * candidate.sc_lateral_shift;
                        candidate.y += std::cos(history_yaw_rad) * candidate.sc_lateral_shift;
                    }
                }
            }
            candidates.push_back(candidate);
        }

        if (candidates.empty()) return false;

        if (has_fusion_candidate) {
            std::vector<std::size_t> iris_order(candidates.size());
            std::vector<std::size_t> sc_order;
            sc_order.reserve(candidates.size());
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                iris_order[i] = i;
                if (candidates[i].fusion_active) {
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
            const double sc_rank_denominator =
                std::max(1.0, static_cast<double>(sc_order.size() - 1));
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                auto& candidate = candidates[i];
                const double iris_rank_cost = static_cast<double>(iris_rank[i]) / rank_denominator;
                if (candidate.fusion_active) {
                    const double sc_rank_cost =
                        static_cast<double>(sc_rank[i]) / sc_rank_denominator;
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
                    // A missing, rejected, or yaw-inconsistent SC++ result pays the
                    // complete SC+yaw cost. It can still survive through a strong
                    // IRIS rank, but no longer receives a fusion-quality score.
                    candidate.rough_score = loc_fusion_base_score +
                                            loc_fusion_sc_rejection_cost +
                                            loc_fusion_iris_weight * iris_rank_cost;
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

    GICPPreparedSource::ConstPtr location::GetPreparedGicpSource(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud,
        float voxel_leaf_size) {
        if (!local_cloud || local_cloud->empty()) return nullptr;

        auto cache_iter = std::find_if(
            m_gicp_source_cache.begin(), m_gicp_source_cache.end(),
            [&](const GicpSourceCacheEntry& entry) {
                return std::abs(entry.voxel_leaf_size - voxel_leaf_size) < 1e-6f;
            });

        if (cache_iter != m_gicp_source_cache.end() &&
            cache_iter->input.get() == local_cloud.get() &&
            cache_iter->prepared) {
            return cache_iter->prepared;
        }

        GICP source_builder;
        source_builder.SetVoxelLeafSize(voxel_leaf_size);
        auto prepared = source_builder.PrepareSource(local_cloud);
        if (!prepared) return nullptr;

        if (cache_iter == m_gicp_source_cache.end()) {
            m_gicp_source_cache.push_back(
                GicpSourceCacheEntry{voxel_leaf_size, local_cloud, prepared});
        } else {
            cache_iter->input = local_cloud;
            cache_iter->prepared = prepared;
        }
        return prepared;
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
        if (!local_cloud || local_cloud->empty() || !m_gicp_target) return false;
        out_match_error = std::numeric_limits<double>::max();
        out_valid_count = 0;

        const auto init_start = std::chrono::steady_clock::now();
        auto prepared_source = GetPreparedGicpSource(local_cloud, voxel_leaf_size);
        if (!prepared_source) {
            m_last_gicp_init_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - init_start).count();
            return false;
        }

        GICP gicp;
        gicp.SetMaxIterations(max_iterations);
        gicp.SetVoxelLeafSize(voxel_leaf_size);
        gicp.SetCorrespondenceK(correspondence_k);
        const bool init_ok = gicp.InitPrepared(
            prepared_source, m_gicp_target, R.cast<float>(), T.cast<float>());
        m_last_gicp_init_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - init_start).count();
        m_last_source_point_count = gicp.GetSourcePointCount();
        m_last_target_point_count = gicp.GetTargetPointCount();
        m_last_gicp_effective_voxel_leaf_size =
            gicp.GetLastEffectiveVoxelLeafSize();
        if (init_ok) {
            Eigen::Matrix3d R_gicp;
            Eigen::Vector3d T_gicp;
            const bool solve_ok = gicp.Solve(R_gicp, T_gicp);
            m_last_gicp_iterations = gicp.GetLastIterations();
            m_last_gicp_solve_time_ms = gicp.GetLastSolveTimeMs();
            if (solve_ok) {
                R = R_gicp;
                T = T_gicp;
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
