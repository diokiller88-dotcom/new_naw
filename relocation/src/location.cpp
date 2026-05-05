#include "relocation/location.hpp"
#include <fstream>
#include <cmath>
#include <limits>
#include <chrono>
#include <pcl/common/transforms.h>

namespace relocation {

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
            kd_input.push_back(node.binary_vec);
        }
        if (!m_iris_tree->Build(kd_input)) {
            std::cerr << "[location::Init] KDTree failed" << std::endl;
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
        if (!local_cloud || local_cloud->empty()) return false;
        cv::Mat1b query_iris = iris::GetIris(*local_cloud);
        auto query_desc = iris::GetFeature(query_iris);
        auto query_binary = iris::IrisToBinaryVec(query_iris);
        std::vector<int> out_indices;
        std::vector<int> out_dists;
        m_iris_tree->SearchKNearest(query_binary, loc_rough_k_candidates, out_indices, out_dists);
        int best_match_idx = -1;
        float best_score = std::numeric_limits<float>::max();
        int best_yaw_bias = 0;
        for (int idx : out_indices) {
            int bias = 0;
            float score = iris::Compare(query_desc, m_history_db[idx].desc, &bias);
            if (score < best_score) {
                best_score = score;
                best_match_idx = idx;
                best_yaw_bias = bias;
            }
        }
        if (best_match_idx == -1) return false;
        out_x = m_history_db[best_match_idx].x;
        out_y = m_history_db[best_match_idx].y;
        out_yaw = std::fmod(best_yaw_bias + 360.0, 360.0);
        return true;
    }

    bool location::SetRoughPoseWithPrePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                           const double& pre_x, const double& pre_y, const double& pre_yaw, 
                                           double& out_x, double& out_y, double& out_yaw) 
    {
        if (!local_cloud || local_cloud->empty()) return false;
        cv::Mat1b query_iris = iris::GetIris(*local_cloud);
        auto query_desc = iris::GetFeature(query_iris);
        auto query_binary = iris::IrisToBinaryVec(query_iris);
        std::vector<int> out_indices;
        std::vector<int> out_dists;
        m_iris_tree->SearchKNearest(query_binary, loc_rough_k_candidates, out_indices, out_dists);
        int best_match_idx = -1;
        float best_score = std::numeric_limits<float>::max();
        int best_yaw_bias = 0;
        for (int idx : out_indices) {
            float cand_x = m_history_db[idx].x;
            float cand_y = m_history_db[idx].y;
            float dist_to_prior = std::hypot(cand_x - pre_x, cand_y - pre_y);
            if (dist_to_prior > loc_prior_radius_limit) continue; 
            int bias = 0;
            float score = iris::Compare(query_desc, m_history_db[idx].desc, &bias);
            float penalty_factor = 1.0f + loc_prior_penalty_scale * (dist_to_prior / loc_prior_radius_limit);
            score *= penalty_factor;
            if (score < best_score) {
                best_score = score;
                best_match_idx = idx;
                best_yaw_bias = bias;
            }
        }
        if (best_match_idx == -1) return false;
        out_x = m_history_db[best_match_idx].x;
        out_y = m_history_db[best_match_idx].y;
        out_yaw = std::fmod(best_yaw_bias + 360.0, 360.0);
        return true;
    }

    bool location::SetPrecisePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud, 
                                  Eigen::Matrix3d& R, Eigen::Vector3d& T) 
    {
        if (!local_cloud || local_cloud->empty() || !m_global_map) return false;
        double coarse_x = T.x();
        double coarse_y = T.y();
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_map(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : m_global_map->points) {
            if (pt.z < loc_icp_crop_z_min || pt.z > loc_icp_crop_z_max) continue;
            if (std::abs(pt.x - coarse_x) < loc_icp_crop_xy && std::abs(pt.y - coarse_y) < loc_icp_crop_xy) {
                target_map->push_back(pt);
            }
        }
        if (target_map->empty()) return false;
        Eigen::Matrix4f init_tf = Eigen::Matrix4f::Identity();
        init_tf.block<3,3>(0,0) = R.cast<float>();
        init_tf.block<3,1>(0,3) = T.cast<float>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*local_cloud, *source_aligned, init_tf);
        P2PointICP_SVD icp;
        if (icp.Init(source_aligned, target_map)) {
            Eigen::Matrix3d R_icp;
            Eigen::Vector3d T_icp;
            if (icp.Solve(R_icp, T_icp)) {
                R = R_icp * R;
                T = R_icp * T + T_icp;
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