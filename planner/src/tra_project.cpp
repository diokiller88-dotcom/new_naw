#include "planner_2d/tra_project.hpp"
#include <iostream>

namespace planner_2d {

    void project::BuildAABBTree(const minco& traj) {
        m_Path = traj.GetPath();
        m_Times = traj.GetTimes();
        
        if (m_Path.empty()) {
            m_RootNodeIdx = -1;
            return;
        }

        std::vector<SegmentInfo> seg_infos(m_Path.size());
        for (size_t i = 0; i < m_Path.size(); ++i) {
            seg_infos[i].id = i;
            seg_infos[i].aabb = ComputeSegmentAABB(m_Path[i], m_Times(i));
            seg_infos[i].centroid = seg_infos[i].aabb.GetCentroid();
        }

        m_TreeNodes.clear();
        m_TreeNodes.reserve(2 * m_Path.size());

        m_RootNodeIdx = BuildTreeRecursive(seg_infos, 0, seg_infos.size());
    }

    int project::BuildTreeRecursive(std::vector<SegmentInfo>& seg_infos, int start, int end) {
        if (start >= end) return -1;

        int node_idx = m_TreeNodes.size();
        m_TreeNodes.push_back(AABBNode());
        AABBNode& node = m_TreeNodes.back();

        int count = end - start;

        AABB total_aabb = seg_infos[start].aabb;
        for (int i = start + 1; i < end; ++i) {
            total_aabb = AABB::Merge(total_aabb, seg_infos[i].aabb);
        }
        m_TreeNodes[node_idx].aabb = total_aabb;

        if (count == 1) {
            m_TreeNodes[node_idx].seg_idx = seg_infos[start].id;
            return node_idx;
        }

        double extent_x = total_aabb.max_pt.x() - total_aabb.min_pt.x();
        double extent_y = total_aabb.max_pt.y() - total_aabb.min_pt.y();
        int split_axis = (extent_x > extent_y) ? 0 : 1;

        std::sort(seg_infos.begin() + start, seg_infos.begin() + end,
            [split_axis](const SegmentInfo& a, const SegmentInfo& b) {
                return (split_axis == 0) ? (a.centroid.x() < b.centroid.x()) 
                                         : (a.centroid.y() < b.centroid.y());
            });

        int mid = start + count / 2;

        int left_idx = BuildTreeRecursive(seg_infos, start, mid);
        int right_idx = BuildTreeRecursive(seg_infos, mid, end);

        m_TreeNodes[node_idx].left_child = left_idx;
        m_TreeNodes[node_idx].right_child = right_idx;

        return node_idx;
    }

    ProjectionResult project::FindClosestPoint(const Eigen::Vector2d& vehicle_pos, double warm_start_dist_sq) const {
        ProjectionResult result;
        if (m_RootNodeIdx == -1 || m_TreeNodes.empty()) {
            return result;
        }

        result.dist_sq = warm_start_dist_sq;
        QueryTreeRecursive(m_RootNodeIdx, vehicle_pos, result);
        return result;
    }

    void project::QueryTreeRecursive(int node_idx, const Eigen::Vector2d& pos, ProjectionResult& best_res) const {
        if (node_idx == -1) return;
        const auto& node = m_TreeNodes[node_idx];

        double dist_to_box_sq = node.aabb.SquaredDistanceTo(pos);
        if (dist_to_box_sq >= best_res.dist_sq) {
            return; 
        }

        if (node.IsLeaf()) {
            EvaluateExactProjection(node.seg_idx, pos, best_res);
            return;
        }

        double d_left = m_TreeNodes[node.left_child].aabb.SquaredDistanceTo(pos);
        double d_right = m_TreeNodes[node.right_child].aabb.SquaredDistanceTo(pos);

        if (d_left < d_right) {
            QueryTreeRecursive(node.left_child, pos, best_res);
            QueryTreeRecursive(node.right_child, pos, best_res);
        } else {
            QueryTreeRecursive(node.right_child, pos, best_res);
            QueryTreeRecursive(node.left_child, pos, best_res);
        }
    }

    void project::EvaluateExactProjection(int seg_idx, const Eigen::Vector2d& pos, ProjectionResult& best_res) const {
        const auto& poly = m_Path[seg_idx];
        double T = m_Times(seg_idx);

        int num_samples = std::max(5, static_cast<int>(T / 0.05));
        double dt = T / num_samples;
        double t_coarse = 0.0;
        double min_d_sq = std::numeric_limits<double>::max();

        for (int i = 0; i <= num_samples; ++i) {
            double t = i * dt;
            double t2 = t * t;
            double t3 = t2 * t;
            double x = poly.a0_x + poly.a1_x*t + poly.a2_x*t2 + poly.a3_x*t3;
            double y = poly.a0_y + poly.a1_y*t + poly.a2_y*t2 + poly.a3_y*t3;
            double d_sq = (x - pos.x())*(x - pos.x()) + (y - pos.y())*(y - pos.y());
            if (d_sq < min_d_sq) {
                min_d_sq = d_sq;
                t_coarse = t;
            }
        }

        double t_fine = t_coarse;
        const int max_iters = 8;
        
        for (int i = 0; i < max_iters; ++i) {
            double t = t_fine;
            double t2 = t * t;
            
            Eigen::Vector2d c_t(poly.a0_x + poly.a1_x*t + poly.a2_x*t2 + poly.a3_x*t2*t,
                                poly.a0_y + poly.a1_y*t + poly.a2_y*t2 + poly.a3_y*t2*t);
            Eigen::Vector2d c_d1(poly.a1_x + 2*poly.a2_x*t + 3*poly.a3_x*t2,
                                 poly.a1_y + 2*poly.a2_y*t + 3*poly.a3_y*t2);
            Eigen::Vector2d c_d2(2*poly.a2_x + 6*poly.a3_x*t,
                                 2*poly.a2_y + 6*poly.a3_y*t);

            Eigen::Vector2d err = c_t - pos;
            
            double g = err.dot(c_d1);
            double g_prime = c_d1.dot(c_d1) + err.dot(c_d2);

            if (std::abs(g_prime) < 1e-6) break;

            double delta_t = g / g_prime;
            t_fine -= delta_t;
            
            t_fine = std::max(0.0, std::min(t_fine, T));

            if (std::abs(delta_t) < 1e-4) break;
        }

        double t2 = t_fine * t_fine;
        double t3 = t2 * t_fine;
        double x_final = poly.a0_x + poly.a1_x*t_fine + poly.a2_x*t2 + poly.a3_x*t3;
        double y_final = poly.a0_y + poly.a1_y*t_fine + poly.a2_y*t2 + poly.a3_y*t3;
        double final_dist_sq = (x_final - pos.x())*(x_final - pos.x()) + (y_final - pos.y())*(y_final - pos.y());

        if (final_dist_sq < best_res.dist_sq) {
            best_res.dist_sq = final_dist_sq;
            best_res.seg_idx = seg_idx;
            best_res.t = t_fine;
            best_res.pos << x_final, y_final;
        }
    }

    void project::GetCubicExtrema(double a0, double a1, double a2, double a3, double T, std::vector<double>& extremas) const {
        extremas.push_back(0.0);
        extremas.push_back(T);

        double A = 3.0 * a3;
        double B = 2.0 * a2;
        double C = a1;

        if (std::abs(A) > 1e-6) {
            double delta = B * B - 4.0 * A * C;
            if (delta >= 0) {
                double sqrt_delta = std::sqrt(delta);
                double t1 = (-B + sqrt_delta) / (2.0 * A);
                double t2 = (-B - sqrt_delta) / (2.0 * A);
                if (t1 > 0.0 && t1 < T) extremas.push_back(t1);
                if (t2 > 0.0 && t2 < T) extremas.push_back(t2);
            }
        } else if (std::abs(B) > 1e-6) {
            double t1 = -C / B;
            if (t1 > 0.0 && t1 < T) extremas.push_back(t1);
        }
    }

    AABB project::ComputeSegmentAABB(const Order3_Polynomial& poly, double T) const {
        AABB aabb;
        std::vector<double> t_candidates_x, t_candidates_y;
        
        GetCubicExtrema(poly.a0_x, poly.a1_x, poly.a2_x, poly.a3_x, T, t_candidates_x);
        GetCubicExtrema(poly.a0_y, poly.a1_y, poly.a2_y, poly.a3_y, T, t_candidates_y);

        for (double t : t_candidates_x) {
            double x = poly.a0_x + poly.a1_x*t + poly.a2_x*t*t + poly.a3_x*t*t*t;
            aabb.min_pt.x() = std::min(aabb.min_pt.x(), x);
            aabb.max_pt.x() = std::max(aabb.max_pt.x(), x);
        }
        for (double t : t_candidates_y) {
            double y = poly.a0_y + poly.a1_y*t + poly.a2_y*t*t + poly.a3_y*t*t*t;
            aabb.min_pt.y() = std::min(aabb.min_pt.y(), y);
            aabb.max_pt.y() = std::max(aabb.max_pt.y(), y);
        }
        return aabb;
    }

} // namespace planner_2d