#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>
#include "lbfgs.hpp"
#include "AABB.hpp"

namespace planner_2d {
    class project {
    public:
        project() = default;
        ~project() = default;

        void BuildAABBTree(const minco& traj);

        ProjectionResult FindClosestPoint(const Eigen::Vector2d& vehicle_pos,
                                          double warm_start_dist_sq = std::numeric_limits<double>::max()) const;

    private:
        std::vector<AABBNode> m_TreeNodes;
        std::vector<Order3_Polynomial> m_Path;
        Eigen::VectorXd m_Times;
        int m_RootNodeIdx = -1;

        AABB ComputeSegmentAABB(const Order3_Polynomial& poly, double T) const;
        void GetCubicExtrema(double a0, double a1, double a2, double a3, double T, std::vector<double>& extremas) const;
        int BuildTreeRecursive(std::vector<SegmentInfo>& seg_infos, int start, int end);
        void QueryTreeRecursive(int node_idx, const Eigen::Vector2d& pos, ProjectionResult& best_res) const;
        void EvaluateExactProjection(int seg_idx, const Eigen::Vector2d& pos, ProjectionResult& best_res) const;
    };

} // namespace planner_2d