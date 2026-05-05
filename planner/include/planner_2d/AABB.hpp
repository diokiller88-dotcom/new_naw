#pragma once

#include <Eigen/Dense>
#include <limits>
#include <algorithm>

namespace planner_2d {

    struct ProjectionResult {
        int seg_idx;            // 最近点所在的轨迹段索引 (-1 表示未找到)
        double t;               // 该段内的局部时间 t
        Eigen::Vector2d pos;    // 最近点的绝对物理坐标
        double dist_sq;         // 车体到该点的距离平方

        ProjectionResult() : seg_idx(-1), t(0.0), pos(0, 0), dist_sq(std::numeric_limits<double>::max()) {}
    };

    ///AABB树用于重规划时候获取当前位姿与目标轨迹之间的映射点，AABB的叶子节点对应的是MINCO的一段轨迹
  
    struct AABB {
        Eigen::Vector2d min_pt;
        Eigen::Vector2d max_pt;

        AABB() {
            min_pt.setConstant(std::numeric_limits<double>::max());
            max_pt.setConstant(std::numeric_limits<double>::lowest());
        }
        Eigen::Vector2d GetCentroid() const {
            return (min_pt + max_pt) * 0.5;
        }

        // 合并两个 AABB
        static AABB Merge(const AABB& a, const AABB& b) {
            AABB res;
            res.min_pt = a.min_pt.cwiseMin(b.min_pt);
            res.max_pt = a.max_pt.cwiseMax(b.max_pt);
            return res;
        }

        // 计算点到 AABB 的最短距离平方 
        double SquaredDistanceTo(const Eigen::Vector2d& pt) const {
            double dx = std::max({ 0.0, min_pt.x() - pt.x(), pt.x() - max_pt.x() });
            double dy = std::max({ 0.0, min_pt.y() - pt.y(), pt.y() - max_pt.y() });
            return dx * dx + dy * dy;
        }
    };

    struct AABBNode {
        AABB aabb;
        int left_child = -1;
        int right_child = -1;
        int seg_idx = -1;     // 如果是叶子节点，记录轨迹段索引；否则为 -1

        bool IsLeaf() const { return left_child == -1 && right_child == -1; }
    };


    
    // 临时结构，用于构建树时的排序
    struct SegmentInfo {
        int id;
        AABB aabb;
        Eigen::Vector2d centroid;
    };
}