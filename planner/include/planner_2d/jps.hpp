#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <array>
#include <Eigen/Dense>
#include <cstdint> 

namespace planner_2d {

    constexpr double jps_esdf_safe_distance = 0.15; 
    constexpr double jps_f_tolerance        = 1e-6; 
    constexpr double jps_diag_cost          = 1.4142135623730951;

    struct Jump_Point {
        int x; int y; int pointIdx; double f;               
        Jump_Point() : x(0), y(0), pointIdx(-1), f(0.0) {}
        Jump_Point(int x_, int y_, int idx, double f_) : x(x_), y(y_), pointIdx(idx), f(f_) {}
        bool operator<(const Jump_Point& other) const { return f > other.f; }
    };

    class jps {
    public:
        jps();

        bool InitMap(const std::vector<int>& map_, const int& len_, const int& weight_);
        bool InitMapWithESDF(const std::vector<int>& grip_map_, const std::vector<double>& esdf_map_, const int& len_, const int& weight_);
        bool UpdateMapPatchWithESDF(const std::vector<int>& grip_map_, const std::vector<double>& esdf_map_,
                                    int local_w, int local_h, int start_x, int start_y);
        bool InitPoint(const int& source_x_, const int& source_y_, const int& target_x_, const int& target_y_);

        inline bool IsFreeGrip(const int& x_, const int& y_) const {
            if (x_ < 0 || x_ >= m_MapLenX || y_ < 0 || y_ >= m_MapWeightY) return false;
            return m_FastMap[y_ * m_MapLenX + x_] == 1;
        }

        bool IsNonStop(const Jump_Point& point_) const;
        bool IsNonStop(const Eigen::Vector2d& sourcepoint_, const Eigen::Vector2d& targetpoint_) const;
        bool SetPath();

        const std::vector<int>& GetPath() const { return m_Path; }
        std::vector<Eigen::Vector2d> GetEigenPath() const;
        double PathDist();

    private:
        struct SearchDirection {
            int dx;
            int dy;
        };

        inline double Heuristic(int x, int y) const { 
            int dx = std::abs(x - m_Target.x);
            int dy = std::abs(y - m_Target.y);
            int diag = std::min(dx, dy);
            int straight = std::max(dx, dy) - diag;
            return 1.001 * (jps_diag_cost * static_cast<double>(diag) + static_cast<double>(straight));
        }
        
        inline double FastDist(int x1, int y1, int x2, int y2) const {
            int dx = std::abs(x1 - x2);
            int dy = std::abs(y1 - y2);
            int diag = std::min(dx, dy);
            int straight = std::max(dx, dy) - diag;
            return jps_diag_cost * static_cast<double>(diag) + static_cast<double>(straight);
        }

        bool HasForcedNeighborStraight(int x_, int y_, int dx_, int dy_) const;
        bool HasForcedNeighborDiagonal(int x_, int y_, int dx_, int dy_) const;
        int CollectSuccessorDirections(int x_, int y_, int parent_idx_, std::array<SearchDirection, 8>& dirs_) const;
        int DirectionToCacheSlot(int dx_, int dy_) const;
        bool JumpStraight(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_);
        bool JumpDiagonal(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_);
        bool Jump(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_);

        inline double GetG(int idx) const { return (m_EpochMap[idx] == m_SearchEpoch) ? m_ValueMap[idx] : 1e9; }
        inline void SetG(int idx, double g, int parent) {
            m_ValueMap[idx] = g; m_ParentMap[idx] = parent; m_EpochMap[idx] = m_SearchEpoch;
        }

        std::vector<uint8_t> m_FastMap;   
        uint16_t m_SearchEpoch;
        std::vector<uint16_t> m_EpochMap;
        std::vector<uint16_t> m_ClosedEpoch;
        
        std::vector<int>    m_GripMap;
        std::vector<double> m_ESDFMap;
        std::priority_queue<Jump_Point> m_Queue;
        std::vector<int>    m_Path;
        std::vector<double> m_ValueMap;    
        std::vector<int>    m_ParentMap;   
        std::array<std::vector<int>, 4> m_StraightJumpResult;
        std::array<std::vector<uint16_t>, 4> m_StraightJumpEpoch;

        Jump_Point m_Source;
        Jump_Point m_Target;

        int m_MapLenX; int m_MapWeightY;
        bool m_IsInitMap; bool m_IsInitESDFMap; bool m_IsInitPoint;
    };

} // namespace planner_2d
