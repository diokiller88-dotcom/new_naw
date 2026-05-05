#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <cstdint> 

namespace planner_2d {

    constexpr double jps_esdf_safe_distance = 0.15; 
    constexpr double jps_f_tolerance        = 1e-6; 

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
        // 纯正欧氏距离启发式 + 1.001 微偏置
        inline double Heuristic(int x, int y) const { 
            double dx = static_cast<double>(std::abs(x - m_Target.x));
            double dy = static_cast<double>(std::abs(y - m_Target.y));
            return 1.001 * std::sqrt(dx * dx + dy * dy); 
        }
        
        inline double FastDist(int x1, int y1, int x2, int y2) const {
            double dx = static_cast<double>(x1 - x2);
            double dy = static_cast<double>(y1 - y2);
            return std::sqrt(dx * dx + dy * dy);
        }

        bool StepInLine(int& x_, int& y_, const int& direction_) const;  
        int  StepInDiagonal(int& x_, int& y_, int& another_x_, int& another_y_, const int& direction_) const;  

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

        Jump_Point m_Source;
        Jump_Point m_Target;

        int m_MapLenX; int m_MapWeightY;
        bool m_IsInitMap; bool m_IsInitESDFMap; bool m_IsInitPoint;
    };

} // namespace planner_2d