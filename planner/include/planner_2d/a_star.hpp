#pragma once
#include <vector>
#include <Eigen/Dense>

namespace planner_2d {

class a_star {
public:
    a_star() = default;
    
    bool InitMap(const std::vector<int>& occupancy, int width, int height);
    bool InitMapWithESDF(const std::vector<int>& occupancy,
                         const std::vector<double>& esdf,
                         int width, int height);
    bool UpdateMapPatchWithESDF(const std::vector<int>& occupancy,
                                const std::vector<double>& esdf,
                                int local_w, int local_h, int start_x, int start_y);
    bool SetStartGoal(int sx, int sy, int gx, int gy);
    bool FindPath();
    std::vector<Eigen::Vector2d> GetPath() const { return m_path; }
    bool IsFree(int x, int y) const;

private:
    std::vector<int> m_occupancy;
    std::vector<double> m_esdf;
    int m_width = 0, m_height = 0;
    bool m_use_esdf = false;
    
    int m_sx = 0, m_sy = 0, m_gx = 0, m_gy = 0;
    std::vector<Eigen::Vector2d> m_path;
    std::vector<double> m_g_score;
    std::vector<int>    m_parent;
    std::vector<bool>   m_closed;
    
    double Heuristic(int x, int y) const;
    double GetCost(int x, int y) const;
};

} // namespace planner_2d
