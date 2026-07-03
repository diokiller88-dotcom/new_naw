#include "planner_2d/a_star.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <spdlog/spdlog.h>

namespace planner_2d {

struct AStarNode {
    int idx;
    double f;
    bool operator>(const AStarNode& other) const { return f > other.f; }
};

bool a_star::InitMap(const std::vector<int>& occupancy, int width, int height) {
    if (occupancy.empty() || width <= 0 || height <= 0) return false;
    m_occupancy = occupancy; m_width = width; m_height = height; m_use_esdf = false;
    int total_size = width * height;
    m_g_score.resize(total_size); m_parent.resize(total_size); m_closed.resize(total_size);
    return true;
}

bool a_star::InitMapWithESDF(const std::vector<int>& occupancy, const std::vector<double>& esdf, int width, int height) {
    if (occupancy.empty() || esdf.empty() || width <= 0 || height <= 0) return false;
    m_occupancy = occupancy; m_esdf = esdf; m_width = width; m_height = height; m_use_esdf = true;
    int total_size = width * height;
    m_g_score.resize(total_size); m_parent.resize(total_size); m_closed.resize(total_size);
    return true;
}

bool a_star::UpdateMapPatchWithESDF(const std::vector<int>& occupancy,
                                    const std::vector<double>& esdf,
                                    int local_w, int local_h, int start_x, int start_y) {
    if (m_occupancy.empty() || !m_use_esdf) return false;
    if (occupancy.empty() || esdf.empty() || local_w <= 0 || local_h <= 0) return false;
    if (static_cast<int>(occupancy.size()) != local_w * local_h) return false;
    if (static_cast<int>(esdf.size()) != local_w * local_h) return false;

    for (int y = 0; y < local_h; ++y) {
        for (int x = 0; x < local_w; ++x) {
            int gx = start_x + x;
            int gy = start_y + y;
            if (gx < 0 || gx >= m_width || gy < 0 || gy >= m_height) continue;
            int global_idx = gy * m_width + gx;
            int local_idx = y * local_w + x;
            m_occupancy[global_idx] = occupancy[local_idx];
            m_esdf[global_idx] = esdf[local_idx];
        }
    }
    return true;
}

bool a_star::SetStartGoal(int sx, int sy, int gx, int gy) {
    if (sx < 0 || sx >= m_width || sy < 0 || sy >= m_height || gx < 0 || gx >= m_width || gy < 0 || gy >= m_height) return false;
    if (!IsFree(sx, sy)) return false;
    if (!IsFree(gx, gy)) return false;
    m_sx = sx; m_sy = sy; m_gx = gx; m_gy = gy; m_path.clear();
    return true;
}

double a_star::Heuristic(int x, int y) const { return std::hypot(x - m_gx, y - m_gy); }

double a_star::GetCost(int x, int y) const {
    double cost = 1.0;
    if (m_use_esdf) {
        int idx = y * m_width + x;
        double dist = m_esdf[idx];
        if (dist < 0.2) cost += 1000.0;
        else if (dist < 0.5) cost += 10.0;
        else cost += 1.0 / dist;
    }
    return cost;
}

bool a_star::IsFree(int x, int y) const {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) return false;
    if (m_occupancy[y * m_width + x] == 1) return false;
    if (m_use_esdf && m_esdf[y * m_width + x] < 0.05) return false;
    return true;
}

bool a_star::FindPath() {
    if (m_width == 0 || m_height == 0) return false;

    std::fill(m_g_score.begin(), m_g_score.end(), std::numeric_limits<double>::max());
    std::fill(m_parent.begin(), m_parent.end(), -1);
    std::fill(m_closed.begin(), m_closed.end(), false);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    auto get_idx = [this](int x, int y) { return y * m_width + x; };

    int start_idx = get_idx(m_sx, m_sy);
    int goal_idx = get_idx(m_gx, m_gy);

    m_g_score[start_idx] = 0.0;
    open.push({start_idx, Heuristic(m_sx, m_sy)});

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double move_cost[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};

    while (!open.empty()) {
        AStarNode current = open.top(); open.pop();
        int c_idx = current.idx;

        if (m_closed[c_idx]) continue; 
        m_closed[c_idx] = true;

        if (c_idx == goal_idx) {
            std::vector<Eigen::Vector2d> path;
            int curr = goal_idx;
            while (curr != -1) {
                path.emplace_back(curr % m_width, curr / m_width);
                curr = m_parent[curr];
            }
            std::reverse(path.begin(), path.end());
            m_path = path;
            return true;
        }

        int cx = c_idx % m_width, cy = c_idx / m_width;
        for (int d = 0; d < 8; ++d) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (dx[d] != 0 && dy[d] != 0) {
                if (!IsFree(cx + dx[d], cy) || !IsFree(cx, cy + dy[d])) continue;
            }
            if (!IsFree(nx, ny)) continue;
            int n_idx = get_idx(nx, ny);
            if (m_closed[n_idx]) continue;

            double new_g = m_g_score[c_idx] + move_cost[d] * GetCost(nx, ny);
            if (new_g < m_g_score[n_idx]) {
                m_g_score[n_idx] = new_g; m_parent[n_idx] = c_idx;
                open.push({n_idx, new_g + Heuristic(nx, ny)});
            }
        }
    }
    spdlog::warn("[a_star] No path found");
    return false;
}

} // namespace planner_2d
