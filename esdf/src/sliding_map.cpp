#include "esdf/sliding_map.hpp"
#include <cmath>
#include <algorithm>

namespace esdf_map {

sliding_map::sliding_map() : m_CenterX(0), m_CenterY(0) {
    int size = local_W * local_H;
    m_HitCount.assign(size, 0);
    m_MissCount.assign(size, 0);
    m_LocalOccupancy.assign(size, 0);
}

void sliding_map::setCenter(int global_cx, int global_cy, const std::vector<grip_info>& global_map, int map_w, int map_h) {
    int dx = global_cx - m_CenterX;
    int dy = global_cy - m_CenterY;
    
    if (dx != 0 || dy != 0) {
        std::vector<int> new_hit(local_W * local_H, 0);
        std::vector<int> new_miss(local_W * local_H, 0);
        
        for (int y = 0; y < local_H; ++y) {
            for (int x = 0; x < local_W; ++x) {
                int old_x = x + dx;
                int old_y = y + dy;
                int idx = y * local_W + x;
                
                if (old_x >= 0 && old_x < local_W && old_y >= 0 && old_y < local_H) {
                    new_hit[idx] = m_HitCount[old_y * local_W + old_x];
                    new_miss[idx] = m_MissCount[old_y * local_W + old_x];
                } else {
                    int gx = global_cx - windowX + x;
                    int gy = global_cy - windowY + y;
                    if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                        auto state = global_map[gy * map_w + gx].state;
                        if (state == grip_state::occupy) new_hit[idx] = 3;
                        else if (state == grip_state::free) new_miss[idx] = 3;
                    }
                }
            }
        }
        m_HitCount = new_hit;
        m_MissCount = new_miss;
        m_CenterX = global_cx;
        m_CenterY = global_cy;
    }
}

bool sliding_map::localToGlobal(int lx, int ly, int& gx, int& gy) const {
    gx = m_CenterX - windowX + lx;
    gy = m_CenterY - windowY + ly;
    return true;
}

bool sliding_map::globalToLocal(int gx, int gy, int& lx, int& ly) const {
    lx = gx - m_CenterX + windowX;
    ly = gy - m_CenterY + windowY;
    return (lx >= 0 && lx < local_W && ly >= 0 && ly < local_H);
}

int sliding_map::getHitCount(int lx, int ly) const { return m_HitCount[ly * local_W + lx]; }
int sliding_map::getMissCount(int lx, int ly) const { return m_MissCount[ly * local_W + lx]; }

void sliding_map::bresenhamRay(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        if (x0 >= 0 && x0 < local_W && y0 >= 0 && y0 < local_H) {
            int idx = y0 * local_W + x0;
            if (x0 == x1 && y0 == y1) {
                m_HitCount[idx] = std::min(m_HitCount[idx] + 2, 20);
                m_MissCount[idx] = std::max(m_MissCount[idx] - 1, 0);
                break;
            } else {
                m_MissCount[idx] = std::min(m_MissCount[idx] + 1, 20);
                m_HitCount[idx] = std::max(m_HitCount[idx] - 1, 0);
            }
        } else {
            break;
        }
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void sliding_map::raycastAndUpdate(int sensor_x_local, int sensor_y_local, const std::vector<std::pair<int, int>>& local_hits) {
    for (const auto& hit : local_hits) {
        bresenhamRay(sensor_x_local, sensor_y_local, hit.first, hit.second);
    }
}

void sliding_map::updateLocalESDF() {
    int size = local_W * local_H;
    for (int i = 0; i < size; ++i) {
        m_LocalOccupancy[i] = (m_HitCount[i] > m_MissCount[i] && m_HitCount[i] > 1) ? 1 : 0;
    }
    
    m_LocalESDFAlgo.Init(m_LocalOccupancy, local_W, local_H);
    m_LocalESDFAlgo.SetSurfMap();
    m_LocalESDFAlgo.ComputeEDT();
}

} // namespace esdf_map