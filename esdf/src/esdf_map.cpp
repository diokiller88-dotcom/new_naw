#include "esdf/esdf_map.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace esdf_map {

esdf_map::esdf_map() : m_MapLenX(0), m_MapWeightY(0), m_IsSetESDF(false) {}

bool esdf_map::Init(const std::vector<int>& map_, int x_, int y_) {
    if (map_.empty() || x_ < 1 || y_ < 1 || x_ * y_ != static_cast<int>(map_.size())) {
        spdlog::warn("ESDF Init: invalid map size");
        return false;
    }
    m_GripMap = map_;
    m_MapLenX = x_;
    m_MapWeightY = y_;
    return true;
}
///根据占据栅格情况，判断墙壁边界，墙壁内还是墙壁外
void esdf_map::SetSurfMap() {
    m_SurfMap = m_GripMap;
    m_ESDFMap.assign(m_GripMap.size(), 1e9); 
    
    for (int i = 1; i < m_MapWeightY - 1; i++) {
        for (int j = 1; j < m_MapLenX - 1; j++) {
            int idx = i * m_MapLenX + j;
            if (m_GripMap[idx] == 1) {
                m_ESDFMap[idx] = 0.0;
                if (m_GripMap[idx+1] == 1 && m_GripMap[idx-1] == 1 &&
                    m_GripMap[idx+m_MapLenX] == 1 && m_GripMap[idx-m_MapLenX] == 1) {
                    m_SurfMap[idx] = -1;
                }
            }
        }
    }
}

void esdf_map::ComputeEDT() {
    const double INF = 1e9;
    const int W = m_MapLenX;
    const int H = m_MapWeightY;
    std::vector<double> g(W * H);
    std::vector<int> v(std::max(W, H));
    std::vector<double> z(std::max(W, H) + 1);
    ///下包络法，通过栈获得二项式的极小值的坐标点，然后通过极小值点获取一维每个点的欧式距离平方
    for (int y = 0; y < H; ++y) {
        int base = y * W;
        int k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
        for (int q = 1; q < W; ++q) {
            auto get_s = [&]() {
                return ((m_ESDFMap[base+q] + q*q) - (m_ESDFMap[base+v[k]] + v[k]*v[k])) / (2.0*(q-v[k]));
            };
            double s = get_s();
            while (s <= z[k]) {
                --k;
                s = get_s();
            }
            v[++k] = q; z[k] = s; z[k+1] = INF;
        }
        for (int q = 0, ki = 0; q < W; ++q) {
            while (z[ki+1] < q) ++ki;
            g[base+q] = std::pow(q - v[ki], 2) + m_ESDFMap[base + v[ki]];
        }
    }
    ///////基于之前一维的获取二维的欧式距离的平方
    for (int x = 0; x < W; ++x) {
        int k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
        for (int q = 1; q < H; ++q) {
            auto get_s = [&]() {
                return ((g[q*W+x] + q*q) - (g[v[k]*W+x] + v[k]*v[k])) / (2.0*(q-v[k]));
            };
            double s = get_s();
            while (s <= z[k]) {
                --k;
                s = get_s();
            }
            v[++k] = q; z[k] = s; z[k+1] = INF;
        }
        for (int q = 0, ki = 0; q < H; ++q) {
            while (z[ki+1] < q) ++ki;
            int idx = q * W + x;
            m_ESDFMap[idx] = std::pow(q - v[ki], 2) + g[v[ki]*W+x];
            if (m_SurfMap[idx] == -1) {
                m_ESDFMap[idx] = -m_ESDFMap[idx];
            }
        }
    }
}

} // namespace esdf_map