#pragma once
#include <vector>
#include <cstdint>
#include <utility>
#include "esdf_map.hpp"

namespace esdf_map {

constexpr int windowX = 20;
constexpr int windowY = 20;
constexpr int local_W = 2 * windowX + 1;
constexpr int local_H = 2 * windowY + 1;

enum class grip_state {
    unknown,
    occupy,
    free,
    acchole,
    possiblehole,
    temp_occupy
};

struct grip_info {
    grip_state state = grip_state::unknown;
    int maxz = -1;
    int minz = -1;
    int cont = 0;
    int grip_cont = 0;
    int groundz = -1;
    uint32_t z[4] = {0};
};

class sliding_map {
public:
    sliding_map();
    void setCenter(int global_cx, int global_cy, const std::vector<grip_info>& global_map, int map_w, int map_h);
    void raycastAndUpdate(int sensor_x_local, int sensor_y_local, const std::vector<std::pair<int, int>>& local_hits);
    void updateLocalESDF();

    const std::vector<int>& getLocalOccupancy() const { return m_LocalOccupancy; }
    const std::vector<double>& getLocalESDF() const { return m_LocalESDFAlgo.m_ESDFMap; }
    
    int getHitCount(int lx, int ly) const;
    int getMissCount(int lx, int ly) const;

    bool localToGlobal(int lx, int ly, int& gx, int& gy) const;
    bool globalToLocal(int gx, int gy, int& lx, int& ly) const;

private:
    void bresenhamRay(int x0, int y0, int x1, int y1);

    int m_CenterX; 
    int m_CenterY; 

    std::vector<int> m_HitCount;   // 占据计数器
    std::vector<int> m_MissCount;  // 空闲穿透计数器
    std::vector<int> m_LocalOccupancy;

    esdf_map m_LocalESDFAlgo; 
};

} // namespace esdf_map