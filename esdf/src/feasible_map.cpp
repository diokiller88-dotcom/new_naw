#include "esdf/feasible_map.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h> 
#include <limits>
#include <cmath>
#include <algorithm>

namespace esdf_map {

feasible_map::feasible_map() 
    : m_MapLenX(0), m_MapWeightY(0), m_IsInitMap(false) {
    m_OriginPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void feasible_map::InitMap(int x_size, int y_size) {
    m_MapLenX = x_size;
    m_MapWeightY = y_size;
    m_Map.resize(m_MapLenX * m_MapWeightY);
    m_GlobalESDF.assign(m_MapLenX * m_MapWeightY, 1e9); 
    m_IsInitMap = true;
}

void feasible_map::Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::copyPointCloud(*cloud, *m_OriginPointCloud);
}

void feasible_map::InitFromPCD(const std::string& pcd_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *m_OriginPointCloud) == -1) {
        throw std::runtime_error("Cannot load offline PCD for prior map.");
    }
}

void feasible_map::PreMapDownSample() {
    if (!m_IsInitMap || m_OriginPointCloud->empty()) return;
    float current_min_z = std::numeric_limits<float>::max();
    for (const auto& pt : m_OriginPointCloud->points) {
        if (pt.z < current_min_z) current_min_z = pt.z;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : m_OriginPointCloud->points) {
        if (pt.z > current_min_z + 0.05f) {
            filtered_cloud->points.push_back(pt);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(20);            
    sor.setStddevMulThresh(1.5);
    sor.filter(*m_OriginPointCloud);

    for (auto& grip : m_Map) {
        grip.state = grip_state::unknown;
        grip.minz = -1;
        grip.maxz = -1;
        grip.groundz = -1;
        grip.z[0] = 0; grip.z[1] = 0; grip.z[2] = 0; grip.z[3] = 0;
    }

    for (const auto& pt : m_OriginPointCloud->points) {
        if (pt.x < minX || pt.x >= minX + m_MapLenX * leafX || 
            pt.y < minY || pt.y >= minY + m_MapWeightY * leafY) continue;
        if (pt.z < minZ || pt.z > maxZ) continue;

        int gx = static_cast<int>((pt.x - minX) / leafX);
        int gy = static_cast<int>((pt.y - minY) / leafY);
        int z_idx = static_cast<int>((pt.z - minZ) / leafZ);

        if (gx >= 0 && gx < m_MapLenX && gy >= 0 && gy < m_MapWeightY && z_idx >= 0 && z_idx < 128) {
            m_Map[gy * m_MapLenX + gx].z[z_idx / 32] |= (1U << (z_idx % 32));
        }
    }
    
    for (int y = 0; y < m_MapWeightY; ++y) {
        for (int x = 0; x < m_MapLenX; ++x) {
            CheckZ(x, y);
        }
    }
    
    CheckGradient();

    std::vector<int> final_occupancy(m_MapLenX * m_MapWeightY, 0);
    struct Point2D { int x, y; };
    std::vector<Point2D> current_obs;

    for (int y = 0; y < m_MapWeightY; ++y) {
        for (int x = 0; x < m_MapLenX; ++x) {
            int i = y * m_MapLenX + x;
            if (m_Map[i].state == grip_state::occupy) {
                final_occupancy[i] = 1;
                current_obs.push_back({x, y});
            }
        }
    }

    const double SEARCH_RADIUS_METERS = 0.40; 
    int max_radius_pixels = static_cast<int>(std::ceil(SEARCH_RADIUS_METERS / leafX));
    const double max_dist_sq = max_radius_pixels * max_radius_pixels;
    std::vector<std::pair<Point2D, Point2D>> dense_lines;
    
    for (const auto& pt : current_obs) {
        int x = pt.x, y = pt.y;
        for (int mdy = 0; mdy <= max_radius_pixels; ++mdy) {
            for (int mdx = -max_radius_pixels; mdx <= max_radius_pixels; ++mdx) {
                if (mdy == 0 && mdx <= 0) continue; 
                int nx = x + mdx, ny = y + mdy;
                if (nx >= 0 && nx < m_MapLenX && ny >= 0 && ny < m_MapWeightY) {
                    if (final_occupancy[ny * m_MapLenX + nx] == 1) {
                        if (mdx * mdx + mdy * mdy <= max_dist_sq) {
                            dense_lines.push_back({{x, y}, {nx, ny}});
                        }
                    }
                }
            }
        }
    }

    for (const auto& line : dense_lines) {
        int x0 = line.first.x, y0 = line.first.y;
        int x1 = line.second.x, y1 = line.second.y;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (x0 >= 0 && x0 < m_MapLenX && y0 >= 0 && y0 < m_MapWeightY) {
                int idx = y0 * m_MapLenX + x0;
                final_occupancy[idx] = 1;
                m_Map[idx].state = grip_state::occupy;
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
    esdf_map global_esdf_calc;
    global_esdf_calc.Init(final_occupancy, m_MapLenX, m_MapWeightY);
    global_esdf_calc.SetSurfMap();
    global_esdf_calc.ComputeEDT();
    
    for (int i = 0; i < m_MapLenX * m_MapWeightY; ++i) {
        m_GlobalESDF[i] = global_esdf_calc.m_ESDFMap[i];
    }
}

grip_state feasible_map::CheckZ(int cx, int cy) {
    int idx = cy * m_MapLenX + cx;
    auto& grip = m_Map[idx];

    if (grip.z[0] == 0 && grip.z[1] == 0 && grip.z[2] == 0 && grip.z[3] == 0) {
        grip.state = grip_state::unknown;
        return grip.state;
    }

    int minz = -1, maxz = -1, bit_count = 0;
    for (int j = 0; j < 4; ++j) {
        if (grip.z[j] != 0) {
            if (minz == -1) minz = j * 32 + __builtin_ctz(grip.z[j]);
            maxz = j * 32 + 31 - __builtin_clz(grip.z[j]);
            bit_count += __builtin_popcount(grip.z[j]);
        }
    }

    int height_diff = maxz - minz + 1;
    double ratio = static_cast<double>(bit_count) / height_diff;
    const int height_thresh_bits = static_cast<int>(obsHeightThreshold / leafZ);

    if (ratio >= densityRatioThreshold && height_diff >= height_thresh_bits) {
        grip.state = grip_state::occupy;
    } else {
        grip.state = grip_state::free;
        grip.groundz = maxz;
    }
    return grip.state;
}

void feasible_map::CheckGradient() {
    const int max_diff_bits = static_cast<int>(maxDownStepHeight / leafZ);
    std::vector<grip_state> next_states(m_MapLenX * m_MapWeightY);
    for(int i = 0; i < m_MapLenX * m_MapWeightY; ++i) next_states[i] = m_Map[i].state;

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

    for (int y = 0; y < m_MapWeightY; ++y) {
        for (int x = 0; x < m_MapLenX; ++x) {
            int idx = y * m_MapLenX + x;
            if (m_Map[idx].state == grip_state::free) {
                for (int k = 0; k < 8; ++k) {
                    int nx = x + dx[k], ny = y + dy[k];
                    if (nx >= 0 && nx < m_MapLenX && ny >= 0 && ny < m_MapWeightY) {
                        int n_idx = ny * m_MapLenX + nx;
                        if (m_Map[n_idx].state != grip_state::unknown && m_Map[n_idx].groundz != -1) {
                            if (std::abs(m_Map[idx].groundz - m_Map[n_idx].groundz) > max_diff_bits) {
                                next_states[idx] = grip_state::occupy;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    for(int i = 0; i < m_MapLenX * m_MapWeightY; ++i) m_Map[i].state = next_states[i];
}

void feasible_map::CheckOverhang() {
    // 已废弃
}

bool feasible_map::Update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &global_cloud, double sensor_tx, double sensor_ty) {
    if (!m_IsInitMap || !global_cloud || global_cloud->empty()) return false;

    int sensor_gx = static_cast<int>((sensor_tx - minX) / leafX);
    int sensor_gy = static_cast<int>((sensor_ty - minY) / leafY);

    m_SW.setCenter(sensor_gx, sensor_gy, m_Map, m_MapLenX, m_MapWeightY);
    
    int sensor_lx, sensor_ly;
    if (!m_SW.globalToLocal(sensor_gx, sensor_gy, sensor_lx, sensor_ly)) {
        return false;
    }

    struct CellScan { float min_z = 1e9, max_z = -1e9; int count = 0; };
    std::vector<CellScan> current_scan(local_W * local_H);

    for (const auto& pt : global_cloud->points) {
        if (pt.z < minZ || pt.z > maxZ) continue;

        int gx = static_cast<int>((pt.x - minX) / leafX);
        int gy = static_cast<int>((pt.y - minY) / leafY);
        
        int lx, ly;
        if (m_SW.globalToLocal(gx, gy, lx, ly)) {
            int idx = ly * local_W + lx;
            current_scan[idx].min_z = std::min(current_scan[idx].min_z, static_cast<float>(pt.z));
            current_scan[idx].max_z = std::max(current_scan[idx].max_z, static_cast<float>(pt.z));
            current_scan[idx].count++;
        }
    }

    double tz = 0.5; 
    std::vector<std::pair<int, int>> scan_hits;
    for (int ly = 0; ly < local_H; ++ly) {
        for (int lx = 0; lx < local_W; ++lx) {
            int idx = ly * local_W + lx;
            if (current_scan[idx].count > 0) {
                float height_diff = current_scan[idx].max_z - current_scan[idx].min_z;
                if (height_diff > obsHeightThreshold || 
                   (current_scan[idx].max_z > tz + obsLowerHeight && current_scan[idx].max_z < tz + obsUpperHeight)) {
                    scan_hits.push_back({lx, ly});
                }
            }
        }
    }

    m_SW.raycastAndUpdate(sensor_lx, sensor_ly, scan_hits);
    m_SW.updateLocalESDF();

    const auto& local_occ = m_SW.getLocalOccupancy();
    const auto& local_esdf = m_SW.getLocalESDF();
    
    for (int ly = 0; ly < local_H; ++ly) {
        for (int lx = 0; lx < local_W; ++lx) {
            int gx, gy;
            m_SW.localToGlobal(lx, ly, gx, gy);
            
            if (gx >= 0 && gx < m_MapLenX && gy >= 0 && gy < m_MapWeightY) {
                int global_idx = gy * m_MapLenX + gx;
                int local_idx = ly * local_W + lx;
                
                int hit_count = m_SW.getHitCount(lx, ly);
                int miss_count = m_SW.getMissCount(lx, ly);
                bool has_local_evidence = hit_count > 0 || miss_count > 0;

                if (local_occ[local_idx] == 1) {
                    m_Map[global_idx].state = grip_state::occupy;
                } else if (miss_count > hit_count) {
                    m_Map[global_idx].state = grip_state::free;
                }
                
                if (has_local_evidence) {
                    m_GlobalESDF[global_idx] = local_esdf[local_idx];
                }
            }
        }
    }

    return true;
}

} // namespace esdf_map
