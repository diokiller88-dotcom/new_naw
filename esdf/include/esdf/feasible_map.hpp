#pragma once
#include "sliding_map.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <vector>
#include <string>

namespace esdf_map {

constexpr double minZ = -1.0;
constexpr double maxZ = 5.0;
constexpr double minX = -20.0;
constexpr double minY = -20.0;
constexpr double leafX = 0.05;
constexpr double leafY = 0.05;
constexpr double leafZ = (maxZ - minZ) / 128.0;

constexpr double obsLowerHeight = 0.0;     
constexpr double obsUpperHeight = 0.40;    
constexpr double maxDownStepHeight = 0.30;   
constexpr double densityRatioThreshold = 0.9;
constexpr double obsHeightThreshold = 0.30;   

class feasible_map {
public:
    feasible_map();

    void Init(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void InitFromPCD(const std::string& pcd_path);
    void InitMap(int x_size = 800, int y_size = 800);
    
    void PreMapDownSample();
    grip_state CheckZ(int x, int y);
    void CheckGradient();
    void CheckOverhang();
    
    bool Update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &global_cloud, double sensor_tx, double sensor_ty);

    inline const std::vector<double>& getGlobalESDF() const { return m_GlobalESDF; }
    inline const std::vector<grip_info>& getGlobalMap() const { return m_Map; }
    
    inline int getMapWidth() const  { return m_MapLenX; }
    inline int getMapHeight() const { return m_MapWeightY; }

private:
    sliding_map m_SW;                    
    std::vector<grip_info> m_Map;        
    std::vector<double> m_GlobalESDF;    
    
    int m_MapLenX, m_MapWeightY;
    bool m_IsInitMap;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_OriginPointCloud;
};

} // namespace esdf_map