#pragma once
#include <vector>

namespace esdf_map {

class esdf_map {
public:
    esdf_map();
    bool Init(const std::vector<int>& map_, int x_, int y_);
    void SetSurfMap();
    void ComputeEDT();

    std::vector<int> m_GripMap;
    std::vector<int> m_SurfMap;
    std::vector<double> m_ESDFMap;
    int m_MapLenX;
    int m_MapWeightY;
    bool m_IsSetESDF;
};

} // namespace esdf_map