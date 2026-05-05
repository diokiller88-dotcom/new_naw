#include "planner_2d/jps.hpp"
#include <algorithm>
#include <limits>
#include <spdlog/spdlog.h>

namespace planner_2d {

    jps::jps() : m_MapLenX(0), m_MapWeightY(0), m_IsInitMap(false), m_IsInitESDFMap(false), m_IsInitPoint(false), m_SearchEpoch(0) {}

    bool jps::InitMap(const std::vector<int>& map_, const int& len_, const int& weight_) {
        m_IsInitMap = false; m_IsInitESDFMap = false;
        if (map_.empty() || len_ < 1 || weight_ < 1) return false;
        m_GripMap = map_; m_MapLenX = len_; m_MapWeightY = weight_;
        
        int size = len_ * weight_;
        if (m_FastMap.size() != static_cast<size_t>(size)) {
            m_FastMap.resize(size); m_EpochMap.assign(size, 0); m_ClosedEpoch.assign(size, 0);
            m_ValueMap.resize(size); m_ParentMap.resize(size); m_SearchEpoch = 0;
        }
        std::fill(m_FastMap.begin(), m_FastMap.end(), 1);
        for (int i = 0; i < size; ++i) { if (m_GripMap[i] != 0) m_FastMap[i] = 0; }
        m_IsInitMap = true; m_IsInitPoint = false;
        return true;
    }

    bool jps::InitMapWithESDF(const std::vector<int>& grip_map_, const std::vector<double>& esdf_map_, const int& len_, const int& weight_) {
        m_IsInitMap = false; m_IsInitESDFMap = false;
        if (grip_map_.empty() || len_ < 1 || weight_ < 1) return false;
        m_GripMap = grip_map_; m_ESDFMap = esdf_map_;
        m_MapLenX = len_; m_MapWeightY = weight_;

        int size = len_ * weight_;
        if (m_FastMap.size() != static_cast<size_t>(size)) {
            m_FastMap.resize(size); m_EpochMap.assign(size, 0); m_ClosedEpoch.assign(size, 0);
            m_ValueMap.resize(size); m_ParentMap.resize(size); m_SearchEpoch = 0;
        }
        std::fill(m_FastMap.begin(), m_FastMap.end(), 1);
        for (int i = 0; i < size; ++i) {
            if (m_GripMap[i] != 0 || m_ESDFMap[i] < jps_esdf_safe_distance) m_FastMap[i] = 0;
        }
        m_IsInitMap = true; m_IsInitESDFMap = true; m_IsInitPoint = false;
        return true;
    }

    bool jps::InitPoint(const int& source_x_, const int& source_y_, const int& target_x_, const int& target_y_) {
        m_IsInitPoint = false;
        if (!m_IsInitMap) return false;
        if (source_x_ < 0 || source_y_ < 0 || target_x_ < 0 || target_y_ < 0) return false;
        if (source_x_ >= m_MapLenX || source_y_ >= m_MapWeightY || target_x_ >= m_MapLenX || target_y_ >= m_MapWeightY) return false;

        int target_idx = target_y_ * m_MapLenX + target_x_;
        if (!IsFreeGrip(target_x_, target_y_)) return false;
        m_Target = Jump_Point(target_x_, target_y_, target_idx, 0.0);

        int source_idx = source_y_ * m_MapLenX + source_x_;
        if (!IsFreeGrip(source_x_, source_y_)) return false;

        m_SearchEpoch++;
        if (m_SearchEpoch >= 65000) {
            std::fill(m_EpochMap.begin(), m_EpochMap.end(), 0);
            std::fill(m_ClosedEpoch.begin(), m_ClosedEpoch.end(), 0);
            m_SearchEpoch = 1;
        }

        SetG(source_idx, 0.0, -1);
        double h_source = Heuristic(source_x_, source_y_);
        m_Source = Jump_Point(source_x_, source_y_, source_idx, h_source);
        m_IsInitPoint = true;
        return true;
    }

    bool jps::IsNonStop(const Jump_Point& point_) const {
        return IsNonStop(Eigen::Vector2d(point_.x, point_.y), Eigen::Vector2d(m_Target.x, m_Target.y));
    }

    bool jps::IsNonStop(const Eigen::Vector2d& sourcepoint_, const Eigen::Vector2d& targetpoint_) const {
        int sx_ = static_cast<int>(sourcepoint_.x()), sy_ = static_cast<int>(sourcepoint_.y());
        int tx_ = static_cast<int>(targetpoint_.x()), ty_ = static_cast<int>(targetpoint_.y());
        int temp_dx = std::abs(tx_ - sx_), temp_dy = std::abs(ty_ - sy_);
        int ux = (tx_ > sx_) ? 1 : -1, uy = (ty_ > sy_) ? 1 : -1;
        int temp_x0 = sx_, temp_y0 = sy_;
        const uint8_t* pMap = m_FastMap.data();

        if (temp_dx > temp_dy) {
            int e = -temp_dx;
            for (int i = 0; i < temp_dx; ++i) {
                temp_x0 += ux; e += 2 * temp_dy;
                if (e >= 0) { temp_y0 += uy; e -= 2 * temp_dx; }
                if (temp_x0 == tx_ && temp_y0 == ty_) return true;
                if (temp_x0 < 0 || temp_x0 >= m_MapLenX || temp_y0 < 0 || temp_y0 >= m_MapWeightY) return false;
                if (pMap[temp_y0 * m_MapLenX + temp_x0] == 0) return false;
            }
        } else {
            int e = -temp_dy;
            for (int i = 0; i < temp_dy; ++i) {
                temp_y0 += uy; e += 2 * temp_dx;
                if (e >= 0) { temp_x0 += ux; e -= 2 * temp_dy; }
                if (temp_x0 == tx_ && temp_y0 == ty_) return true;
                if (temp_x0 < 0 || temp_x0 >= m_MapLenX || temp_y0 < 0 || temp_y0 >= m_MapWeightY) return false;
                if (pMap[temp_y0 * m_MapLenX + temp_x0] == 0) return false;
            }
        }
        return true;
    }

    bool jps::StepInLine(int& x_, int& y_, const int& direction_) const {
        int ux = 0, uy = 0, step_idx = 0;
        if (direction_ == 0) { ux = 1; step_idx = 1; }
        else if (direction_ == 1) { uy = 1; step_idx = m_MapLenX; }
        else if (direction_ == 2) { ux = -1; step_idx = -1; }
        else if (direction_ == 3) { uy = -1; step_idx = -m_MapLenX; }
        else return false;

        int cur_x = x_ + ux, cur_y = y_ + uy;
        if (cur_x < 0 || cur_x >= m_MapLenX || cur_y < 0 || cur_y >= m_MapWeightY) return false;

        const uint8_t* pMap = m_FastMap.data();
        int idx = cur_y * m_MapLenX + cur_x;
        const int map_len = m_MapLenX;
        
        if (ux != 0) { 
            while (cur_x >= 0 && cur_x < map_len) {
                if (pMap[idx] == 0) return false;
                if (cur_x == m_Target.x && cur_y == m_Target.y) { x_ = cur_x; y_ = cur_y; return true; }
                if (cur_y > 0 && pMap[idx - map_len] == 0 && pMap[idx - map_len + ux] == 1) { x_ = cur_x; y_ = cur_y; return true; }
                if (cur_y + 1 < m_MapWeightY && pMap[idx + map_len] == 0 && pMap[idx + map_len + ux] == 1) { x_ = cur_x; y_ = cur_y; return true; }
                cur_x += ux; idx += step_idx; 
            }
        } else { 
            while (cur_y >= 0 && cur_y < m_MapWeightY) {
                if (pMap[idx] == 0) return false;
                if (cur_x == m_Target.x && cur_y == m_Target.y) { x_ = cur_x; y_ = cur_y; return true; }
                if (cur_x > 0 && pMap[idx - 1] == 0 && pMap[idx - 1 + uy * map_len] == 1) { x_ = cur_x; y_ = cur_y; return true; }
                if (cur_x + 1 < map_len && pMap[idx + 1] == 0 && pMap[idx + 1 + uy * map_len] == 1) { x_ = cur_x; y_ = cur_y; return true; }
                cur_y += uy; idx += step_idx; 
            }
        }
        return false;
    }

    int jps::StepInDiagonal(int& x_, int& y_, int&, int&, const int& direction_) const {
        int ux = 0, uy = 0, dir_horiz = 0, dir_vert = 0;
        if (direction_ == 0) { ux = 1; uy = 1; dir_horiz = 0; dir_vert = 1; }
        else if (direction_ == 1) { ux = 1; uy = -1; dir_horiz = 0; dir_vert = 3; }
        else if (direction_ == 2) { ux = -1; uy = -1; dir_horiz = 2; dir_vert = 3; }
        else if (direction_ == 3) { ux = -1; uy = 1; dir_horiz = 2; dir_vert = 1; }
        else return -1;

        int cur_x = x_ + ux, cur_y = y_ + uy;
        if (cur_x < 0 || cur_x >= m_MapLenX || cur_y < 0 || cur_y >= m_MapWeightY) return -1;

        const uint8_t* pMap = m_FastMap.data();
        int step_idx = uy * m_MapLenX + ux;
        int idx = cur_y * m_MapLenX + cur_x;

        while (cur_x >= 0 && cur_x < m_MapLenX && cur_y >= 0 && cur_y < m_MapWeightY) {
            if (pMap[idx] == 0) return -1; 
            if (cur_x == m_Target.x && cur_y == m_Target.y) { x_ = cur_x; y_ = cur_y; return 1; }
            int tx = cur_x, ty = cur_y;
            if (StepInLine(tx, ty, dir_horiz)) { x_ = cur_x; y_ = cur_y; return 1; }
            tx = cur_x; ty = cur_y;
            if (StepInLine(tx, ty, dir_vert)) { x_ = cur_x; y_ = cur_y; return 1; }
            cur_x += ux; cur_y += uy; idx += step_idx;
        }
        return -1;
    }

    bool jps::SetPath() {
        if (!m_IsInitMap || !m_IsInitPoint) return false;

        m_Queue = std::priority_queue<Jump_Point>();
        m_Path.clear();

        if (IsNonStop(m_Source)) {
            m_Path.push_back(m_Source.pointIdx);
            m_Path.push_back(m_Target.pointIdx);
            return true;
        }

        m_Queue.push(m_Source);

        uint16_t* pClosedEpoch = m_ClosedEpoch.data();
        uint16_t* pEpochMap    = m_EpochMap.data();
        double* pValueMap    = m_ValueMap.data();
        int* pParentMap   = m_ParentMap.data();
        uint16_t  curEpoch     = m_SearchEpoch;

        while (!m_Queue.empty()) {
            Jump_Point current = m_Queue.top(); m_Queue.pop();

            if (pClosedEpoch[current.pointIdx] == curEpoch) continue;
            pClosedEpoch[current.pointIdx] = curEpoch;

            double current_g = (pEpochMap[current.pointIdx] == curEpoch) ? pValueMap[current.pointIdx] : 1e9;
            double current_f = current_g + Heuristic(current.x, current.y);
            if (current.f > current_f + jps_f_tolerance) continue;

            if (current.pointIdx == m_Target.pointIdx) {
                int idx = m_Target.pointIdx;
                while (idx >= 0) {
                    m_Path.push_back(idx);
                    idx = pParentMap[idx];
                }
                std::reverse(m_Path.begin(), m_Path.end());
                return true;
            }

            for (int dir = 0; dir < 4; ++dir) {
                int x = current.x, y = current.y;
                if (StepInLine(x, y, dir)) {
                    int child_idx = y * m_MapLenX + x;
                    if (pClosedEpoch[child_idx] == curEpoch) continue;

                    double new_g = current_g + FastDist(x, y, current.x, current.y);
                    double old_g = (pEpochMap[child_idx] == curEpoch) ? pValueMap[child_idx] : 1e9;

                    if (new_g < old_g) {
                        pValueMap[child_idx] = new_g;
                        pParentMap[child_idx] = current.pointIdx;
                        pEpochMap[child_idx] = curEpoch;
                        m_Queue.push(Jump_Point(x, y, child_idx, new_g + Heuristic(x, y)));
                    }
                }
            }

            for (int dir = 0; dir < 4; ++dir) {
                int x = current.x, y = current.y, another_x = 0, another_y = 0;
                int state = StepInDiagonal(x, y, another_x, another_y, dir);
                if (state == -1) continue;

                int diag_idx = y * m_MapLenX + x;
                if (pClosedEpoch[diag_idx] == curEpoch) continue;

                double new_g = current_g + FastDist(x, y, current.x, current.y);
                double old_g = (pEpochMap[diag_idx] == curEpoch) ? pValueMap[diag_idx] : 1e9;

                if (new_g < old_g) {
                    pValueMap[diag_idx] = new_g;
                    pParentMap[diag_idx] = current.pointIdx;
                    pEpochMap[diag_idx] = curEpoch;
                    m_Queue.push(Jump_Point(x, y, diag_idx, new_g + Heuristic(x, y)));
                }
            }
        }
        return false;
    }

    std::vector<Eigen::Vector2d> jps::GetEigenPath() const {
        std::vector<Eigen::Vector2d> res;
        if (m_Path.empty()) return res;
        for (int p : m_Path) res.emplace_back(p % m_MapLenX, p / m_MapLenX);
        return res;
    }

    double jps::PathDist() {
        std::vector<Eigen::Vector2d> path = GetEigenPath();
        double dist = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x() - path[i-1].x();
            double dy = path[i].y() - path[i-1].y();
            dist += std::sqrt(dx*dx + dy*dy);
        }
        return dist;
    }

} // namespace planner_2d