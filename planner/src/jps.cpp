#include "planner_2d/jps.hpp"
#include <algorithm>
#include <limits>
#include <spdlog/spdlog.h>

namespace planner_2d {
    namespace {
        inline int Sign(int value) {
            return (value > 0) - (value < 0);
        }
    }

    jps::jps() : m_SearchEpoch(0), m_MapLenX(0), m_MapWeightY(0), m_IsInitMap(false), m_IsInitESDFMap(false), m_IsInitPoint(false) {}

    bool jps::InitMap(const std::vector<int>& map_, const int& len_, const int& weight_) {
        m_IsInitMap = false; m_IsInitESDFMap = false;
        if (map_.empty() || len_ < 1 || weight_ < 1) return false;
        m_GripMap = map_; m_MapLenX = len_; m_MapWeightY = weight_;
        
        int size = len_ * weight_;
        if (m_FastMap.size() != static_cast<size_t>(size)) {
            m_FastMap.resize(size); m_EpochMap.assign(size, 0); m_ClosedEpoch.assign(size, 0);
            m_ValueMap.resize(size); m_ParentMap.resize(size); m_SearchEpoch = 0;
            for (int i = 0; i < 4; ++i) {
                m_StraightJumpResult[i].assign(size, -1);
                m_StraightJumpEpoch[i].assign(size, 0);
            }
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
            for (int i = 0; i < 4; ++i) {
                m_StraightJumpResult[i].assign(size, -1);
                m_StraightJumpEpoch[i].assign(size, 0);
            }
        }
        std::fill(m_FastMap.begin(), m_FastMap.end(), 1);
        for (int i = 0; i < size; ++i) {
            if (m_GripMap[i] != 0 || m_ESDFMap[i] < jps_esdf_safe_distance) m_FastMap[i] = 0;
        }
        m_IsInitMap = true; m_IsInitESDFMap = true; m_IsInitPoint = false;
        return true;
    }

    bool jps::UpdateMapPatchWithESDF(const std::vector<int>& grip_map_, const std::vector<double>& esdf_map_,
                                     int local_w, int local_h, int start_x, int start_y) {
        if (!m_IsInitMap || !m_IsInitESDFMap) return false;
        if (grip_map_.empty() || esdf_map_.empty() || local_w <= 0 || local_h <= 0) return false;
        if (static_cast<int>(grip_map_.size()) != local_w * local_h) return false;
        if (static_cast<int>(esdf_map_.size()) != local_w * local_h) return false;

        for (int y = 0; y < local_h; ++y) {
            for (int x = 0; x < local_w; ++x) {
                int gx = start_x + x;
                int gy = start_y + y;
                if (gx < 0 || gx >= m_MapLenX || gy < 0 || gy >= m_MapWeightY) continue;
                int global_idx = gy * m_MapLenX + gx;
                int local_idx = y * local_w + x;
                m_GripMap[global_idx] = grip_map_[local_idx];
                m_ESDFMap[global_idx] = esdf_map_[local_idx];
                m_FastMap[global_idx] = (m_GripMap[global_idx] == 0 && m_ESDFMap[global_idx] >= jps_esdf_safe_distance) ? 1 : 0;
            }
        }
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
            for (int i = 0; i < 4; ++i) {
                std::fill(m_StraightJumpEpoch[i].begin(), m_StraightJumpEpoch[i].end(), 0);
            }
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
                if (temp_x0 < 0 || temp_x0 >= m_MapLenX || temp_y0 < 0 || temp_y0 >= m_MapWeightY) return false;
                if (pMap[temp_y0 * m_MapLenX + temp_x0] == 0) return false;
                if (temp_x0 == tx_ && temp_y0 == ty_) return true;
            }
        } else {
            int e = -temp_dy;
            for (int i = 0; i < temp_dy; ++i) {
                temp_y0 += uy; e += 2 * temp_dx;
                if (e >= 0) { temp_x0 += ux; e -= 2 * temp_dy; }
                if (temp_x0 < 0 || temp_x0 >= m_MapLenX || temp_y0 < 0 || temp_y0 >= m_MapWeightY) return false;
                if (pMap[temp_y0 * m_MapLenX + temp_x0] == 0) return false;
                if (temp_x0 == tx_ && temp_y0 == ty_) return true;
            }
        }
        return true;
    }

    bool jps::HasForcedNeighborStraight(int x_, int y_, int dx_, int dy_) const {
        if (dx_ != 0) {
            return (IsFreeGrip(x_, y_ + 1) == false && IsFreeGrip(x_ + dx_, y_ + 1)) ||
                   (IsFreeGrip(x_, y_ - 1) == false && IsFreeGrip(x_ + dx_, y_ - 1));
        }
        if (dy_ != 0) {
            return (IsFreeGrip(x_ + 1, y_) == false && IsFreeGrip(x_ + 1, y_ + dy_)) ||
                   (IsFreeGrip(x_ - 1, y_) == false && IsFreeGrip(x_ - 1, y_ + dy_));
        }
        return false;
    }

    bool jps::HasForcedNeighborDiagonal(int x_, int y_, int dx_, int dy_) const {
        return (IsFreeGrip(x_ - dx_, y_) == false && IsFreeGrip(x_ - dx_, y_ + dy_)) ||
               (IsFreeGrip(x_, y_ - dy_) == false && IsFreeGrip(x_ + dx_, y_ - dy_));
    }

    int jps::DirectionToCacheSlot(int dx_, int dy_) const {
        if (dx_ == 1 && dy_ == 0) return 0;
        if (dx_ == 0 && dy_ == 1) return 1;
        if (dx_ == -1 && dy_ == 0) return 2;
        if (dx_ == 0 && dy_ == -1) return 3;
        return -1;
    }

    int jps::CollectSuccessorDirections(int x_, int y_, int parent_idx_, std::array<SearchDirection, 8>& dirs_) const {
        int count = 0;
        if (parent_idx_ < 0) {
            dirs_[count++] = { 1, 0 };
            dirs_[count++] = { 0, 1 };
            dirs_[count++] = { -1, 0 };
            dirs_[count++] = { 0, -1 };
            dirs_[count++] = { 1, 1 };
            dirs_[count++] = { 1, -1 };
            dirs_[count++] = { -1, -1 };
            dirs_[count++] = { -1, 1 };
            return count;
        }

        int px = parent_idx_ % m_MapLenX;
        int py = parent_idx_ / m_MapLenX;
        int dx = Sign(x_ - px);
        int dy = Sign(y_ - py);

        if (dx != 0 && dy != 0) {
            dirs_[count++] = { dx, dy };
            dirs_[count++] = { dx, 0 };
            dirs_[count++] = { 0, dy };

            if (!IsFreeGrip(x_ - dx, y_) && IsFreeGrip(x_ - dx, y_ + dy)) dirs_[count++] = { -dx, dy };
            if (!IsFreeGrip(x_, y_ - dy) && IsFreeGrip(x_ + dx, y_ - dy)) dirs_[count++] = { dx, -dy };
            return count;
        }

        if (dx != 0) {
            dirs_[count++] = { dx, 0 };
            if (!IsFreeGrip(x_, y_ + 1) && IsFreeGrip(x_ + dx, y_ + 1)) dirs_[count++] = { dx, 1 };
            if (!IsFreeGrip(x_, y_ - 1) && IsFreeGrip(x_ + dx, y_ - 1)) dirs_[count++] = { dx, -1 };
            return count;
        }

        if (dy != 0) {
            dirs_[count++] = { 0, dy };
            if (!IsFreeGrip(x_ + 1, y_) && IsFreeGrip(x_ + 1, y_ + dy)) dirs_[count++] = { 1, dy };
            if (!IsFreeGrip(x_ - 1, y_) && IsFreeGrip(x_ - 1, y_ + dy)) dirs_[count++] = { -1, dy };
            return count;
        }

        dirs_[count++] = { 1, 0 };
        dirs_[count++] = { 0, 1 };
        dirs_[count++] = { -1, 0 };
        dirs_[count++] = { 0, -1 };
        return count;
    }

    bool jps::JumpStraight(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_) {
        int slot = DirectionToCacheSlot(dx_, dy_);
        int start_idx = start_y_ * m_MapLenX + start_x_;
        if (slot >= 0 && m_StraightJumpEpoch[slot][start_idx] == m_SearchEpoch) {
            int cached = m_StraightJumpResult[slot][start_idx];
            if (cached < 0) return false;
            out_x_ = cached % m_MapLenX;
            out_y_ = cached / m_MapLenX;
            return true;
        }

        int cur_x = start_x_ + dx_;
        int cur_y = start_y_ + dy_;
        bool found = false;
        int result_idx = -1;

        while (cur_x >= 0 && cur_x < m_MapLenX && cur_y >= 0 && cur_y < m_MapWeightY) {
            if (!IsFreeGrip(cur_x, cur_y)) break;
            if ((cur_x == m_Target.x && cur_y == m_Target.y) || HasForcedNeighborStraight(cur_x, cur_y, dx_, dy_)) {
                out_x_ = cur_x;
                out_y_ = cur_y;
                found = true;
                result_idx = cur_y * m_MapLenX + cur_x;
                break;
            }
            cur_x += dx_;
            cur_y += dy_;
        }

        if (slot >= 0) {
            m_StraightJumpEpoch[slot][start_idx] = m_SearchEpoch;
            m_StraightJumpResult[slot][start_idx] = result_idx;
        }
        return found;
    }

    bool jps::JumpDiagonal(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_) {
        int cur_x = start_x_ + dx_;
        int cur_y = start_y_ + dy_;

        while (cur_x >= 0 && cur_x < m_MapLenX && cur_y >= 0 && cur_y < m_MapWeightY) {
            if (!IsFreeGrip(cur_x, cur_y)) return false;
            if (cur_x == m_Target.x && cur_y == m_Target.y) {
                out_x_ = cur_x;
                out_y_ = cur_y;
                return true;
            }

            if (HasForcedNeighborDiagonal(cur_x, cur_y, dx_, dy_)) {
                out_x_ = cur_x;
                out_y_ = cur_y;
                return true;
            }

            int tx = 0, ty = 0;
            if (JumpStraight(cur_x, cur_y, dx_, 0, tx, ty) || JumpStraight(cur_x, cur_y, 0, dy_, tx, ty)) {
                out_x_ = cur_x;
                out_y_ = cur_y;
                return true;
            }

            cur_x += dx_;
            cur_y += dy_;
        }
        return false;
    }

    bool jps::Jump(int start_x_, int start_y_, int dx_, int dy_, int& out_x_, int& out_y_) {
        if (dx_ == 0 && dy_ == 0) return false;
        if (dx_ == 0 || dy_ == 0) return JumpStraight(start_x_, start_y_, dx_, dy_, out_x_, out_y_);
        return JumpDiagonal(start_x_, start_y_, dx_, dy_, out_x_, out_y_);
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

            std::array<SearchDirection, 8> dirs;
            int dir_count = CollectSuccessorDirections(current.x, current.y, pParentMap[current.pointIdx], dirs);
            for (int i = 0; i < dir_count; ++i) {
                int x = 0, y = 0;
                if (!Jump(current.x, current.y, dirs[i].dx, dirs[i].dy, x, y)) continue;

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
