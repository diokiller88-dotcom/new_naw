#include "planner_2d/lbfgs.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>
#include <chrono>

namespace planner_2d {

    bool LBFGS::InitMap(const std::vector<float>& cost_map, int width, int height, double res_x, double res_y, double origin_x, double origin_y) {
        if (cost_map.empty() || width <= 0 || height <= 0) return false;
        m_CostMap = cost_map; m_MapWidth = width; m_MapHeight = height; m_ResX = res_x; m_ResY = res_y;
        m_InvResX = 1.0 / res_x; m_InvResY = 1.0 / res_y; m_OriginX = origin_x; m_OriginY = origin_y;
        m_IsOptimized = false;
        return true;
    }

    bool LBFGS::UpdateMapPatch(const std::vector<float>& local_cost_map, int local_w, int local_h,
                               double local_origin_x, double local_origin_y) {
        if (m_CostMap.empty() || local_cost_map.empty() || local_w <= 0 || local_h <= 0) return false;
        if (static_cast<int>(local_cost_map.size()) != local_w * local_h) return false;

        int start_x = static_cast<int>(std::round((local_origin_x - m_OriginX) * m_InvResX));
        int start_y = static_cast<int>(std::round((local_origin_y - m_OriginY) * m_InvResY));

        for (int y = 0; y < local_h; ++y) {
            for (int x = 0; x < local_w; ++x) {
                int gx = start_x + x;
                int gy = start_y + y;
                if (gx < 0 || gx >= m_MapWidth || gy < 0 || gy >= m_MapHeight) continue;
                m_CostMap[gy * m_MapWidth + gx] = local_cost_map[y * local_w + x];
            }
        }
        return true;
    }

    bool LBFGS::InitXState(const std::vector<Eigen::Vector2d>& points, const std::vector<double>& /*init_times*/, double total_time) {
        if (points.size() < 2) return false;
        m_StartPoint = points.front(); m_EndPoint = points.back();
        m_NumInteriorPoints = points.size() - 2; m_TotalTime = total_time;

        if (m_NumInteriorPoints > 0) {
            m_X.resize(2 * m_NumInteriorPoints);
            for (int i = 0; i < m_NumInteriorPoints; ++i) {
                m_X[2 * i] = points[i + 1].x(); m_X[2 * i + 1] = points[i + 1].y();
            }
            m_Gradient.resize(2 * m_NumInteriorPoints); m_Gradient.setZero();
        }

        m_HistoryS.clear(); m_HistoryY.clear(); m_HistoryG.clear(); m_HistoryRho.clear();
        m_Phase1Points.clear(); m_Phase2InitPoints.clear(); m_IsOptimized = false;
        return true;
    }

    bool LBFGS::DoLBFGSIterations(int max_iter) {
        double cost = ComputeCostAndGradient(m_X, m_Gradient);
        auto start_time = std::chrono::high_resolution_clock::now();

        for (int iter = 0; iter < max_iter; ++iter) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - start_time).count();
            if (elapsed_ms > lbfgs_max_i_durationtime) {
                spdlog::warn("[LBFGS] Timeout ({}ms) triggered at iter {}. Breaking early.", elapsed_ms, iter);
                break;
            }

            if (m_Gradient.norm() < lbfgs_min_i_value) break;

            Eigen::VectorXd direction;
            if (!ComputeSearchDirection(direction)) break;

            double max_move = direction.lpNorm<Eigen::Infinity>();
            if (max_move > lbfgs_max_i_move) direction *= (lbfgs_max_i_move / max_move);

            double step = lbfgs_step_len;
            Eigen::VectorXd x_old = m_X;

            if (!LineSearch(step, direction, m_Gradient, cost, m_X)) {
                m_X = x_old;
                m_HistoryS.clear(); m_HistoryY.clear(); m_HistoryG.clear(); m_HistoryRho.clear();
                direction = -m_Gradient; 
                double fallback_move = direction.lpNorm<Eigen::Infinity>();
                if (fallback_move > 0.05) direction *= (0.05 / fallback_move);
                m_X = x_old + direction;
                cost = ComputeCostAndGradient(m_X, m_Gradient);
                continue;
            }

            Eigen::VectorXd grad_new(m_X.size());
            double cost_new = ComputeCostAndGradient(m_X, grad_new);
            Eigen::VectorXd s = m_X - x_old;
            Eigen::VectorXd y = grad_new - m_Gradient;
            double sy = s.dot(y);
            
            if (sy > 1e-10) {
                m_HistoryS.push_back(s); m_HistoryY.push_back(y);
                m_HistoryRho.push_back(1.0 / sy); m_HistoryG.push_back(m_Gradient);
                if (m_HistoryS.size() > static_cast<size_t>(lbfgs_max_size)) {
                    m_HistoryS.erase(m_HistoryS.begin()); m_HistoryY.erase(m_HistoryY.begin());
                    m_HistoryRho.erase(m_HistoryRho.begin()); m_HistoryG.erase(m_HistoryG.begin());
                }
            } else { if (!m_HistoryG.empty()) m_HistoryG.back() = m_Gradient; }
            m_Gradient = grad_new; cost = cost_new;
        }
        return true;
    }

    bool LBFGS::CheckLineOfSight(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const {
        int x0 = static_cast<int>(std::round((p1.x() - m_OriginX) * m_InvResX));
        int y0 = static_cast<int>(std::round((p1.y() - m_OriginY) * m_InvResY));
        int x1 = static_cast<int>(std::round((p2.x() - m_OriginX) * m_InvResX));
        int y1 = static_cast<int>(std::round((p2.y() - m_OriginY) * m_InvResY));

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        int cx = x0, cy = y0;
        
        while (true) {
            if (cx >= 0 && cx < m_MapWidth && cy >= 0 && cy < m_MapHeight) {
                if (m_CostMap[cy * m_MapWidth + cx] >= lbfgs_fatal_cost) return false;
            } else { return false; }
            if (cx == x1 && cy == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
        return true;
    }

    struct LocalAStarNode {
        int idx; double f;
        bool operator>(const LocalAStarNode& other) const { return f > other.f; }
    };

    bool LBFGS::RepairPathAStar(const Eigen::Vector2d& start_phys, const Eigen::Vector2d& goal_phys, std::vector<Eigen::Vector2d>& detour_path) const {
        int sx = static_cast<int>(std::round((start_phys.x() - m_OriginX) * m_InvResX));
        int sy = static_cast<int>(std::round((start_phys.y() - m_OriginY) * m_InvResY));
        int gx = static_cast<int>(std::round((goal_phys.x() - m_OriginX) * m_InvResX));
        int gy = static_cast<int>(std::round((goal_phys.y() - m_OriginY) * m_InvResY));

        if (sx < 0 || sx >= m_MapWidth || sy < 0 || sy >= m_MapHeight || gx < 0 || gx >= m_MapWidth || gy < 0 || gy >= m_MapHeight) return false;

        std::vector<double> g_score(m_MapWidth * m_MapHeight, std::numeric_limits<double>::max());
        std::vector<int> parent(m_MapWidth * m_MapHeight, -1);
        std::priority_queue<LocalAStarNode, std::vector<LocalAStarNode>, std::greater<LocalAStarNode>> open_list;

        int start_idx = sy * m_MapWidth + sx;
        int goal_idx  = gy * m_MapWidth + gx;

        g_score[start_idx] = 0.0;
        open_list.push({start_idx, std::hypot(gx - sx, gy - sy)});

        const int dir_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        const int dir_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};
        const double d_cost[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};

        while (!open_list.empty()) {
            LocalAStarNode current = open_list.top();
            open_list.pop();

            if (current.idx == goal_idx) {
                std::vector<int> path_idx;
                int curr = goal_idx;
                while (curr != -1) { path_idx.push_back(curr); curr = parent[curr]; }
                std::reverse(path_idx.begin(), path_idx.end());
                
                detour_path.push_back(start_phys);
                double acc_dist = 0.0;
                for (size_t i = 1; i < path_idx.size(); ++i) {
                    int p_cx = path_idx[i] % m_MapWidth, p_cy = path_idx[i] / m_MapWidth;
                    int p_lx = path_idx[i-1] % m_MapWidth, p_ly = path_idx[i-1] / m_MapWidth;
                    acc_dist += std::hypot(p_cx - p_lx, p_cy - p_ly) * m_ResX;
                    if (acc_dist >= lbfgs_sample_dist) {
                        detour_path.emplace_back(m_OriginX + p_cx * m_ResX, m_OriginY + p_cy * m_ResY);
                        acc_dist = 0.0;
                    }
                }
                if ((detour_path.back() - goal_phys).norm() > 1e-3) detour_path.push_back(goal_phys);
                return true;
            }

            int cx = current.idx % m_MapWidth, cy = current.idx / m_MapWidth;
            for (int i = 0; i < 8; ++i) {
                int nx = cx + dir_x[i], ny = cy + dir_y[i];
                if (nx >= 0 && nx < m_MapWidth && ny >= 0 && ny < m_MapHeight) {
                    int n_idx = ny * m_MapWidth + nx;
                    if (m_CostMap[n_idx] >= lbfgs_fatal_cost) continue;
                    double move_cost = d_cost[i] + m_CostMap[n_idx] * 0.01; 
                    double new_g = g_score[current.idx] + move_cost;

                    if (new_g < g_score[n_idx]) {
                        g_score[n_idx] = new_g; parent[n_idx] = current.idx;
                        open_list.push({n_idx, new_g + std::hypot(gx - nx, gy - ny)});
                    }
                }
            }
        }
        return false; 
    }

    float LBFGS::QueryCost(const Eigen::Vector2d& point) const {
        if (m_CostMap.empty()) return 0.0f;
        int gx = static_cast<int>(std::round((point.x() - m_OriginX) * m_InvResX));
        int gy = static_cast<int>(std::round((point.y() - m_OriginY) * m_InvResY));
        if (gx < 0 || gx >= m_MapWidth || gy < 0 || gy >= m_MapHeight) return lbfgs_fatal_cost;
        return m_CostMap[gy * m_MapWidth + gx];
    }

    double LBFGS::QueryAdaptiveSampleDist(const Eigen::Vector2d& point) const {
        float cost = QueryCost(point);
        if (cost >= lbfgs_fatal_cost) return 0.25;
        double normalized = std::clamp(static_cast<double>(cost) / 2.5, 0.0, 1.0);
        return 1.15 - 0.8 * normalized;
    }

    void LBFGS::BuildAdaptiveSamples(const std::vector<Eigen::Vector2d>& source_path, std::vector<Eigen::Vector2d>& dense_pts) {
        dense_pts.clear();
        if (source_path.empty()) return;

        dense_pts.push_back(source_path.front());
        for (size_t i = 0; i + 1 < source_path.size(); ++i) {
            const Eigen::Vector2d& p_start = source_path[i];
            const Eigen::Vector2d& p_end = source_path[i + 1];
            double dist = (p_end - p_start).norm();
            if (dist <= 1e-6) {
                if ((dense_pts.back() - p_end).norm() > 1e-6) dense_pts.push_back(p_end);
                continue;
            }

            Eigen::Vector2d mid = 0.5 * (p_start + p_end);
            double sample_dist = std::min({QueryAdaptiveSampleDist(p_start), QueryAdaptiveSampleDist(mid), QueryAdaptiveSampleDist(p_end)});
            int num_samples = std::max(1, static_cast<int>(std::round(dist / sample_dist)));

            for (int j = 1; j < num_samples; ++j) {
                dense_pts.push_back(p_start + (p_end - p_start) * (static_cast<double>(j) / num_samples));
            }
            if ((dense_pts.back() - p_end).norm() > 1e-6) dense_pts.push_back(p_end);
        }
    }

    bool LBFGS::Optimize() {
        if (m_NumInteriorPoints == 0) {
            m_Minco.Init({m_StartPoint, m_EndPoint});
            m_Times.resize(1); m_Times[0] = m_TotalTime;
            m_Minco.SetPath(m_Times);
            m_Phase1Points = {m_StartPoint, m_EndPoint};
            m_IsOptimized = true;
            return true;
        }

        m_OptimizePhase = 1;
        DoLBFGSIterations(lbfgs_max_firsti_time);

        std::vector<Eigen::Vector2d> raw_p1_pts;
        raw_p1_pts.push_back(m_StartPoint);
        for (int i = 0; i < m_NumInteriorPoints; ++i) raw_p1_pts.emplace_back(m_X[2 * i], m_X[2 * i + 1]);
        raw_p1_pts.push_back(m_EndPoint);

        m_Phase1Points.clear();
        m_Phase1Points.push_back(raw_p1_pts.front());
        for (size_t i = 0; i < raw_p1_pts.size() - 1; ++i) {
            if (!CheckLineOfSight(raw_p1_pts[i], raw_p1_pts[i+1])) {
                std::vector<Eigen::Vector2d> detour;
                if (RepairPathAStar(raw_p1_pts[i], raw_p1_pts[i+1], detour)) {
                    for (size_t j = 1; j < detour.size(); ++j) m_Phase1Points.push_back(detour[j]);
                } else { m_Phase1Points.push_back(raw_p1_pts[i+1]); }
            } else { m_Phase1Points.push_back(raw_p1_pts[i+1]); }
        }

        m_OptimizePhase = 2;
        BuildAdaptiveSamples(m_Phase1Points, m_DensePointCache);
        std::vector<Eigen::Vector2d>& dense_pts = m_DensePointCache;
        if (dense_pts.empty()) dense_pts = {m_StartPoint, m_EndPoint};

        m_Phase2InitPoints = dense_pts;
        m_StartPoint = dense_pts.front(); m_EndPoint = dense_pts.back();
        m_NumInteriorPoints = dense_pts.size() - 2;
        
        m_X.resize(2 * m_NumInteriorPoints);
        for (int i = 0; i < m_NumInteriorPoints; ++i) { m_X[2 * i] = dense_pts[i + 1].x(); m_X[2 * i + 1] = dense_pts[i + 1].y(); }
        m_Gradient.resize(2 * m_NumInteriorPoints); m_Gradient.setZero();
        m_HistoryS.clear(); m_HistoryY.clear(); m_HistoryG.clear(); m_HistoryRho.clear();
        
        DoLBFGSIterations(lbfgs_max_secondi_time);

        std::vector<Eigen::Vector2d> final_pts(m_NumInteriorPoints + 2);
        final_pts[0] = m_StartPoint; final_pts.back() = m_EndPoint;
        for (int i = 0; i < m_NumInteriorPoints; ++i) final_pts[i + 1] = Eigen::Vector2d(m_X[2 * i], m_X[2 * i + 1]);

        m_Minco.Init(final_pts); m_Times.resize(final_pts.size() - 1);
        double total_dist = 0; std::vector<double> dists(final_pts.size() - 1);
        for (size_t i = 0; i < final_pts.size() - 1; ++i) { dists[i] = (final_pts[i + 1] - final_pts[i]).norm(); total_dist += dists[i]; }
        if (total_dist <= 1e-6) {
            double even_time = std::max(m_TotalTime / static_cast<double>(final_pts.size() - 1), lbfgs_min_minco_time);
            for (size_t i = 0; i < final_pts.size() - 1; ++i) m_Times[i] = even_time;
        } else {
            for (size_t i = 0; i < final_pts.size() - 1; ++i) {
                m_Times[i] = m_TotalTime * (dists[i] / total_dist);
                if (m_Times[i] < lbfgs_min_minco_time) m_Times[i] = lbfgs_min_minco_time; 
            }
        }
        if (!m_Minco.SetPath(m_Times)) return false;
        m_IsOptimized = true; return true;
    }

    double LBFGS::ComputeCostAndGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        grad.setZero(x.size()); double cost = 0.0;
        std::vector<Eigen::Vector2d> pts(m_NumInteriorPoints + 2);
        pts[0] = m_StartPoint; pts.back() = m_EndPoint;
        for (int i = 0; i < m_NumInteriorPoints; ++i) pts[i + 1] = Eigen::Vector2d(x[2 * i], x[2 * i + 1]);

        double w_obs = (m_OptimizePhase == 1) ? m_w1_obstacle : m_w2_obstacle;
        double w_len = (m_OptimizePhase == 1) ? m_w1_length   : m_w2_length;
        double w_smo = (m_OptimizePhase == 1) ? 0.0           : m_w2_smooth;

        for (int i = 1; i <= m_NumInteriorPoints; ++i) {
            double gx = (pts[i].x() - m_OriginX) * m_InvResX, gy = (pts[i].y() - m_OriginY) * m_InvResY;
            int fast_ix = static_cast<int>(std::round(gx)), fast_iy = static_cast<int>(std::round(gy));
            if (fast_ix >= 0 && fast_ix < m_MapWidth && fast_iy >= 0 && fast_iy < m_MapHeight) {
                if (m_CostMap[fast_iy * m_MapWidth + fast_ix] < 1e-4) continue; 
            }

            double obs_cost = 0.0; Eigen::Vector2d grad_obs(0.0, 0.0);
            if (gx >= 1.0 && gx < m_MapWidth - 2.0 && gy >= 1.0 && gy < m_MapHeight - 2.0) {
                double dx = gx - fast_ix, dy = gy - fast_iy;
                float f00   = m_CostMap[fast_iy * m_MapWidth + fast_ix];
                float f10   = m_CostMap[fast_iy * m_MapWidth + fast_ix + 1];
                float fm10  = m_CostMap[fast_iy * m_MapWidth + fast_ix - 1];
                float f01   = m_CostMap[(fast_iy + 1) * m_MapWidth + fast_ix];
                float f0m1  = m_CostMap[(fast_iy - 1) * m_MapWidth + fast_ix];
                float f11   = m_CostMap[(fast_iy + 1) * m_MapWidth + fast_ix + 1];
                float f1m1  = m_CostMap[(fast_iy - 1) * m_MapWidth + fast_ix + 1];
                float fm11  = m_CostMap[(fast_iy + 1) * m_MapWidth + fast_ix - 1];
                float fm1m1 = m_CostMap[(fast_iy - 1) * m_MapWidth + fast_ix - 1];

                double cx = (f10 - fm10) * 0.5, cy = (f01 - f0m1) * 0.5;
                double cxx = f10 - 2.0 * f00 + fm10, cyy = f01 - 2.0 * f00 + f0m1;
                double cxy = (f11 - f1m1 - fm11 + fm1m1) * 0.25;

                obs_cost = f00 + cx * dx + cy * dy + 0.5 * cxx * dx * dx + 0.5 * cyy * dy * dy + cxy * dx * dy;
                if (obs_cost < 0.0) obs_cost = 0.0; 

                grad_obs.x() = (cx + cxx * dx + cxy * dy) * m_InvResX;
                grad_obs.y() = (cy + cyy * dy + cxy * dx) * m_InvResY;
            } else {
                obs_cost = 1e5; 
                grad_obs.x() = (gx < 1.0) ? -2e4 : (gx >= m_MapWidth - 2.0) ? 2e4 : 0.0;
                grad_obs.y() = (gy < 1.0) ? -2e4 : (gy >= m_MapHeight - 2.0) ? 2e4 : 0.0;
            }

            cost += w_obs * obs_cost;
            grad[2 * (i - 1)]     += w_obs * grad_obs.x();
            grad[2 * (i - 1) + 1] += w_obs * grad_obs.y();
        }

        if (m_OptimizePhase == 1 && w_len > 0) {
            for (size_t i = 1; i < pts.size(); ++i) {
                Eigen::Vector2d diff = pts[i] - pts[i - 1];
                double len = diff.norm();
                if (len > 1e-4) {
                    cost += w_len * len;
                    Eigen::Vector2d grad_len = w_len * (diff / len);
                    if (i <= static_cast<size_t>(m_NumInteriorPoints)) {
                        grad[2 * (i - 1)] += grad_len.x(); grad[2 * (i - 1) + 1] += grad_len.y();
                    }
                    if (i - 1 >= 1) { 
                        grad[2 * (i - 2)] -= grad_len.x(); grad[2 * (i - 2) + 1] -= grad_len.y();
                    }
                }
            }
        }

        if (m_OptimizePhase == 2) {
            for (int i = 1; i <= m_NumInteriorPoints; ++i) {
                Eigen::Vector2d d_smooth = pts[i-1] - 2.0 * pts[i] + pts[i+1];
                cost += w_smo * d_smooth.squaredNorm();

                Eigen::Vector2d g_smo = 2.0 * w_smo * d_smooth;
                grad[2 * (i - 1)] += g_smo.x() * (-2.0); grad[2 * (i - 1) + 1] += g_smo.y() * (-2.0);
                if (i - 1 >= 1) { grad[2 * (i - 2)] += g_smo.x(); grad[2 * (i - 2) + 1] += g_smo.y(); }
                if (i + 1 <= m_NumInteriorPoints) { grad[2 * i] += g_smo.x(); grad[2 * i + 1] += g_smo.y(); }

                Eigen::Vector2d diff_anchor = pts[i] - m_Phase2InitPoints[i];
                cost += m_w2_anchor * diff_anchor.squaredNorm();
                grad[2 * (i - 1)] += 2.0 * m_w2_anchor * diff_anchor.x();
                grad[2 * (i - 1) + 1] += 2.0 * m_w2_anchor * diff_anchor.y();
            }
        }
        return cost;
    }

    bool LBFGS::ComputeSearchDirection(Eigen::VectorXd& direction) {
        if (m_HistoryG.empty()) { direction = -m_Gradient; return true; }
        Eigen::VectorXd q = m_Gradient;
        const int m = static_cast<int>(m_HistoryS.size());
        std::vector<double> alpha(m);
        for (int i = m - 1; i >= 0; --i) {
            alpha[i] = m_HistoryRho[i] * m_HistoryS[i].dot(q);
            q -= alpha[i] * m_HistoryY[i];
        }
        double gamma = 1.0;
        if (m > 0) {
            const auto& s_last = m_HistoryS.back(); const auto& y_last = m_HistoryY.back();
            double yy = y_last.dot(y_last);
            if (yy > 1e-10) gamma = s_last.dot(y_last) / yy;
        }
        Eigen::VectorXd r = gamma * q;
        for (int i = 0; i < m; ++i) {
            double beta = m_HistoryRho[i] * m_HistoryY[i].dot(r);
            r += m_HistoryS[i] * (alpha[i] - beta);
        }
        direction = -r; return true;
    }

    bool LBFGS::LineSearch(double& step, const Eigen::VectorXd& direction, const Eigen::VectorXd& grad, double cost, Eigen::VectorXd& x) {
        double step_trial = step; Eigen::VectorXd x_orig = x;
        double grad_dir = grad.dot(direction);
        if (grad_dir >= 0.0) return false;

        std::vector<bool> started_free(m_NumInteriorPoints);
        for (int i = 0; i < m_NumInteriorPoints; ++i) {
            int cx = static_cast<int>(std::round((x_orig[2*i] - m_OriginX) * m_InvResX));
            int cy = static_cast<int>(std::round((x_orig[2*i+1] - m_OriginY) * m_InvResY));
            if (cx >= 0 && cx < m_MapWidth && cy >= 0 && cy < m_MapHeight) started_free[i] = (m_CostMap[cy * m_MapWidth + cx] < lbfgs_fatal_cost);
            else started_free[i] = false;
        }

        for (int iter = 0; iter < 15; ++iter) {
            Eigen::VectorXd x_new = x_orig + step_trial * direction;
            bool is_invalid_step = false;

            for (int i = 0; i < m_NumInteriorPoints; ++i) {
                if (!started_free[i]) continue; 
                int x0 = static_cast<int>(std::round((x_orig[2*i] - m_OriginX) * m_InvResX)), y0 = static_cast<int>(std::round((x_orig[2*i+1] - m_OriginY) * m_InvResY));
                int x1 = static_cast<int>(std::round((x_new[2*i] - m_OriginX) * m_InvResX)), y1 = static_cast<int>(std::round((x_new[2*i+1] - m_OriginY) * m_InvResY));
                int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1, dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
                int err = dx + dy, e2, cx = x0, cy = y0;
                
                while (true) {
                    if (cx >= 0 && cx < m_MapWidth && cy >= 0 && cy < m_MapHeight) {
                        if (m_CostMap[cy * m_MapWidth + cx] >= lbfgs_fatal_cost) { is_invalid_step = true; break; }
                    } else { is_invalid_step = true; break; }
                    if (cx == x1 && cy == y1) break;
                    e2 = 2 * err;
                    if (e2 >= dy) { err += dy; cx += sx; }
                    if (e2 <= dx) { err += dx; cy += sy; }
                }
                if (is_invalid_step) break;
            }

            if (!is_invalid_step) {
                Eigen::VectorXd grad_new(x.size());
                double cost_new = ComputeCostAndGradient(x_new, grad_new);

                for (int i = 0; i < m_NumInteriorPoints; ++i) {
                    Eigen::Vector2d g_old(grad[2 * i], grad[2 * i + 1]);
                    Eigen::Vector2d g_new(grad_new[2 * i], grad_new[2 * i + 1]);
                    double norms = g_old.norm() * g_new.norm(), dot_prod = g_old.dot(g_new);
                    if (norms > 1e-4 && dot_prod < -0.5 * norms) { is_invalid_step = true; break; }
                }

                if (!is_invalid_step) {
                    if (cost_new <= cost + lbfgs_armijo * step_trial * grad_dir) { step = step_trial; x = x_new; return true; }
                }
            }
            step_trial *= lbfgs_backtrack;
        }
        return false;
    }

    std::vector<Eigen::Vector2d> LBFGS::GetTrajectorySamples(double dt) const {
        std::vector<Eigen::Vector2d> samples;
        if (!m_IsOptimized) return samples;
        const auto& path = m_Minco.GetPath();
        const auto& times = m_Times;
        for (size_t seg = 0; seg < path.size(); ++seg) {
            double T = times[seg];
            for (double t = 0.0; t <= T + 1e-6; t += dt) samples.push_back(m_Minco.GetPosition(static_cast<int>(seg), t));
        }
        return samples;
    }

    bool LBFGS::OptimizeLocalPhase2(const std::vector<Eigen::Vector2d>& local_path, double total_time) {
        if (local_path.size() < 2) return false;
        m_StartPoint = local_path.front(); m_EndPoint = local_path.back(); m_TotalTime = total_time;
        
        m_Phase1Points = local_path;
        m_OptimizePhase = 2;
        BuildAdaptiveSamples(m_Phase1Points, m_DensePointCache);
        std::vector<Eigen::Vector2d>& dense_pts = m_DensePointCache;
        if (dense_pts.empty()) dense_pts = {m_StartPoint, m_EndPoint};

        m_Phase2InitPoints = dense_pts;
        m_StartPoint = dense_pts.front(); m_EndPoint = dense_pts.back();
        m_NumInteriorPoints = dense_pts.size() - 2;
        
        if (m_NumInteriorPoints <= 0) {
            m_Minco.Init({m_StartPoint, m_EndPoint}); m_Times.resize(1); m_Times[0] = m_TotalTime;
            m_Minco.SetPath(m_Times); m_IsOptimized = true; return true;
        }

        m_X.resize(2 * m_NumInteriorPoints);
        for (int i = 0; i < m_NumInteriorPoints; ++i) { m_X[2 * i] = dense_pts[i + 1].x(); m_X[2 * i + 1] = dense_pts[i + 1].y(); }
        m_Gradient.resize(2 * m_NumInteriorPoints); m_Gradient.setZero();
        m_HistoryS.clear(); m_HistoryY.clear(); m_HistoryG.clear(); m_HistoryRho.clear();
        
        DoLBFGSIterations(lbfgs_max_secondi_time);

        std::vector<Eigen::Vector2d> final_pts(m_NumInteriorPoints + 2);
        final_pts[0] = m_StartPoint; final_pts.back() = m_EndPoint;
        for (int i = 0; i < m_NumInteriorPoints; ++i) final_pts[i + 1] = Eigen::Vector2d(m_X[2 * i], m_X[2 * i + 1]);

        m_Minco.Init(final_pts); m_Times.resize(final_pts.size() - 1);
        double total_dist = 0; std::vector<double> dists(final_pts.size() - 1);
        for (size_t i = 0; i < final_pts.size() - 1; ++i) { dists[i] = (final_pts[i + 1] - final_pts[i]).norm(); total_dist += dists[i]; }
        if (total_dist <= 1e-6) {
            double even_time = std::max(m_TotalTime / static_cast<double>(final_pts.size() - 1), lbfgs_min_minco_time);
            for (size_t i = 0; i < final_pts.size() - 1; ++i) m_Times[i] = even_time;
        } else {
            for (size_t i = 0; i < final_pts.size() - 1; ++i) {
                m_Times[i] = m_TotalTime * (dists[i] / total_dist);
                if (m_Times[i] < lbfgs_min_minco_time) m_Times[i] = lbfgs_min_minco_time; 
            }
        }

        if (!m_Minco.SetPath(m_Times)) return false;
        m_IsOptimized = true; return true;
    }

    void LBFGS::SetCustomTrajectory(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& times) {
        if(points.size() < 2) return;
        m_StartPoint = points.front(); m_EndPoint = points.back();
        m_NumInteriorPoints = points.size() - 2;
        m_Minco.Init(points); m_Minco.SetPath(times); m_Times = times; m_IsOptimized = true;
    }
} // namespace planner_2d
