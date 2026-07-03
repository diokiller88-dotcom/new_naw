#include "planner_2d/replanner.hpp"
#include <chrono>
#include <cmath>
#include <queue>
#include <spdlog/spdlog.h>

namespace planner_2d {
    namespace {
        constexpr double min_particalreplan_sq = min_particalreplan * min_particalreplan;
        constexpr double min_wholereplan_sq = min_wholereplan * min_wholereplan;
        constexpr double narrow_region_esdf = 0.9;
        constexpr double narrow_region_sample_step = 0.25;
        constexpr double open_region_sample_step = 1.0;

        float ComputeCostFromESDF(double dist) {
            if (dist < 0.05) return 1e4f;
            if (dist < safe_dist) {
                double penalty = safe_dist - dist;
                return static_cast<float>(std::pow(penalty, 3) * 20.0);
            }
            return 0.0f;
        }

        Eigen::Vector2d GetForwardSample(const minco& traj) {
            int num_points = static_cast<int>(traj.GetNumPoints());
            if (num_points < 2) return Eigen::Vector2d::Zero();
            int seg_idx = std::min(1, num_points - 2);
            return traj.GetPosition(seg_idx, 0.0);
        }
    }

    replanner::replanner(const jps& jps_, const LBFGS& lbfgs_, const project& project_)
        : m_jps(jps_), m_lbfgs(lbfgs_), m_Project(project_) ,m_ResPose(Eigen::Vector2d::Zero()),m_ResVec(Eigen::Vector2d::Zero()){}

    void replanner::ResetPlanStats() {
        m_LastPlanType = "none";
        m_LastPlanDurationMs = 0.0;
        m_LastJpsDurationMs = 0.0;
        m_LastPlanTriggered = false;
    }

    void replanner::SetPlanStats(const std::string& plan_type, double duration_ms) {
        m_LastPlanType = plan_type;
        m_LastPlanDurationMs = duration_ms;
        m_LastPlanTriggered = true;
    }

    bool replanner::InitMapWithESDF(const std::vector<int>& occupancy_map, const std::vector<double>& esdf_map, int width, int height, double res_x, double res_y, double origin_x, double origin_y){
        m_ResX = res_x; m_ResY = res_y; m_OriginX = origin_x; m_OriginY = origin_y; m_MapLenX = width; m_MapWeightY = height;
        m_GetRes = false;
        ResetPlanStats();
        
        m_GlobalOccupancy = occupancy_map;
        m_GlobalEsdf = esdf_map;

        std::vector<float> cost_map(width * height, 0.0f);
        for (size_t i = 0; i < esdf_map.size(); ++i) {
            double dist = esdf_map[i]; 
            if (dist < 0.05) cost_map[i] = 1e4f;
            else if (dist < safe_dist) {
                double penalty = safe_dist - dist;
                cost_map[i] = static_cast<float>(std::pow(penalty, 3) * 20.0);
            } else { cost_map[i] = 0.0f; }
        }
        if (!m_lbfgs.InitMap(cost_map, width, height, m_ResX, m_ResY, m_OriginX, m_OriginY)) { return false; }
        m_ReplanAstar.InitMapWithESDF(occupancy_map, esdf_map, width, height);
        if (!m_jps.InitMapWithESDF(occupancy_map, esdf_map, width, height)) return false;
        return true;
    }

    void replanner::UpdateGlobalMap(const std::vector<int>& local_occ, const std::vector<double>& local_esdf,
                                    int local_w, int local_h, double local_origin_x, double local_origin_y) {
        if (m_GlobalOccupancy.empty() || m_GlobalEsdf.empty()) return;

        int start_x = static_cast<int>(std::round((local_origin_x - m_OriginX) / m_ResX));
        int start_y = static_cast<int>(std::round((local_origin_y - m_OriginY) / m_ResY));

        std::vector<float> local_cost_map(local_w * local_h, 0.0f);
        for (int y = 0; y < local_h; ++y) {
            for (int x = 0; x < local_w; ++x) {
                int gx = start_x + x;
                int gy = start_y + y;
                int l_idx = y * local_w + x;
                local_cost_map[l_idx] = ComputeCostFromESDF(local_esdf[l_idx]);
                if (gx >= 0 && gx < m_MapLenX && gy >= 0 && gy < m_MapWeightY) {
                    int g_idx = gy * m_MapLenX + gx;
                    m_GlobalOccupancy[g_idx] = local_occ[l_idx];
                    m_GlobalEsdf[g_idx] = local_esdf[l_idx];
                }
            }
        }

        m_lbfgs.UpdateMapPatch(local_cost_map, local_w, local_h, local_origin_x, local_origin_y);
        m_ReplanAstar.UpdateMapPatchWithESDF(local_occ, local_esdf, local_w, local_h, start_x, start_y);
        m_jps.UpdateMapPatchWithESDF(local_occ, local_esdf, local_w, local_h, start_x, start_y);
    }

    Eigen::Vector2d replanner::GridToPhys(const Eigen::Vector2i& grid_pt) const {
        return Eigen::Vector2d(m_OriginX + grid_pt.x() * m_ResX, m_OriginY + grid_pt.y() * m_ResY);
    }

    bool replanner::IsNarrowGrid(int gx, int gy) const {
        if (gx < 0 || gx >= m_MapLenX || gy < 0 || gy >= m_MapWeightY) return true;
        return m_GlobalEsdf[gy * m_MapLenX + gx] < narrow_region_esdf;
    }

    void replanner::AppendUniquePoint(std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& point) const {
        if (path.empty() || (path.back() - point).norm() > 1e-6) path.push_back(point);
    }

    void replanner::RasterizeSegment(const Eigen::Vector2i& start_pt, const Eigen::Vector2i& end_pt) {
        m_GridSegmentCache.clear();

        int x0 = start_pt.x(), y0 = start_pt.y();
        int x1 = end_pt.x(), y1 = end_pt.y();
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            m_GridSegmentCache.emplace_back(x0, y0);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void replanner::AppendAdaptiveSegmentSamples(int begin_idx, int end_idx, bool is_narrow, std::vector<Eigen::Vector2d>& phys_path) {
        if (begin_idx < 0 || end_idx < begin_idx || end_idx >= static_cast<int>(m_GridSegmentCache.size())) return;

        double sample_step = is_narrow ? narrow_region_sample_step : open_region_sample_step;
        AppendUniquePoint(phys_path, GridToPhys(m_GridSegmentCache[begin_idx]));

        double accum_dist = 0.0;
        for (int i = begin_idx + 1; i <= end_idx; ++i) {
            accum_dist += (GridToPhys(m_GridSegmentCache[i]) - GridToPhys(m_GridSegmentCache[i - 1])).norm();
            if (accum_dist >= sample_step || i == end_idx) {
                AppendUniquePoint(phys_path, GridToPhys(m_GridSegmentCache[i]));
                accum_dist = 0.0;
            }
        }
    }

    void replanner::BuildSegmentedAdaptivePath(const std::vector<Eigen::Vector2d>& jps_grid_path, std::vector<Eigen::Vector2d>& phys_path) {
        phys_path.clear();
        if (jps_grid_path.empty()) return;

        if (jps_grid_path.size() == 1) {
            AppendUniquePoint(phys_path, Eigen::Vector2d(m_OriginX + jps_grid_path.front().x() * m_ResX,
                                                         m_OriginY + jps_grid_path.front().y() * m_ResY));
            return;
        }

        for (size_t seg_idx = 1; seg_idx < jps_grid_path.size(); ++seg_idx) {
            Eigen::Vector2i start_pt(static_cast<int>(std::round(jps_grid_path[seg_idx - 1].x())),
                                     static_cast<int>(std::round(jps_grid_path[seg_idx - 1].y())));
            Eigen::Vector2i end_pt(static_cast<int>(std::round(jps_grid_path[seg_idx].x())),
                                   static_cast<int>(std::round(jps_grid_path[seg_idx].y())));
            RasterizeSegment(start_pt, end_pt);
            if (m_GridSegmentCache.empty()) continue;

            int run_start = 0;
            bool current_narrow = IsNarrowGrid(m_GridSegmentCache.front().x(), m_GridSegmentCache.front().y());

            for (int i = 1; i < static_cast<int>(m_GridSegmentCache.size()); ++i) {
                bool next_narrow = IsNarrowGrid(m_GridSegmentCache[i].x(), m_GridSegmentCache[i].y());
                if (next_narrow != current_narrow) {
                    AppendAdaptiveSegmentSamples(run_start, i - 1, current_narrow, phys_path);
                    AppendUniquePoint(phys_path, GridToPhys(m_GridSegmentCache[i]));
                    run_start = i;
                    current_narrow = next_narrow;
                }
            }
            AppendAdaptiveSegmentSamples(run_start, static_cast<int>(m_GridSegmentCache.size()) - 1, current_narrow, phys_path);
        }
    }

    bool replanner::InitPoint(const double& source_x_, const double& source_y_, const double& target_x_, const double& target_y_){
        auto t_start = std::chrono::high_resolution_clock::now();
        m_GetRes = false;
        ResetPlanStats();

        Eigen::Vector2d valid_target = ClosestFreeGoal(Eigen::Vector2d(target_x_, target_y_));

        m_SourcePoint.x() = source_x_; m_SourcePoint.y() = source_y_; 
        m_TargetPoint = valid_target;

        int sx = static_cast<int>((source_x_ - m_OriginX) / m_ResX), sy = static_cast<int>((source_y_ - m_OriginY) / m_ResY);
        int tx = static_cast<int>((valid_target.x() - m_OriginX) / m_ResX), ty = static_cast<int>((valid_target.y() - m_OriginY) / m_ResY);
        if (!m_jps.InitPoint(sx, sy, tx, ty)) return false;
        auto t_jps = std::chrono::high_resolution_clock::now();
        if (!m_jps.SetPath()) return false;
        m_LastJpsDurationMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_jps).count();

        std::vector<Eigen::Vector2d> grid_path = m_jps.GetEigenPath();
        m_AdaptivePathCache.clear();
        BuildSegmentedAdaptivePath(grid_path, m_AdaptivePathCache);
        if (m_AdaptivePathCache.size() < 2) {
            m_AdaptivePathCache.clear();
            for (const auto& pt : grid_path) {
                m_AdaptivePathCache.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
            }
        }
        const auto& phys_path = m_AdaptivePathCache;
        double time = ComputeWholeTime(phys_path);
        if (!m_lbfgs.InitXState(phys_path, {}, time)) return false;
        if (!m_lbfgs.Optimize()) spdlog::warn("[Replanner] LBFGS returned false.");
        SetTrajectory(); 
        double duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        SetPlanStats("global", duration_ms);
        return true;
    }

    bool replanner::SetTrajectory() { m_Project.BuildAABBTree(m_lbfgs.GetMincoTrajectory()); return true; }
    
    double replanner::ComputeWholeTime(const std::vector<Eigen::Vector2d>& path_){
        double dist = 0.0;
        for (size_t i = 1; i < path_.size(); ++i) dist += (path_[i] - path_[i-1]).norm();
        if (dist > acc_dist) return max_speed / max_acc + (dist - acc_dist) / max_speed;
        return std::sqrt(dist / max_acc);
    }

    bool replanner::Update(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_) {
        return Update(p_, v_, {}, {}, 0, 0, 0.0, 0.0);
    }

    bool replanner::Update(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_, 
                           const std::vector<int>& local_occ, const std::vector<double>& local_esdf, 
                           int local_w, int local_h, double local_origin_x, double local_origin_y) {
        m_GetRes = false;
        ResetPlanStats();
        m_DetourPath.clear();
        m_FrontTime = 0.0; 
        
        if (!local_occ.empty()) {
            UpdateGlobalMap(local_occ, local_esdf, local_w, local_h, local_origin_x, local_origin_y);
        }

        ProjectionResult close_pos = m_Project.FindClosestPoint(p_);
        Eigen::Vector2d temp_vec = close_pos.pos - p_;
        auto temp_tra = m_lbfgs.GetMincoTrajectory();
        Eigen::Vector2d temp_speed = temp_tra.GetVelocity(close_pos.seg_idx, close_pos.t);
        
        Eigen::Vector2d resvec_ = temp_speed;
        Eigen::Vector2d respose_ = close_pos.pos;
        
        Eigen::Vector2d grid_p((p_.x() - m_OriginX) / m_ResX, (p_.y() - m_OriginY) / m_ResY);
        Eigen::Vector2d grid_proj((close_pos.pos.x() - m_OriginX) / m_ResX, (close_pos.pos.y() - m_OriginY) / m_ResY);
        
        bool nostop = m_jps.IsNonStop(grid_p, grid_proj);

        int idx = close_pos.seg_idx;
        int splice_idx = idx + 1;
        double window_radius = windowradius_cont * m_ResX; 
        for (int k = idx + 1; k < temp_tra.GetNumPoints() - 1; ++k) {
            Eigen::Vector2d pt = temp_tra.GetPosition(k, 0.0);
            if ((pt - p_).norm() > window_radius) break; 
            splice_idx = k;
        }
        splice_idx = std::min(splice_idx, static_cast<int>(temp_tra.GetNumPoints()) - 2);
        if (splice_idx <= idx) splice_idx = std::min(idx + 3, static_cast<int>(temp_tra.GetNumPoints()) - 2);

        if (close_pos.dist_sq > min_wholereplan_sq) {
            InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
            resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
            respose_ = GetForwardSample(m_lbfgs.GetMincoTrajectory());
            m_FrontPoint = respose_;
        }
        else if (nostop && close_pos.dist_sq <= min_particalreplan_sq) {
            Eigen::Vector2d frontpoint = temp_tra.GetPosition(splice_idx, 0.0);
            m_FrontPoint = frontpoint; 
            
            Eigen::Vector2d grid_front((frontpoint.x() - m_OriginX) / m_ResX, (frontpoint.y() - m_OriginY) / m_ResY);

            if (m_jps.IsNonStop(grid_proj, grid_front)) {
                resvec_ += temp_vec; 
            } else {
                Eigen::Vector2d frontvector = temp_tra.GetVelocity(splice_idx, 0.0);
                int sx = static_cast<int>(grid_proj.x()), sy = static_cast<int>(grid_proj.y());
                int gx = static_cast<int>(grid_front.x()), gy = static_cast<int>(grid_front.y());

                auto t_start = std::chrono::high_resolution_clock::now();
                m_ReplanAstar.SetStartGoal(sx, sy, gx, gy);
                if (m_ReplanAstar.FindPath() && (m_ReplanAstar.GetPath().size() * m_ResX) < min_wholereplan * 3.0) {
                    for (const auto& pt : m_ReplanAstar.GetPath()) {
                        m_DetourPath.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
                    }
                    PraticalReplan(p_, v_, frontpoint, frontvector, m_ReplanAstar.GetPath(), splice_idx);
                    resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
                    resvec_ *= 0.5; 
                    double duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
                    SetPlanStats("local", duration_ms);
                } else {
                    InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
                    resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
                }
                respose_ = GetForwardSample(m_lbfgs.GetMincoTrajectory());
            }
        }
        else if (!nostop || (nostop && close_pos.dist_sq > min_particalreplan_sq)) {
            Eigen::Vector2d frontpoint = temp_tra.GetPosition(splice_idx, 0.0);
            Eigen::Vector2d frontvector = temp_tra.GetVelocity(splice_idx, 0.0);
            m_FrontPoint = frontpoint;
            
            int newx = static_cast<int>((frontpoint.x() - m_OriginX) / m_ResX);
            int newy = static_cast<int>((frontpoint.y() - m_OriginY) / m_ResY);
            
            auto t_start = std::chrono::high_resolution_clock::now();
            m_ReplanAstar.SetStartGoal(static_cast<int>(grid_p.x()), static_cast<int>(grid_p.y()), newx, newy);
            
            if (m_ReplanAstar.FindPath() && (m_ReplanAstar.GetPath().size() * m_ResX) < min_wholereplan * 3.0) {
                for (const auto& pt : m_ReplanAstar.GetPath()) {
                    m_DetourPath.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
                }
                PraticalReplan(p_, v_, frontpoint, frontvector, m_ReplanAstar.GetPath(), splice_idx);
                resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
                double duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
                SetPlanStats("local", duration_ms);
            } else {
                InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
                resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
            }
            respose_ = GetForwardSample(m_lbfgs.GetMincoTrajectory());
        }

        m_ResVec = resvec_;
        m_ResPose = respose_;
        m_ProjectedPoint = m_ResPose; 

        if (m_FrontPoint.norm() > 1e-4) {
            ProjectionResult front_res = m_Project.FindClosestPoint(m_FrontPoint);
            double f_time = 0.0;
            auto new_tra = m_lbfgs.GetMincoTrajectory();
            if (front_res.seg_idx >= 0 && front_res.seg_idx < new_tra.GetNumPoints() - 1) {
                for(int k = 0; k < front_res.seg_idx; ++k) {
                    f_time += new_tra.GetTimes()(k);
                }
                f_time += front_res.t;
                m_FrontTime = f_time;
            } else {
                m_FrontTime = 0.0;
            }
        }

        m_GetRes = true;
        return true;
    }

    bool replanner::PraticalReplan(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_, 
                                   const Eigen::Vector2d& tp_, const Eigen::Vector2d& tv_, 
                                   const std::vector<Eigen::Vector2d>& grid_path_, int splice_idx){
        (void)v_;
        (void)tv_;
        std::vector<Eigen::Vector2d> phys_path;
        phys_path.reserve(grid_path_.size());
        for (const auto& pt : grid_path_) phys_path.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
        
        if(!phys_path.empty()) { phys_path.front() = p_; phys_path.back() = tp_; } 
        else { phys_path.push_back(p_); phys_path.push_back(tp_); }

        std::vector<Eigen::Vector2d> sparse_path;
        sparse_path.push_back(phys_path.front());
        for (size_t i = 1; i < phys_path.size() - 1; ++i) {
            Eigen::Vector2d dir1 = (phys_path[i] - phys_path[i-1]).normalized();
            Eigen::Vector2d dir2 = (phys_path[i+1] - phys_path[i]).normalized();
            if ((dir1 - dir2).norm() > 1e-3) sparse_path.push_back(phys_path[i]);
        }
        sparse_path.push_back(phys_path.back());
        
        double local_time = ComputeWholeTime(sparse_path);
        LBFGS local_lbfgs = m_lbfgs; 
        if (!local_lbfgs.OptimizeLocalPhase2(sparse_path, local_time)) return false;

        const minco& local_minco = local_lbfgs.GetMincoTrajectory();
        std::vector<Eigen::Vector2d> local_pts = local_minco.GetPoints();
        Eigen::VectorXd local_times = local_minco.GetTimes();

        const minco& global_minco = m_lbfgs.GetMincoTrajectory();
        std::vector<Eigen::Vector2d> global_pts = global_minco.GetPoints();
        Eigen::VectorXd global_times = global_minco.GetTimes();

        std::vector<Eigen::Vector2d> spliced_pts;
        std::vector<double> spliced_times_vec;

        for(size_t i = 0; i < local_pts.size(); ++i) spliced_pts.push_back(local_pts[i]);
        for(int i = 0; i < local_times.size(); ++i) spliced_times_vec.push_back(local_times(i));

        for(size_t i = splice_idx + 1; i < global_pts.size(); ++i) spliced_pts.push_back(global_pts[i]);
        for(int i = splice_idx; i < global_times.size(); ++i) spliced_times_vec.push_back(global_times(i));

        Eigen::VectorXd spliced_times(spliced_times_vec.size());
        for(size_t i = 0; i < spliced_times_vec.size(); ++i) spliced_times(i) = spliced_times_vec[i];

        m_lbfgs.SetCustomTrajectory(spliced_pts, spliced_times);
        SetTrajectory();
        return true;
    }

    bool replanner::UpdateGoal(const Eigen::Vector2d& p_, const Eigen::Vector2d& new_goal_, const Eigen::Vector2d& new_vel_) {
        m_GetRes = false;
        ResetPlanStats();
        
        Eigen::Vector2d valid_goal = ClosestFreeGoal(new_goal_);

        double dist_sq = (valid_goal - m_TargetPoint).squaredNorm();
        auto temp_tra = m_lbfgs.GetMincoTrajectory();
        int num_pts = temp_tra.GetNumPoints();
        
        Eigen::Vector2d grid_new_target((valid_goal.x() - m_OriginX) / m_ResX, (valid_goal.y() - m_OriginY) / m_ResY);

        if (dist_sq > min_wholereplan_sq) {
            InitPoint(p_.x(), p_.y(), valid_goal.x(), valid_goal.y());
        } else {
            auto t_start = std::chrono::high_resolution_clock::now();
            ProjectionResult close_pos = m_Project.FindClosestPoint(p_);
            int robot_idx = std::max(0, close_pos.seg_idx);
            int splice_idx = std::max(robot_idx, num_pts - front_replan - 2);

            Eigen::Vector2d splice_pos = temp_tra.GetPosition(splice_idx, 0.0);
            Eigen::Vector2d splice_vel = temp_tra.GetVelocity(splice_idx, 0.0);
            
            int sx = static_cast<int>((splice_pos.x() - m_OriginX) / m_ResX);
            int sy = static_cast<int>((splice_pos.y() - m_OriginY) / m_ResY);
            int gx = static_cast<int>(grid_new_target.x());
            int gy = static_cast<int>(grid_new_target.y());
            
            m_ReplanAstar.SetStartGoal(sx, sy, gx, gy);
            
            if (m_ReplanAstar.FindPath() && (m_ReplanAstar.GetPath().size() * m_ResX) < min_wholereplan * 3.0) {
                if (!PraticalReplanGoal(splice_pos, splice_vel, valid_goal, new_vel_, m_ReplanAstar.GetPath(), splice_idx)) {
                    InitPoint(p_.x(), p_.y(), valid_goal.x(), valid_goal.y());
                } else {
                    double duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
                    SetPlanStats("local", duration_ms);
                }
            } else {
                InitPoint(p_.x(), p_.y(), valid_goal.x(), valid_goal.y());
            }
        }
        
        m_TargetPoint = valid_goal;

        ProjectionResult final_close = m_Project.FindClosestPoint(p_);
        if (final_close.seg_idx >= 0) {
            m_ResVec = m_lbfgs.GetMincoTrajectory().GetVelocity(final_close.seg_idx, final_close.t);
            m_ResPose = final_close.pos;
            m_GetRes = true;
        }
        return true;
    }

    bool replanner::PraticalReplanGoal(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_, 
                                       const Eigen::Vector2d& tp_, const Eigen::Vector2d& tv_, 
                                       const std::vector<Eigen::Vector2d>& grid_path_, int splice_idx) {
        (void)v_;
        (void)tv_;

        std::vector<Eigen::Vector2d> phys_path;
        phys_path.reserve(grid_path_.size());
        for (const auto& pt : grid_path_) phys_path.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
        
        if(!phys_path.empty()) { phys_path.front() = p_; phys_path.back() = tp_; } 
        else { phys_path.push_back(p_); phys_path.push_back(tp_); }

        std::vector<Eigen::Vector2d> sparse_path;
        sparse_path.push_back(phys_path.front());
        for (size_t i = 1; i < phys_path.size() - 1; ++i) {
            Eigen::Vector2d dir1 = (phys_path[i] - phys_path[i-1]).normalized();
            Eigen::Vector2d dir2 = (phys_path[i+1] - phys_path[i]).normalized();
            if ((dir1 - dir2).norm() > 1e-3) sparse_path.push_back(phys_path[i]);
        }
        sparse_path.push_back(phys_path.back());

        double local_time = ComputeWholeTime(sparse_path);
        LBFGS local_lbfgs = m_lbfgs; 
        if (!local_lbfgs.OptimizeLocalPhase2(sparse_path, local_time)) return false;

        const minco& local_minco = local_lbfgs.GetMincoTrajectory();
        std::vector<Eigen::Vector2d> local_pts = local_minco.GetPoints();
        Eigen::VectorXd local_times = local_minco.GetTimes();

        const minco& global_minco = m_lbfgs.GetMincoTrajectory();
        std::vector<Eigen::Vector2d> global_pts = global_minco.GetPoints();
        Eigen::VectorXd global_times = global_minco.GetTimes();

        std::vector<Eigen::Vector2d> spliced_pts;
        std::vector<double> spliced_times_vec;

        for(int i = 0; i <= splice_idx; ++i) spliced_pts.push_back(global_pts[i]);
        for(int i = 0; i < splice_idx; ++i) spliced_times_vec.push_back(global_times(i));
        
        for(size_t i = 1; i < local_pts.size(); ++i) spliced_pts.push_back(local_pts[i]);
        for(int i = 0; i < local_times.size(); ++i) spliced_times_vec.push_back(local_times(i));

        Eigen::VectorXd spliced_times(spliced_times_vec.size());
        for(size_t i = 0; i < spliced_times_vec.size(); ++i) spliced_times(i) = spliced_times_vec[i];

        m_lbfgs.SetCustomTrajectory(spliced_pts, spliced_times);
        SetTrajectory();
        return true;
    }

    Eigen::Vector2d replanner::ClosestFreeGoal(const Eigen::Vector2d& target_) const {
        // 转换物理坐标为网格坐标
        int gx = static_cast<int>((target_.x() - m_OriginX) / m_ResX);
        int gy = static_cast<int>((target_.y() - m_OriginY) / m_ResY);

        if (gx < 0) gx = 0; if (gx >= m_MapLenX) gx = m_MapLenX - 1;
        if (gy < 0) gy = 0; if (gy >= m_MapWeightY) gy = m_MapWeightY - 1;
        if (m_jps.IsFreeGrip(gx, gy)) {
            return target_;
        }

        std::queue<std::pair<int, int>> q;
        std::vector<bool> visited(m_MapLenX * m_MapWeightY, false);

        q.push({gx, gy});
        visited[gy * m_MapLenX + gx] = true;

        const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();

            if (m_jps.IsFreeGrip(cx, cy)) {
                return Eigen::Vector2d(m_OriginX + cx * m_ResX, m_OriginY + cy * m_ResY);
            }

            for (int i = 0; i < 8; ++i) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (nx >= 0 && nx < m_MapLenX && ny >= 0 && ny < m_MapWeightY) {
                    int idx = ny * m_MapLenX + nx;
                    if (!visited[idx]) {
                        visited[idx] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }
        
        // 如果异常（全地图均不可达等）， fallback 返回原始目标点防止崩溃
        return target_;
    }

} // namespace planner_2d
