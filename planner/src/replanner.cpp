#include "planner_2d/replanner.hpp"
#include <cmath>
#include <spdlog/spdlog.h>

namespace planner_2d {

    replanner::replanner(const jps& jps_, const LBFGS& lbfgs_, const project& project_)
        : m_jps(jps_), m_lbfgs(lbfgs_), m_Project(project_) ,m_ResPose(Eigen::Vector2d::Zero()),m_ResVec(Eigen::Vector2d::Zero()){}

    bool replanner::InitMapWithESDF(const std::vector<int>& occupancy_map, const std::vector<double>& esdf_map, int width, int height, double res_x, double res_y, double origin_x, double origin_y){
        m_ResX = res_x; m_ResY = res_y; m_OriginX = origin_x; m_OriginY = origin_y; m_MapLenX = width; m_MapWeightY = height;
        m_GetRes = false;
        
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

        for (int y = 0; y < local_h; ++y) {
            for (int x = 0; x < local_w; ++x) {
                int gx = start_x + x;
                int gy = start_y + y;
                if (gx >= 0 && gx < m_MapLenX && gy >= 0 && gy < m_MapWeightY) {
                    int g_idx = gy * m_MapLenX + gx;
                    int l_idx = y * local_w + x;
                    m_GlobalOccupancy[g_idx] = local_occ[l_idx];
                    m_GlobalEsdf[g_idx] = local_esdf[l_idx];
                }
            }
        }
        InitMapWithESDF(m_GlobalOccupancy, m_GlobalEsdf, m_MapLenX, m_MapWeightY, m_ResX, m_ResY, m_OriginX, m_OriginY);
    }

    bool replanner::InitPoint(const double& source_x_, const double& source_y_, const double& target_x_, const double& target_y_){
        m_GetRes = false;
        m_SourcePoint.x() = source_x_; m_SourcePoint.y() = source_y_; m_TargetPoint.x() = target_x_; m_TargetPoint.y() = target_y_;
        int sx = static_cast<int>((source_x_ - m_OriginX) / m_ResX), sy = static_cast<int>((source_y_ - m_OriginY) / m_ResY);
        int tx = static_cast<int>((target_x_ - m_OriginX) / m_ResX), ty = static_cast<int>((target_y_ - m_OriginY) / m_ResY);
        if (!m_jps.InitPoint(sx, sy, tx, ty)) return false;
        if (!m_jps.SetPath()) return false;

        std::vector<Eigen::Vector2d> grid_path = m_jps.GetEigenPath();
        std::vector<Eigen::Vector2d> phys_path;
        phys_path.reserve(grid_path.size());
        for (const auto& pt : grid_path) phys_path.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
        double time = ComputeWholeTime(phys_path);
        if (!m_lbfgs.InitXState(phys_path, {}, time)) return false;
        if (!m_lbfgs.Optimize()) spdlog::warn("[Replanner] LBFGS returned false.");
        SetTrajectory(); 
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

        if (close_pos.dist_sq > min_wholereplan) {
            InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
            resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
            respose_ = m_lbfgs.GetMincoTrajectory().GetPosition(1, 0.0);
            m_FrontPoint = respose_;
        }
        else if (nostop && close_pos.dist_sq <= min_particalreplan) {
            Eigen::Vector2d frontpoint = temp_tra.GetPosition(splice_idx, 0.0);
            m_FrontPoint = frontpoint; 
            
            Eigen::Vector2d grid_front((frontpoint.x() - m_OriginX) / m_ResX, (frontpoint.y() - m_OriginY) / m_ResY);

            if (m_jps.IsNonStop(grid_proj, grid_front)) {
                resvec_ += temp_vec; 
            } else {
                Eigen::Vector2d frontvector = temp_tra.GetVelocity(splice_idx, 0.0);
                int sx = static_cast<int>(grid_proj.x()), sy = static_cast<int>(grid_proj.y());
                int gx = static_cast<int>(grid_front.x()), gy = static_cast<int>(grid_front.y());

                m_ReplanAstar.SetStartGoal(sx, sy, gx, gy);
                if (m_ReplanAstar.FindPath() && (m_ReplanAstar.GetPath().size() * m_ResX) < min_wholereplan * 3.0) {
                    for (const auto& pt : m_ReplanAstar.GetPath()) {
                        m_DetourPath.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
                    }
                    PraticalReplan(p_, v_, frontpoint, frontvector, m_ReplanAstar.GetPath(), splice_idx);
                    resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
                    resvec_ *= 0.5; 
                } else {
                    InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
                    resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
                }
                respose_ = m_lbfgs.GetMincoTrajectory().GetPosition(1, 0.0);
            }
        }
        else if (!nostop || (nostop && close_pos.dist_sq > min_particalreplan)) {
            Eigen::Vector2d frontpoint = temp_tra.GetPosition(splice_idx, 0.0);
            Eigen::Vector2d frontvector = temp_tra.GetVelocity(splice_idx, 0.0);
            m_FrontPoint = frontpoint;
            
            int newx = static_cast<int>((frontpoint.x() - m_OriginX) / m_ResX);
            int newy = static_cast<int>((frontpoint.y() - m_OriginY) / m_ResY);
            
            m_ReplanAstar.SetStartGoal(static_cast<int>(grid_p.x()), static_cast<int>(grid_p.y()), newx, newy);
            
            if (m_ReplanAstar.FindPath() && (m_ReplanAstar.GetPath().size() * m_ResX) < min_wholereplan * 3.0) {
                for (const auto& pt : m_ReplanAstar.GetPath()) {
                    m_DetourPath.emplace_back(m_OriginX + pt.x() * m_ResX, m_OriginY + pt.y() * m_ResY);
                }
                PraticalReplan(p_, v_, frontpoint, frontvector, m_ReplanAstar.GetPath(), splice_idx);
                resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
            } else {
                InitPoint(p_.x(), p_.y(), m_TargetPoint.x(), m_TargetPoint.y());
                resvec_ = m_lbfgs.GetMincoTrajectory().GetVelocity(0, 0.0);
            }
            respose_ = m_lbfgs.GetMincoTrajectory().GetPosition(1, 0.0);
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
        double dist_sq = (new_goal_ - m_TargetPoint).squaredNorm();
        auto temp_tra = m_lbfgs.GetMincoTrajectory();
        int num_pts = temp_tra.GetNumPoints();
        
        Eigen::Vector2d grid_new_target((new_goal_.x() - m_OriginX) / m_ResX, (new_goal_.y() - m_OriginY) / m_ResY);

        if (dist_sq > min_wholereplan) {
            InitPoint(p_.x(), p_.y(), new_goal_.x(), new_goal_.y());
        } else {
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
                PraticalReplanGoal(splice_pos, splice_vel, new_goal_, new_vel_, m_ReplanAstar.GetPath(), splice_idx);
            } else {
                InitPoint(p_.x(), p_.y(), new_goal_.x(), new_goal_.y());
            }
        }
        
        m_TargetPoint = new_goal_;

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

} // namespace planner_2d