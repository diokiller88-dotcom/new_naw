#pragma once
#include "tra_project.hpp"
#include "lbfgs.hpp"
#include "jps.hpp"
#include "a_star.hpp"
#include <Eigen/Dense>
#include <vector>

namespace planner_2d {
    constexpr double max_speed = 6.0;
    constexpr double max_acc = 3.0;
    constexpr double acc_dist = max_speed * max_speed / max_acc;
    constexpr double min_particalreplan = 0.5;
    constexpr double min_wholereplan = 1.4;
    constexpr int front_replan = 7;
    constexpr double safe_dist = 0.6; 

    class replanner {
    public:
        replanner(const jps& jps_ = jps(), const LBFGS& lbfgs_ = LBFGS(), const project& project_ = project());
        bool InitMapWithESDF(const std::vector<int>& occupancy_map, const std::vector<double>& esdf_map, int width, int height,
                             double res_x = 0.05, double res_y = 0.05, double origin_x = -30.0, double origin_y = -30.0);
                             
        bool InitPoint(const double& source_x_, const double& source_y_, const double& target_x_, const double& target_y_);
        bool SetTrajectory();
        
        // 双端重规划入口
        bool Update(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_);
        bool UpdateGoal(const Eigen::Vector2d& p_, const Eigen::Vector2d& new_goal_, const Eigen::Vector2d& new_vel_);
        
        double ComputeWholeTime(const std::vector<Eigen::Vector2d>& path_);

        // 双端重规划内部缝合逻辑
        bool PraticalReplan(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_, const Eigen::Vector2d& tp_, const Eigen::Vector2d& tv_,
                            const std::vector<Eigen::Vector2d>& grid_path_, int splice_idx);
        bool PraticalReplanGoal(const Eigen::Vector2d& p_, const Eigen::Vector2d& v_, const Eigen::Vector2d& tp_, const Eigen::Vector2d& tv_,
                                const std::vector<Eigen::Vector2d>& grid_path_, int splice_idx);

        const minco& GetTrajectory() const { return m_lbfgs.GetMincoTrajectory(); }

        std::vector<Eigen::Vector2d> GetJpsGridPath() const { return m_jps.GetEigenPath(); }
        std::vector<Eigen::Vector2d> GetPhase1PhysPoints() const { return m_lbfgs.GetPhase1Points(); }
        Eigen::Vector2d GetResPose()const {return m_ResPose;}
        Eigen::Vector2d GetResVec()const {return m_ResVec;}
        bool IsGetRes()const {return m_GetRes;}

        double GetTrajStartTime(const Eigen::Vector2d& p_) const {
            ProjectionResult res = m_Project.FindClosestPoint(p_);
            if(res.seg_idx < 0) return 0.0;
            double t_sum = 0.0;
            Eigen::VectorXd times = m_lbfgs.GetTimes();
            for(int i = 0; i < res.seg_idx; ++i) t_sum += times(i);
            return t_sum + res.t;
        }

    private:
        jps m_jps;
        a_star m_ReplanAstar;
        LBFGS m_lbfgs;
        project m_Project;
        
        double m_ResX = 0.05; double m_ResY = 0.05;
        double m_OriginX = -30.0; double m_OriginY = -30.0;
        int m_MapLenX = 0; int m_MapWeightY = 0;
        Eigen::Vector2d m_SourcePoint, m_TargetPoint;
        Eigen::Vector2d m_Position;
        Eigen::Vector2d m_ResPose,m_ResVec;
        bool m_GetRes=false;
    };
} // namespace planner_2d