#pragma once

#include <vector>
#include <Eigen/Dense>
#include "minco.hpp"

namespace planner_2d {

    constexpr int    lbfgs_max_size          = 5;      
    constexpr int    lbfgs_max_firsti_time   = 40;     
    constexpr int    lbfgs_max_secondi_time  = 40;     
    constexpr double lbfgs_min_i_value       = 1e-4;   
    constexpr double lbfgs_step_len          = 1.0;    
    constexpr double lbfgs_armijo            = 1e-4;   
    constexpr double lbfgs_backtrack         = 0.5;    
    constexpr double lbfgs_fatal_cost        = 86.0;   
    constexpr double lbfgs_sample_dist       = 1.0;    
    constexpr double lbfgs_min_minco_time    = 0.05;   
    constexpr double lbfgs_max_i_move        = 2.0;    
    constexpr double lbfgs_max_i_durationtime= 20.0;   

    class LBFGS {
    public:
        bool InitMap(const std::vector<float>& cost_map, int width, int height, 
                     double res_x, double res_y, double origin_x, double origin_y);
        bool UpdateMapPatch(const std::vector<float>& local_cost_map, int local_w, int local_h,
                            double local_origin_x, double local_origin_y);
        
        bool InitXState(const std::vector<Eigen::Vector2d>& points, const std::vector<double>& init_times = {}, double total_time = 10.0);
        bool Optimize();

        std::vector<Eigen::Vector2d> GetTrajectorySamples(double dt = 0.1) const;
        std::vector<Eigen::Vector2d> GetPhase1Points() const { return m_Phase1Points; }

        const minco& GetMincoTrajectory() const { return m_Minco; }
        Eigen::VectorXd GetTimes() const { return m_Times; }

        // 重规划专用打磨接口
        bool OptimizeLocalPhase2(const std::vector<Eigen::Vector2d>& local_path, double total_time);
        // 全局缝合强行写入接口
        void SetCustomTrajectory(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& times);

    private:
        bool DoLBFGSIterations(int max_iter);
        double ComputeCostAndGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad);
        bool ComputeSearchDirection(Eigen::VectorXd& direction);
        bool LineSearch(double& step, const Eigen::VectorXd& direction, const Eigen::VectorXd& grad, double cost, Eigen::VectorXd& x);
        bool CheckLineOfSight(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
        bool RepairPathAStar(const Eigen::Vector2d& start_phys, const Eigen::Vector2d& goal_phys, std::vector<Eigen::Vector2d>& detour_path) const;
        float QueryCost(const Eigen::Vector2d& point) const;
        double QueryAdaptiveSampleDist(const Eigen::Vector2d& point) const;
        void BuildAdaptiveSamples(const std::vector<Eigen::Vector2d>& source_path, std::vector<Eigen::Vector2d>& dense_pts);

        std::vector<float> m_CostMap;
        int m_MapWidth  = 0; int m_MapHeight = 0;
        double m_ResX = 0.05; double m_ResY = 0.05;
        double m_InvResX = 20.0; double m_InvResY = 20.0;
        double m_OriginX = 0.0; double m_OriginY = 0.0;

        minco m_Minco; 
        Eigen::Vector2d m_StartPoint; Eigen::Vector2d m_EndPoint;
        Eigen::VectorXd m_X; 
        int m_NumInteriorPoints;
        Eigen::VectorXd m_Times;
        double m_TotalTime;
        Eigen::VectorXd m_Gradient;
        std::vector<Eigen::VectorXd> m_HistoryS; std::vector<Eigen::VectorXd> m_HistoryY;
        std::vector<Eigen::VectorXd> m_HistoryG; std::vector<double> m_HistoryRho;

        bool m_IsOptimized = false;
        int m_OptimizePhase = 1;

        std::vector<Eigen::Vector2d> m_Phase1Points;
        std::vector<Eigen::Vector2d> m_Phase2InitPoints; 
        std::vector<Eigen::Vector2d> m_DensePointCache;

        double m_w1_obstacle = 200.0; 
        double m_w1_length   = 5.0;   
        double m_w2_obstacle = 40.0;  
        double m_w2_length   = 0.0;
        double m_w2_smooth   = 200.0; 
        double m_w2_anchor   = 2.0;   
    };

} // namespace planner_2d
