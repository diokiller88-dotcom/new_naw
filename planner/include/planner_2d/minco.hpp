#pragma once

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace planner_2d {

    struct Order3_Polynomial {
        double a0_x; double a1_x; double a2_x; double a3_x;
        double a0_y; double a1_y; double a2_y; double a3_y;
    };

    class minco {
    public:
        bool Init(const std::vector<Eigen::Vector2d>& points);
        bool SetPath(const Eigen::VectorXd& times);

        inline std::vector<Order3_Polynomial> GetPath() const { return m_Path; }
        inline Eigen::VectorXd GetTimes() const { return m_Times; }
        
        // 暴露出内部锚点用于重规划缝合
        inline std::vector<Eigen::Vector2d> GetPoints() const { return m_Points; }
        inline size_t GetNumPoints() const { return m_Points.size(); }

        void ComputeGradient(const Eigen::MatrixXd& dCost_dCoeffs, Eigen::VectorXd& gradient) const;

        Eigen::Vector2d GetPosition(int seg_idx, double t) const;
        Eigen::Vector2d GetVelocity(int seg_idx, double t) const;
        Eigen::Vector2d GetAcceleration(int seg_idx, double t) const;

    private:
        std::vector<Order3_Polynomial> m_Path;
        std::vector<Eigen::Vector2d>   m_Points;
        Eigen::VectorXd                m_Times;

        Eigen::SparseMatrix<double>    m_MatrixA;
        Eigen::MatrixXd                m_RhsB;
        Eigen::MatrixXd                m_Coeffs; 
    };

} // namespace planner_2d