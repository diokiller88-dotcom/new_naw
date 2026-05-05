#include "planner_2d/minco.hpp"
#include <spdlog/spdlog.h>

namespace planner_2d {

    bool minco::Init(const std::vector<Eigen::Vector2d>& points) {
        if (points.size() < 2) {
            spdlog::warn("[minco] Need at least 2 points");
            return false;
        }
        m_Points = points;
        m_Times.resize(points.size() - 1);
        m_Times.setOnes();
        m_Path.clear();
        return true;
    }

    bool minco::SetPath(const Eigen::VectorXd& times) {
        if (times.size() != static_cast<int>(m_Points.size()) - 1) {
            spdlog::error("[minco] Times size mismatch");
            return false;
        }
        m_Times = times;

        const int K = static_cast<int>(m_Points.size()) - 1;
        m_Coeffs.resize(4 * K, 2);
        m_Coeffs.setZero();

        for (int i = 0; i < K; ++i) {
            double T_i = m_Times(i);
            if (T_i <= 1e-6) T_i = 1e-6;

            double T2 = T_i * T_i;
            double T3 = T2 * T_i;

            Eigen::Vector2d p0 = m_Points[i];
            Eigen::Vector2d p1 = m_Points[i + 1];
            Eigen::Vector2d v0 = Eigen::Vector2d::Zero();
            Eigen::Vector2d v1 = Eigen::Vector2d::Zero();

            if (i == 0) {
                v0.setZero();
                if (K > 1) {
                    double T_next = m_Times(i + 1);
                    v1 = (m_Points[i + 2] - p0) / (T_i + T_next);
                }
            } else if (i == K - 1) {
                v1.setZero();
                double T_prev = m_Times(i - 1);
                v0 = (p1 - m_Points[i - 1]) / (T_prev + T_i);
            } else {
                double T_prev = m_Times(i - 1);
                double T_next = m_Times(i + 1);
                v0 = (p1 - m_Points[i - 1]) / (T_prev + T_i);
                v1 = (m_Points[i + 2] - p0) / (T_i + T_next);
            }

            Eigen::Vector2d a0 = p0;
            Eigen::Vector2d a1 = v0;
            Eigen::Vector2d a2 = (3.0 / T2) * (p1 - p0) - (1.0 / T_i) * (v1 + 2.0 * v0);
            Eigen::Vector2d a3 = -(2.0 / T3) * (p1 - p0) + (1.0 / T2) * (v1 + v0);

            int base = 4 * i;
            m_Coeffs(base, 0)     = a0.x(); m_Coeffs(base, 1)     = a0.y();
            m_Coeffs(base + 1, 0) = a1.x(); m_Coeffs(base + 1, 1) = a1.y();
            m_Coeffs(base + 2, 0) = a2.x(); m_Coeffs(base + 2, 1) = a2.y();
            m_Coeffs(base + 3, 0) = a3.x(); m_Coeffs(base + 3, 1) = a3.y();
        }

        m_Path.resize(K);
        for (int i = 0; i < K; ++i) {
            int base = 4 * i;
            m_Path[i] = {
                m_Coeffs(base, 0), m_Coeffs(base + 1, 0), m_Coeffs(base + 2, 0), m_Coeffs(base + 3, 0),
                m_Coeffs(base, 1), m_Coeffs(base + 1, 1), m_Coeffs(base + 2, 1), m_Coeffs(base + 3, 1)
            };
        }

        return true;
    }

    void minco::ComputeGradient(const Eigen::MatrixXd& dCost_dCoeffs, Eigen::VectorXd& gradient) const {
        const int num_seg = static_cast<int>(m_Path.size());
        gradient = Eigen::VectorXd::Zero(num_seg);
    }

    Eigen::Vector2d minco::GetPosition(int seg_idx, double t) const {
        if (seg_idx < 0 || seg_idx >= static_cast<int>(m_Path.size())) return Eigen::Vector2d::Zero();
        const auto& poly = m_Path[seg_idx];
        double t2 = t * t;
        double t3 = t2 * t;
        return Eigen::Vector2d(
            poly.a0_x + poly.a1_x * t + poly.a2_x * t2 + poly.a3_x * t3,
            poly.a0_y + poly.a1_y * t + poly.a2_y * t2 + poly.a3_y * t3
        );
    }

    Eigen::Vector2d minco::GetVelocity(int seg_idx, double t) const {
        if (seg_idx < 0 || seg_idx >= static_cast<int>(m_Path.size())) return Eigen::Vector2d::Zero();
        const auto& poly = m_Path[seg_idx];
        double t2 = t * t;
        return Eigen::Vector2d(
            poly.a1_x + 2.0 * poly.a2_x * t + 3.0 * poly.a3_x * t2,
            poly.a1_y + 2.0 * poly.a2_y * t + 3.0 * poly.a3_y * t2
        );
    }

    Eigen::Vector2d minco::GetAcceleration(int seg_idx, double t) const {
        if (seg_idx < 0 || seg_idx >= static_cast<int>(m_Path.size())) return Eigen::Vector2d::Zero();
        const auto& poly = m_Path[seg_idx];
        return Eigen::Vector2d(
            2.0 * poly.a2_x + 6.0 * poly.a3_x * t,
            2.0 * poly.a2_y + 6.0 * poly.a3_y * t
        );
    }

} // namespace planner_2d