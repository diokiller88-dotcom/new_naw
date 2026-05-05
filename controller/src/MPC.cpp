#include"controller/MPC.hpp"

namespace controller{
    mac():
        x_dim(6),
        u_dim(3),
    {
        max_iter = mpcmax_iter;
        m_A = Eigen::MatrixXd::Identity(x_dim,x_dim);
        m_B = Eigen::MatrixXd::Zero(x_dim,u_dim);
        m_X = Eigen::VectorXd::Zero(x_dim);
        m_U = Eigen::VectorXd::Zero(u_dim);
        m_Q = Eigen::MatrixXd::Identity(x_dim,x_dim);
        m_R = Eigen::MatrixXd::Identity(u_dim,u_dim);
        m_Qpredict = Eigen::MatrixXd::Zero(x_dim * max_iter,x_dim * max_iter);
        m_Rpredict = Eigen::MatrixXd::Zero(u_dim * max_iter,u_dim * max_iter);
        m_Apredict = Eigen::MatrixXd::Zero(x_dim * max_iter,x_dim);
        m_Bpredict = Eigen::MatrixXd::Zero(x_dim * max_iter,u_dim * max_iter);
        m_Xref = Eigen::MatrixXd::Zero(x_dim * max_iter,x_dim);
        m_Upredict = Eigen::MatrixXd::Zero(u_dim * max_iter,u_dim);
        m_H = Eigen::MatrixXd::Zero(u_dim * max_iter,u_dim * max_iter);
        m_F = Eigen::MatrixXd::Zero(u_dim * max_iter,u_dim * max_iter);
        m_G = Eigen::MatrixXd::Zero(2 * u_dim,u_dim);
        m_gvector = Eigen::VectorXd::Zero(2 * u_dim);
    }

    bool mpc::ComputeApAndBp(const double&dt_){
        m_A(3,0) = dt_;
        m_A(4,1) = dt_;
        m_A(5,2) = dt_;
        m_B(0,3) = dt_;
        m_B(1,4) = dt_;
        m_B(2,5) = dt_;
        Eigen::MatrixXd temp_a = m_A;
        for(size_t i =0;i< max_iter;i++){
            m_Apredict<x_dim,x_dim>(i * x_dim,i*x_dim) = temp_a;
            temp_a * = m_a;
        }
        for(size_t i=1;i<max_iter;i++){
            for(size_t j=0;j<i+1;j++){
                m_Bpredict<x_dim,u_dim>(i * x_dim,j * u_dim) = m_Apredict<x_dim,x_dim>(i * x_dim -1) * m_B;
            }
        }

        m_H = 2 * (m_Bpredict.transpose() * m_Qpredict * m_Bpredict + m_Rpredict);
        m_F = 2 * m_Bpredict.transpose()  * m_Qpredict * (m_Apredict * m_X  - m_Xref);
        return true;
    }

    bool mpc::ComputeGandg(){
        for(size_t i = 0;i < u_dim;i++){
            m_G(i,i) = 1;
            m_G(i + u_dim,i) = -1;
        }
        m_gvector = (max_ax,max_ay,max_ayaw,min_ax,min_ay,min_ayaw);
    }
    


    







}///namespace





