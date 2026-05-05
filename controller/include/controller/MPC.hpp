
namespace controller{
    constexpr size_t mpcmax_iter = 10;
    constexpr double max_ax = 3.0;
    constexpr double max_ay = 3.0;
    constexpr double max_ayaw = 1.0;
    constexpr double min_ax = -3.0;
    constexpr double min_ay = -3.0;
    constexpr double min_ayaw = -1.0;
    class mpc{
        public:
            mpc();
            bool ComputeApAndBp(const double&dt_);
            bool ComputeGandg();

        private:
            ////////x,y,yaw,vx,vy,vyaw
            ////////ax,ay,az
            const size_t x_dim,u_dim;
            size_t max_iter;
            Eigen::MatrixXd m_A,m_B;//6*6,6*3
            Eigen::VectorXd m_X,m_U;//6*1,3*1
            Eigen::MatrixXd m_Q,m_R;//6*6,3*3
            Eigen::MatrixXd m_Qpredict,m_Rpredict;//(6*max_iter)*(6*max_iter),(3*max_iter)*(3*max_iter)
            Eigen::MatrixXd m_Apredict,m_Bpredict;//(6*max_iter)*6,(6*max_iter)*(3*max_iter) 
            Eigen::MatrixXd m_Xref,m_Upredict//(6*max_iter) * 6,(3*max_iter) * 3;
            Eigen::MatrixXd m_H,m_F;
            Eigen::MatrixXd m_G,m_gvector;

            double m_LastTime,m_Dt;
            
    };

}//namespace