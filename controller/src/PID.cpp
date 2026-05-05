#include"controller/PID.hpp"

namespace controller{
    bool PIDcontroller::InitPID(const double&p,const double& i,const double&d){
        m_IsInit = false;
        if(std::isnan(p) || std::isnan(i) || std::isnan(d)){
            spdlog::warn("set pid fail");
            return false;
        }
        m_P = p;
        m_I = i;
        m_D = d;
        m_valueI = 0.;
        m_ErrorI = 0.;
        m_IsInitPID = true;
        m_IsInitValue = false;
        m_LastError = 0.;
        return true;
    }



    double PIDcontroller::update(const double&setPoint,const double&targetPoint,const double&dt,const double frontspeed){
        double error_ = targetPoint - setPoint;
        m_ErrorI += dt * error_;
        if(m_valueI > maxi_pid){
            m_valueI = maxi_pid;
        }else if(m_valueI < mini_pid){
            m_valueI = mini_pid;
        }
        double valuedt_ = 0.;
        if(m_IsInitValue)
        {
            valuedt_ = (error_ - m_LastError)/(dt + 1e-9);
        }
        m_IsInitValue = true;    
        m_LastError = error_;
        double res_ = error_ * m_P + m_ErrorI * m_I - m_D * valuedt_ + frontspeed * dt;
        if(res_ > maxout){
            res_ = maxout;
        }else if(res_ < minout){
            res_ = minout;
        }
        return res_;
    }



    









}