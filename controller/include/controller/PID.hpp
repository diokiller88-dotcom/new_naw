#pragma once

namespace controller{
    constexpr double maxi_pid = 2.0;
    constexpr double mini_pid = -2.0;
    constexpr double maxout = 6.0;
    constexpr double minout = -6.0;
    class PIDcontroller{
    public:
        void InitPID(const double&p,const double&i,const double&d);
        void update(const double&setPoint,const double&targetPoint,const double&dt,const double frontspeed = 0.);    
        
    private:
        double m_ErrorI,m_LastError;
        double m_P,m_I,m_D;
        bool m_IsInitPID,m_IsInitValue;
    }





}