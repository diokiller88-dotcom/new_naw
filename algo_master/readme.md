下位机通信包，沿用nautilus自瞄的通信移植到ROS2，不多介绍，协议相关在serial.hpp中，如果需要修改协议记得修改custom_msgs自定义的消息包
当前通信包含义：
上位机发送给下位机：
struct Nav2PLCSend {
    uint8_t m_FrameHead = 0xA5;///帧头
    float m_TargetvX{};///目标速度X，即车体当前的朝向为X正方向
    float m_TargetvY{};///目标速度，与X正交
    float m_TargetX{};///局部的目标点的X，与目标速度同坐标系
    float m_TargetY{};///局部的目标点的Y，与目标速度同坐标系
    float m_YawDiff{};///YAW目标差角，其实无用
    float m_TargetVYaw{};///YAW目标差角速度，其实无用
    bool m_IsTarget{};///反馈云台手给的目标点是否可用（有没有在墙里）
    bool m_FindPath{};///是否寻路成功
    bool m_IsClose{};///是否接近目标
    uint8_t m_FrameTail = 0xAA;
};

struct NavRecvFromPLC {
    uint8_t m_FrameHead = 0xA5;
    bool m_ResetTarget{};///获取新的目标点
    bool m_ResetPose{};///是否触发重定位
    float m_Speed{};///车体速度，由下位机获得（轮式编码器+加速度计卡尔曼）
    float m_Yaw{};///YAW的角度，无用，给0即可
    float m_Targetx{};///云台手的目标的X，可能需要联调测试保证目标点为MAP系（所有的信息包括定位规划均采用MAP系，一般是中心为0点，向左为X正方向，向下位Y正方向）
    float m_Targety{};///云台手的目标的Y，注意同X
    uint8_t m_FrameTail = 0xAA;
};
#pragma pack(pop)

无TEST，通过运行ros2 run algo_master algo_master运行


//@anthur QQ:1526853523 WIT-Nautilus HYL