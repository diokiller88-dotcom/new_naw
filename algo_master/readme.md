根据你提供的内容，整理如下：

---

# algo_master

> 下位机通信

## 功能概述

沿袭 nautilus 自瞄通信方案并移植到 ROS2。协议详情见 `serial.hpp`，如需修改协议请同步更新 `custom_msgs` 中的自定义消息包。

## 通信协议

### 上位机 → 下位机

```cpp
struct Nav2PLCSend {
    uint8_t m_FrameHead = 0xA5;   // 帧头
    float m_TargetvX{};           // 目标速度 X（车体当前朝向为 X 正方向）
    float m_TargetvY{};           // 目标速度 Y（与 X 正交）
    float m_TargetX{};            // 局部目标点 X（与目标速度同坐标系）
    float m_TargetY{};            // 局部目标点 Y（与目标速度同坐标系）
    float m_YawDiff{};            // YAW 目标差角（实际无用）
    float m_TargetVYaw{};         // YAW 目标差角速度（实际无用）
    bool  m_IsTarget{};           // 目标点是否可用（是否在墙内）
    bool  m_FindPath{};           // 是否寻路成功
    bool  m_IsClose{};            // 是否接近目标
    uint8_t m_FrameTail = 0xAA;  // 帧尾
};
```

### 下位机 → 上位机

```cpp
struct NavRecvFromPLC {
    uint8_t m_FrameHead = 0xA5;
    bool  m_ResetTarget{};        // 获取新目标点
    bool  m_ResetPose{};          // 是否触发重定位
    float m_Speed{};              // 车体速度（由下位机轮式编码器+加速度计卡尔曼获得）
    float m_Yaw{};                // YAW 角度（无用，给 0 即可）
    float m_Targetx{};            // 云台手目标点 X（需联调保证为 MAP 坐标系，地图中心为原点，左为 X 正，下为 Y 正）
    float m_Targety{};            // 云台手目标点 Y（注意同 X）
    uint8_t m_FrameTail = 0xAA;
};
#pragma pack(pop)
```

> **注意**：所有信息（定位、规划等）均采用 MAP 坐标系。一般以地图中心为原点，左为 X 正方向，下为 Y 正方向。

## 运行

```bash
ros2 run algo_master algo_master
```

无 TEST。

---

//@anthur QQ:1526853523 WIT-Nautilus HYL