#pragma once
#include "circular_buffer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <array>
#include <atomic>
#include <chrono>

namespace serial {

#pragma pack(push, 1)
struct Nav2PLCSend {
    uint8_t m_FrameHead = 0xA5;
    float m_TargetvX{};
    float m_TargetvY{};
    float m_TargetX{};
    float m_TargetY{};
    float m_YawDiff{};
    float m_TargetVYaw{};
    bool m_IsTarget{};
    bool m_FindPath{};
    bool m_IsClose{};
    uint8_t m_FrameTail = 0xAA;
};

struct NavRecvFromPLC {
    uint8_t m_FrameHead = 0xA5;
    bool m_ResetTarget{};
    bool m_ResetPose{};
    float m_Speed{};
    float m_Yaw{};
    float m_Targetx{};
    float m_Targety{};
    uint8_t m_FrameTail = 0xAA;
};
#pragma pack(pop)

// 串口相关常量
constexpr size_t send_buf_size = sizeof(Nav2PLCSend);
constexpr size_t recv_msg_size = sizeof(NavRecvFromPLC);
constexpr size_t recv_buf_size = 256;

// 串口通信定时参数
constexpr long long serial_timeout_ms = 1000;    // 掉线判定超时时间
constexpr int serial_read_interval_ms = 2;      // 读取线程休眠时间
constexpr int serial_proc_interval_ms = 2;      // 处理线程休眠时间
constexpr int serial_reconnect_sleep_s = 1;     // 重连失败后的等待时间
constexpr int serial_monitor_interval_ms = 100; // 状态监控轮询间隔

using RawBufRecv = std::array<uint8_t, recv_buf_size>;

class NautilusSerialPort {
public:
    explicit NautilusSerialPort(rclcpp::Node::SharedPtr node = nullptr) 
        : msgSerialRecv(1), msgRawBufs(1), port_available(false), node_(node) {}

    bool OpenPort(const std::string& portname, int baudrate, int parity, int databit, int stopbit, int synchronizeflag = 1);
    void ClosePort();
    bool Send(const Nav2PLCSend& payload);
    void ReadRawBuf();
    void ProcRawBuf();
    void CheckAndReconnect();
    void setNode(rclcpp::Node::SharedPtr node) { node_ = node; }

    CircularBuffer<NavRecvFromPLC> msgSerialRecv;
    CircularBuffer<RawBufRecv> msgRawBufs;

private:
    int pHandle = -1;
    std::string m_Portname;
    int m_Baudrate, m_Parity, m_Databit, m_Stopbit, m_Synchronizeflag;
    
    std::atomic_bool port_available;
    std::chrono::steady_clock::time_point last_received;
    rclcpp::Node::SharedPtr node_;

    bool m_Open(const char* portname, int baudrate, int parity, int databit, int stopbit);
    void m_Close(); // 修复缺失的声明
    int m_Send(const void* buf, int len);
    int m_Receive(void* buf, int maxlen);
};

} // namespace serial