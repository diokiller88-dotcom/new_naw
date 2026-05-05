#include "constants.hpp"
#include "serialport.hpp"
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/chassis_info.hpp"
#include "custom_msgs/msg/result.hpp"

namespace serial {

constexpr const char* serial_config_path = "/home/hyl/new_nav/algo_master/configs/serial_configs.json";
constexpr float close_distance_threshold = 0.2f;

constexpr const char* topic_chassis_info = "serial/chassis";
constexpr const char* topic_result = "planner/result";
constexpr const char* frame_id_chassis = "chassis";

class AlgoMasterNode : public rclcpp::Node {
public:
    AlgoMasterNode() : Node("algo_master"), sp(nullptr) {
        SerialPortConfig config;
        if (!InitConfigs(serial_config_path, config)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load config: %s", serial_config_path);
            std::exit(-1);
        }
        if (sp.OpenPort(config.portName, config.baudrate, config.parity, config.dataBit, config.stopBit)) {
            RCLCPP_INFO(this->get_logger(), "Serial Port Initialized Successfully.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Initial Serial Port Open Failed. Daemon will keep trying...");
        }

        chassis_pub_ = this->create_publisher<custom_msgs::msg::ChassisInfo>(topic_chassis_info, 10);
        result_sub_  = this->create_subscription<custom_msgs::msg::Result>(
            topic_result, 10, std::bind(&AlgoMasterNode::ResultCallback, this, std::placeholders::_1));

        threads_.emplace_back(&NautilusSerialPort::ReadRawBuf, &sp);
        threads_.emplace_back(&NautilusSerialPort::ProcRawBuf, &sp);
        threads_.emplace_back(&NautilusSerialPort::CheckAndReconnect, &sp);
    }

    ~AlgoMasterNode() {
        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

    void run() {
        sp.setNode(shared_from_this());

        NavRecvFromPLC recv;
        while (rclcpp::ok()) {
            if (sp.msgSerialRecv.Pop(recv)) {
                { 
                    std::lock_guard<std::mutex> lock(mtx_); 
                    last_recv_ = recv; 
                }
                
                auto msg = custom_msgs::msg::ChassisInfo();
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = frame_id_chassis;
                msg.is_valid = true;
                
                msg.speed      = recv.m_Speed;
                msg.gimbal_yaw = recv.m_Yaw;
                msg.target_x   = recv.m_Targetx;
                msg.target_y   = recv.m_Targety;

                msg.trigger_relocation = recv.m_ResetPose;   
                msg.trigger_target     = recv.m_ResetTarget; 
                
                chassis_pub_->publish(msg);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(serial_proc_interval_ms));
            }
            rclcpp::spin_some(shared_from_this());
        }
    }

private:
    void ResultCallback(const custom_msgs::msg::Result::SharedPtr msg) {
        Nav2PLCSend send;
        if (msg->is_valid) {
            send.m_TargetX    = msg->res_pose_x;
            send.m_TargetY    = msg->res_pose_y;
            send.m_TargetvX   = msg->res_vel_x;
            send.m_TargetvY   = msg->res_vel_y;
            send.m_YawDiff    = msg->yaw_diff;
            send.m_TargetVYaw = msg->vyaw;
            send.m_FindPath   = true;
            send.m_IsTarget   = true;

            NavRecvFromPLC current_recv;
            { 
                std::lock_guard<std::mutex> lock(mtx_); 
                current_recv = last_recv_; 
            }
            float dist = std::hypot(current_recv.m_Targetx - msg->res_pose_x, 
                                    current_recv.m_Targety - msg->res_pose_y);
            send.m_IsClose = (dist < close_distance_threshold);
        } else {
            send.m_IsTarget = false;
            send.m_FindPath = false;
            send.m_IsClose  = false;
        }
        sp.Send(send);
    }

    NautilusSerialPort sp;
    std::vector<std::thread> threads_;
    rclcpp::Publisher<custom_msgs::msg::ChassisInfo>::SharedPtr chassis_pub_;
    rclcpp::Subscription<custom_msgs::msg::Result>::SharedPtr result_sub_;
    
    NavRecvFromPLC last_recv_;
    std::mutex mtx_;
};

} // namespace serial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial::AlgoMasterNode>(); 
    node->run();                                            
    rclcpp::shutdown();
    return 0;
}