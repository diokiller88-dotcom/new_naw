#pragma once
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include "custom_msgs/msg/chassis_info.hpp"
#include "custom_msgs/msg/result.hpp"

namespace serial {

using json = nlohmann::json;
constexpr double pi = 3.14159265358979323846;
constexpr double degree_to_rad = pi / 180.0;
constexpr double rad_to_degree = 180.0 / pi;

struct SerialPortConfig {
    std::string portName;
    int baudrate, parity, dataBit, stopBit, synchronize, sendInterval;
};

inline bool InitConfigs(const std::string& pathToConfig, SerialPortConfig& config) {
    std::ifstream ifs(pathToConfig);
    if (!ifs.is_open()) return false;
    json data = json::parse(ifs);
    auto s = data["serialPort"];
    config.portName = s["portName"];
    config.baudrate = s["baudrate"];
    config.parity = s["parity"];
    config.dataBit = s["dataBit"];
    config.stopBit = s["stopBit"];
    config.synchronize = s["synchronize"];
    config.sendInterval = s["sendInterval"];
    return true;
}

} // namespace serial