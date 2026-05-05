cmake_minimum_required(VERSION 3.8)
project(algo_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED) # 修改为 custom_msgs

find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_executable(algo_master 
  src/algo_master_node.cpp
  src/serialport.cpp
)

target_include_directories(algo_master PRIVATE
  include
)

# ROS 2 依赖
ament_target_dependencies(algo_master
  rclcpp
  custom_msgs
)

# 第三方库链接
target_link_libraries(algo_master
  spdlog::spdlog
  fmt::fmt
  nlohmann_json::nlohmann_json
)

install(TARGETS
  algo_master
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()