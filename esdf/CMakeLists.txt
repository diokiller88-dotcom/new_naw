cmake_minimum_required(VERSION 3.8)
project(esdf)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(custom_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters io) 

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ================= 静态库（核心算法，完全剥离 ROS） =================
add_library(esdf_map_lib STATIC
  src/esdf_map.cpp
  src/feasible_map.cpp
  src/sliding_map.cpp
)

target_link_libraries(esdf_map_lib
  ${PCL_COMMON_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  tbb
  fmt::fmt
  spdlog::spdlog
)

ament_target_dependencies(esdf_map_lib
  Eigen3
  spdlog
)

# ================= esdf_node (ROS 接口节点) =================
add_executable(esdf_node src/esdf_node.cpp)

target_link_libraries(esdf_node
  esdf_map_lib
  ${PCL_COMMON_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  tbb
  fmt::fmt
  spdlog::spdlog
)

ament_target_dependencies(esdf_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  custom_msgs
  Eigen3
  spdlog
)

# ================= 测试程序 esdf_test =================
add_executable(esdf_test src/test.cpp)

target_link_libraries(esdf_test
  esdf_map_lib
  ${PCL_COMMON_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_IO_LIBRARIES} 
  ${OpenCV_LIBS}
  tbb
  fmt::fmt
  spdlog::spdlog
  usb-1.0 
)

ament_target_dependencies(esdf_test
  Eigen3
  spdlog
)

install(TARGETS 
  esdf_node
  esdf_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()