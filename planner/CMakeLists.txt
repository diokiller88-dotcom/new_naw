cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)   
find_package(custom_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED) # [新增] message_filters 依赖

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(planner_node
  src/jps.cpp
  src/a_star.cpp
  src/minco.cpp
  src/lbfgs.cpp
  src/tra_project.cpp
  src/replanner.cpp
  src/planner_node.cpp
)

target_link_libraries(planner_node
  rclcpp::rclcpp
  fmt::fmt
  spdlog::spdlog
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# 确保所有消息包和 filters 都包含在内
ament_target_dependencies(planner_node
  custom_msgs 
  nav_msgs 
  geometry_msgs 
  visualization_msgs
  message_filters # [新增] message_filters 依赖
)

add_executable(planner_test
  src/test.cpp
  src/a_star.cpp
  src/jps.cpp
  src/minco.cpp
  src/lbfgs.cpp
  src/tra_project.cpp
  src/replanner.cpp
)

target_link_libraries(planner_test
  rclcpp::rclcpp
  fmt::fmt
  spdlog::spdlog
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# 安装可执行文件
install(TARGETS planner_node planner_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()