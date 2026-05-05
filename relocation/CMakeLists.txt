cmake_minimum_required(VERSION 3.8)
project(relocation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 核心包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(custom_msgs REQUIRED) # 自定义消息包
# [新增] 查找 visualization_msgs
find_package(visualization_msgs REQUIRED) 

# 查找第三方包
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# 1. 编译核心算法库
add_library(relocation_core SHARED
  src/icp.cpp
  src/kd_tree.cpp
  src/ann_kdtree.cpp
  src/iris.cpp
  src/location.cpp
)

target_include_directories(relocation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(relocation_core
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX 
)

# 2. 编译测试节点
add_executable(test_node src/test.cpp)
target_link_libraries(test_node relocation_core)

# 3. 编译重定位 ROS 2 节点
add_executable(relocation_node src/relocation_node.cpp)
target_include_directories(relocation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(relocation_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  message_filters
  custom_msgs
  visualization_msgs # [新增] 添加依赖
)
target_link_libraries(relocation_node relocation_core ${PCL_LIBRARIES})

# 安装规则
install(TARGETS relocation_core test_node relocation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()