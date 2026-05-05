colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run planner planner_test