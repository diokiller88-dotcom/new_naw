colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash 
ros2 run relocation test_node