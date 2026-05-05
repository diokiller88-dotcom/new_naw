std_msgs/Header header

bool is_valid
# 地图基础信息
int32 width
int32 height
float64 res_x
float64 res_y
float64 origin_x
float64 origin_y

# 核心地图数据 (一维展开存储)
int32[] occupancy_array
float64[] esdf_array