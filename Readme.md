哨兵导航代码
包括以下包:
---algo_master	    //下位机通信
---controller 	    //无用，暂未实现
---custom_msgs	    //自定义消息包
---esdf       	    //点云转导航地图（占据栅格与ESDF）
---livox_ros_driver2//MID360ROS2驱动
---planner	        //规划器
---point_lio	    //开源里程计
---relocation	    //定位与重定位
暂时没写决策相关

包与包之间高度解耦合，只通过ROS的话题进行通信，可以分开跑。只要满足相应的话题名称类型即可，且esdf,relocation,planner包均有非ROS的test文件。请务必在使用之前运行test文件，具体原因各个包会描述
point_lio和livox相关不再描述，请自行查阅相关的开源文档或者官方文档

注意事项:由于下位机云台坐标系每次上电不一，则采用相对云台坐标系进行。另外，由于里程计与初始位姿不同，这里将所有的坐标系转化为MAP坐标系，即，占据栅格的坐标系下
//@anthur QQ:1526853523 WIT-Nautilus HYL

可能需要的环境opencv,ros2(很多包)，spdlog，json......可以阅读CMAKE和XML文件
运行test和node之前记得检查路径，有一些是绝对路径（因为相对路径不知道为什么读不出来）很多代码都是从头造的轮子，因为开源都是3D导航的，不适配2D的情况，且计算开销和代码复杂度高（悲）

# new_nav
