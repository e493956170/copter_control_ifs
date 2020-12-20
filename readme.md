# copter_control_ifs 多旋翼无人机控制接口框架
copter control interface with dji or mavlink devicce like px4/ardupilot

适应大疆（未开发）和mavlink设备

# Introduction 介绍
基于C++的多旋翼无人机控制程序。可用于SLAM、路径跟踪或者其他算法的开发和验证。

A C++ based copter control interface to simplify the repeated work in copter control . One should be easily use this to devolop slam,trajectory planning ,or other state-of-art things.Currently Only mavlink interface has been developed.


# FlightTaskManager 飞行任务管理器
采用虚函数进行开发，方便拓展

Flight tasks manager based on virtual funtions ,so it is easy to extend other motion group by using the api.More work will be done.

# RouteTracker 路径跟踪器
纯跟踪算法

RouteTracker is based on pure pursuit.

# Visualization 视觉可视化
简化了可视化操作，方便可视化路径规划结果以及传感器信息。
Visualization in rviz has been wrapped into some simple steps by template class.So it will be more convenient to show your hard work.

More visualiztion wrapper of rviz  is under working . Also ,the purpose of this is to simplify work not to heavy study content.

无人机避障演示效果

https://www.bilibili.com/video/BV1C7411f75E/

# 依赖 Requirements
sudo apt install ros-melodic-desktop-full mavros opencv2 octomap python3 libboost-dev

pip3 install pyqt5 osqp numpy scipy matplotlib


# 近期的更新 Recently Updates
1、调整了配置文件的读取方式，现在会自动追踪名为"my_catkin_ws"的工作空间，并在其中读取配置文件了。2020年12月19日

# 下一步计划Next Plan
1、解耦框架中的一些视觉模块的嵌入，轻化主程序。

2、将采用反射的方式载入配置文件。


依赖于ROS、MAVROS、ROS_PCL、OPENCV2、OCTOMAP库等

目前动力学优化部分代码未上传至gitee，如无需要删去这部分代码，即可按照你的期望进行飞行。