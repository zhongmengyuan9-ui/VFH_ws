# VFH_ws文件结构
## 1. 项目介绍
VFH_ws是一个基于ROS的路径规划项目，旨在实现机器人在复杂环境中的避障和路径规划功能。该项目使用了Vector Field Histogram (VFH)算法来实现实时的路径规划和避障。

## 2. 启动项目
VFH_ws分别使用了python和c++编写

### 2.1 启动python版本

基于视觉传感器的避障启动
```bash
roslaunch mower_vfh vfh_gazebo.launch.py 2>&1 | grep -v "blade_trajectory_controller"
``` 

基于雷达传感器的避障启动
```bash
roslaunch mower_vfh_lidar lidar_vfh_gazebo.launch.py 2>&1 | grep -v "blade_trajectory_controller"
```

### 2.2 启动c++版本
c++版本均在mower_vfh_lidar目录下的launch文件夹中vfh_gazebo.launch.py中实现，，通过vfh_type:=lidar或vfh_type:=depth来指定使用视觉传感器还是雷达传感器。源代码cpp文件在各自src文件夹中。

基于视觉传感器的避障启动
```bash
ros2 launch mower_vfh_lidar vfh_gazebo.launch.py vfh_type:=lidar 2>&1 | grep -v "blade_trajectory_controller"
``` 

基于雷达传感器的避障启动
```bash
ros2 launch mower_vfh_lidar vfh_gazebo.launch.py vfh_type:=depth  2>&1 | grep -v "blade_trajectory_controller"
``` 


# 切割电机档位开启与关闭
## 启动一档
ros2 service call /gmower/cutting_motor/control mower_description/srv/CuttingMotorControl "{speed_level: 1}"

## 启动二档
ros2 service call /gmower/cutting_motor/control mower_description/srv/CuttingMotorControl "{speed_level: 2}"

## 启动三档
ros2 service call /gmower/cutting_motor/control mower_description/srv/CuttingMotorControl "{speed_level: 3}"

## 关闭电机
ros2 service call /gmower/cutting_motor/control mower_description/srv/CuttingMotorControl "{speed_level: 0}"
