# WPR系列机器人ROS2仿真工具

## 介绍课程
Bilibili: [机器人操作系统 ROS2 入门教材](https://www.bilibili.com/video/BV1oz421v7tB)  
Youtube: [机器人操作系统 ROS2 入门教材](https://www.youtube.com/watch?v=j0foOvBqQTc)

## 配套教材书籍
《机器人操作系统（ROS2）入门与实践》  
![视频课程](./media/book_1.jpg)
淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 系统版本

- ROS2 Humble (Ubuntu 22.04)

## 兼容性说明

- 当前仓库按 Gazebo Classic 编写，直接依赖 `gazebo_ros_pkgs`、`gazebo_ros`、`gazebo_ros2_control`。
- Ubuntu 24.04 + ROS2 Jazzy 默认使用的是新 Gazebo 技术栈，因此不能直接执行 `install_for_humble.sh` 来安装这些经典 Gazebo 依赖。
- 如果你使用的是 Ubuntu 24.04 + ROS2 Jazzy，需要二选一：
  1. 切换到 Ubuntu 22.04 + ROS2 Humble 后按本文步骤运行。
  2. 将本仓库迁移到 `gz_sim` / `ros_gz` / `gz_ros2_control` 后再在 Jazzy 下编译。

## 使用说明

### 一、 启智ROS机器人
1. 获取源码:
```
cd ~/ros2_ws/src/
git clone https://github.com/6-robot/wpr_simulation2.git
```
2. 安装依赖项:  
ROS2 Humble (Ubuntu 22.04)
```
cd ~/ros2_ws/src/wpr_simulation2/scripts
./install_for_humble.sh
```
3. 编译
```
cd ~/ros2_ws
colcon build --symlink-install
```

简单场景:
```
ros2 launch wpr_simulation2 wpb_simple.launch.py 
```
![wpb_simple pic](./media/wpb_simple.png)

SLAM环境地图创建:
```
ros2 launch wpr_simulation2 slam.launch.py 
ros2 run rqt_robot_steering rqt_robot_steering 
```
![wpb_gmapping pic](./media/wpb_gmapping.png)

Navigation导航:
```
ros2 launch wpr_simulation2 navigation.launch.py 
```
![wpb_navigation pic](./media/wpb_navigation.png)
