# Nav Combat Workspace

> 基于 ROS1 的双车对抗机器人完整工作空间。本代码曾参加过全国大学生汽车竞赛，并在全国总决赛瑞士轮小组赛中打出了全胜成绩（击溃多所顶9院校），最终拿到**全国一等奖**。这个仓库记录的是我把巡航、对抗、定位、导航、识别、裁判系统、物资点处理、云台控制和行为树决策真正接起来之后留下来的完整工程结果。
> 本人把一套比赛机器人在真实场地里需要反复联调的核心链路和相关经验都放进来了。底盘和云台的执行、IMU 和雷达接入、地图定位、局部规划、固定路径巡航、视觉检测、裁判系统通信、物资点处理、比赛状态切换，以及上层行为树决策，我都放在了同一套 catkin 工作空间里。这个仓库也是后来回头看自己整套比赛系统时最完整的一次留档。
> 如果这个项目对你有帮助，欢迎点一个 Star，感谢。

<p align="center">
  <img alt="ROS1" src="https://img.shields.io/badge/ROS-1-blue" />
  <img alt="Catkin" src="https://img.shields.io/badge/Build-catkin-orange" />
  <img alt="Language" src="https://img.shields.io/badge/C%2B%2B-Python-green" />
  <img alt="Status" src="https://img.shields.io/badge/Use-Competition%20Robot-red" />
</p>

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 工作空间结构](#2-工作空间结构)
- [3. 系统环境建议](#3-系统环境建议)
- [4. 主要依赖](#4-主要依赖)
- [5. 编译方式](#5-编译方式)
- [6. 启动方式](#6-启动方式)
  - [6.1 整车基础启动](#61-整车基础启动)
  - [6.2 导航定位链路](#62-导航定位链路)
  - [6.3 裁判系统链路](#63-裁判系统链路)
  - [6.4 视觉检测链路](#64-视觉检测链路)
  - [6.5 完整比赛链路](#65-完整比赛链路)
- [7. 核心运行链路](#7-核心运行链路)
- [8. 整体架构解析](#8-整体架构解析)
  - [8.1 执行与状态层](#81-执行与状态层)
  - [8.2 定位与导航层](#82-定位与导航层)
  - [8.3 巡航层](#83-巡航层)
  - [8.4 感知层](#84-感知层)
  - [8.5 比赛逻辑拼接层](#85-比赛逻辑拼接层)
  - [8.6 决策层](#86-决策层)
- [9. 主要功能包展开](#9-主要功能包展开)
  - [9.1 dzactuator](#91-dzactuator)
  - [9.2 dzjudgment](#92-dzjudgment)
  - [9.3 jie_ware](#93-jie_ware)
  - [9.4 move_base](#94-move_base)
  - [9.5 pure_pursuit](#95-pure_pursuit)
  - [9.6 transformer_servo](#96-transformer_servo)
  - [9.7 rknn_pt](#97-rknn_pt)
  - [9.8 laser_aimer](#98-laser_aimer)
  - [9.9 decision_behavior_tree](#99-decision_behavior_tree)
  - [9.10 area_checker](#910-area_checker)
  - [9.11 dzsavepath](#911-dzsavepath)
  - [9.12 其他导航插件与恢复模块](#912-其他导航插件与恢复模块)
  - [9.13 some_msgs](#913-some_msgs)
- [10. 关键 launch 文件](#10-关键-launch-文件)
- [11. 关键话题、服务、动作与 TF](#11-关键话题服务动作与-tf)
  - [11.1 最常盯的控制与状态话题](#111-最常盯的控制与状态话题)
  - [11.2 最常查的服务与动作](#112-最常查的服务与动作)
  - [11.3 TF 关系](#113-tf-关系)
- [12. 地图、路径、模型与资源文件](#12-地图路径模型与资源文件)
- [13. 实际联调时的理解方式](#13-实际联调时的理解方式)
- [14. 二次开发方向](#14-二次开发方向)
- [15. 可能的移植注意事项](#15-可能的移植注意事项)
- [16. 致谢](#16-致谢)
- [17. 维护说明](#17-维护说明)


---

## 1. 项目简介

该项目开源的是一套围绕巡航赛与对抗赛组织起来的 ROS 工作空间。

这个仓库里既有底盘和云台执行链路，也有定位与导航，也有视觉识别、物资点处理、裁判系统通信和行为树决策。对我来说，这套工程的价值一直都不在某一个单独拆出来的模块，而在整条比赛链路终于能稳定跑起来之后呈现出来的整体性。

在这套工程里完成的事情主要包括：

- 底盘串口驱动与底层控制
- 里程计、IMU、激光雷达接入
- 基于静态地图的激光定位
- 基于 `move_base` 的导航规划与局部避障
- 基于 `pure_pursuit` 的固定路径巡航
- 基于 RKNN 的视觉推理链路
- 基于激光输入的辅助瞄准能力
- 裁判系统通信与比赛状态解析
- 物资点状态处理与目标点切换
- 云台动作控制与目标联动
- 基于行为树的高层策略组织

我把第三方功能包、自写的比赛逻辑节点、资源文件和若干实验性模块都放在同一个工作空间里，后续联调时会轻松很多。自己在准备比赛和赛后回收工程时，也一直更喜欢这种组织方式，因为这能把问题尽量留在一套统一的系统里去解决。

这个仓库记录的是本人把系统真正推到比赛环境里之后留下来的工程版本，所以它身上会有很强的真实项目痕迹。你会看到参数调优的痕迹，插件替换过的痕迹，launch 链路迭代过的痕迹，也会看到一些资源路径、模型文件、地图和比赛流程节点都和具体平台耦合得很深。这些东西恰恰也是这类项目最真实的样子。

---

## 2. 工作空间结构

```text
DZACS/
├── build/
├── devel/
├── src/
│   ├── ackermann_recovery/
│   ├── area_checker/
│   ├── clear_costmap_recovery/
│   ├── cmu_local_planner/
│   ├── decision_behavior_tree/
│   ├── decaying_obstacle_layer/
│   ├── donothing_local_planner/
│   ├── dzactuator/
│   ├── dzjudgment/
│   ├── dzsavepath/
│   ├── hybrid_astar_planner/
│   ├── jie_ware/
│   ├── laser_aimer/
│   ├── move_base/
│   ├── navigation_msgs/
│   ├── pure_pursuit/
│   ├── resource/
│   ├── rknn_pt/
│   ├── rplidar_ros/
│   ├── spatio_temporal_voxel_layer/
│   ├── transformer_servo/
│   └── some_msgs/
├── CHANGELOG.rst
└── README.md
```

我在这个工作空间里保留了比赛主线用到的核心模块，也保留了几类不同时期尝试过的规划器、恢复插件和代价地图插件。后面回头看这套工程时，仍然能从目录结构里清楚看到自己当时是怎么一点点把系统往前堆起来的。

通常会把这些目录分成下面几类来看：

### 2.1 自己写的比赛主线模块

- `dzactuator`
- `dzjudgment`
- `transformer_servo`
- `decision_behavior_tree`
- `laser_aimer`
- `area_checker`
- `dzsavepath`

### 2.2 实际在比赛主链路里使用的导航与定位模块

- `move_base`
- `jie_ware`
- `pure_pursuit`
- `clear_costmap_recovery`
- `rplidar_ros`

### 2.3 保留下来的规划与代价地图尝试

- `cmu_local_planner`
- `hybrid_astar_planner`
- `donothing_local_planner`
- `decaying_obstacle_layer`
- `spatio_temporal_voxel_layer`
- `ackermann_recovery`

### 2.4 统一收纳的资源与消息定义

- `resource`
- `some_msgs`
- `navigation_msgs`

这样组织目录不是为了好看，而是为了联调方便。比赛中真正折磨人的地方不是某一个模块能不能单独跑，而是你要能在同一个工作空间里把底盘、导航、感知、裁判系统和高层逻辑一起拉起来，然后在一台车上反复迭代。

---

## 3. 系统环境建议

当前这套工程主要基于下面这套环境整理：

- Ubuntu 20.04
- ROS Noetic
- catkin_make
- C++
- Python

这套工程本身是 ROS1 体系，很多包也明显带着老 ROS 工程的味道，所以我在整理时仍然按照 Noetic 兼容方向去看它。实际上，这已经是在比赛后期比较稳定的一套环境选择。

更推荐你直接在下面这套组合上做第一轮复现：

```text
Ubuntu 20.04 + ROS Noetic + catkin_make
```

不太建议一上来就做这些事情：

- 直接迁移到 ROS2
- 一边改底盘协议一边改导航参数
- 一边换模型一边改行为树
- 一边清理第三方插件一边做首轮联调

在整理这类比赛仓库时，本人始终更偏向先把原始链路跑通，再一点点拆耦合。因为这类工作空间里真正会消耗时间的地方，往往不是编译通过，而是运行后的接口、参数和节奏能不能保持一致。

---

## 4. 主要依赖

### 4.1 ROS 侧依赖

在工程里实际会用到这些 ROS 侧依赖：

- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`
- `tf2_ros`
- `tf_conversions`
- `actionlib`
- `actionlib_msgs`
- `move_base_msgs`
- `message_generation`
- `message_runtime`
- `visualization_msgs`
- `trajectory_msgs`
- `costmap_2d`
- `pluginlib`
- `dynamic_reconfigure`
- `map_server`
- `move_base`
- `imu_filter_madgwick`
- `robot_pose_ekf`
- `rplidar_ros`
- `cv_bridge`
- `image_transport`

### 4.2 系统与第三方依赖

在工程里还实际依赖这些内容：

- OpenCV
- Eigen3
- serial
- RKNN 运行时相关库
- 雷达 SDK
- 部分第三方导航插件
- 行为树库

### 4.3 安装方式

在一台干净环境上重建时，通常会先做最基础的一轮安装：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-move-base \
  ros-noetic-map-server \
  ros-noetic-imu-filter-madgwick \
  ros-noetic-robot-pose-ekf \
  ros-noetic-rviz \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-costmap-2d \
  ros-noetic-pluginlib \
  ros-noetic-dynamic-reconfigure
```

然后再根据编译报错和 launch 缺失依赖去补：

- `teb_local_planner`
- `global_planner`
- `costmap_converter`
- `usb_cam`
- 其他第三方插件依赖

---

## 5. 编译方式

在本地通常直接这样编译：

```bash
cd ~/DZACS
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果发现脚本权限不对，先统一处理：

```bash
find src -type f \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
```

如果只想先验证某几个包，可以会单独看这些位置：

- `package.xml`
- `CMakeLists.txt`
- `launch/`
- `param/`
- `behavior_tree/`
- `resource/`

每次接手这种比赛工程时，建议第一轮都不要急着删目录或者重构命名，而是先让它原样编译一次。因为很多看起来“应该清理”的东西，实际上都可能还在某条链路里发挥作用。

---

## 6. 启动方式

在联调整套系统时，不要只记“一个 launch 全拉起来”。而是会把启动顺序拆成几条链，每条链分别确认，再拼成完整比赛系统。这样出问题的时候排查会快很多。

### 6.1 整车基础启动

先拉起底盘、IMU、雷达、静态 TF 和姿态融合：

```bash
roslaunch dzactuator bringup.launch
```

这个 launch 里我实际做了这些事情：

- 启动 `dzactuator_node`
- 配置串口名和串口波特率
- 发布底盘里程计
- 启动 `imu_filter_madgwick`
- 启动 `rplidar_ros`
- 发布 `base_link -> imu_link` 静态 TF
- 发布 `base_link -> laser` 静态 TF
- 发布 `base_link -> base_center` 静态 TF
- 启动 `robot_pose_ekf`

这里通常会先看下面几个点有没有正常出来：

- `/odom`
- `/imu/data`
- `/scan`
- `map / odom / base_link / laser / imu_link` 这几棵 TF

### 6.2 导航定位链路

底盘基础层正常之后，再启动导航定位：

```bash
roslaunch dzactuator dznavigation.launch
```

在这条链路里拉起了：

- `jie_ware/lidar_loc`
- `jie_ware/lidar_filter`
- `map_server`
- `move_base`
- `clear_costmap_recovery/costmap_cleaner`
- `transformer_servo/cmd_vel_to_angle_node`

在这一步需要特别关心：

- 地图能不能正确加载
- 激光定位能不能稳定锁到地图上
- `move_base` 的 global 和 local costmap 有没有正常起起来
- `/cmd_vel` 有没有持续输出
- 底盘有没有真正响应

### 6.3 裁判系统链路

通常建议单独把裁判系统也确认一遍：

```bash
roslaunch dzactuator dzjudgment.launch
```

这一步需要最关心的是：

- 串口能不能打开
- 裁判系统数据能不能解析
- 物资点、血量、比赛阶段这些字段有没有正常更新

### 6.4 视觉检测链路

单独测视觉时会直接这样拉起：

```bash
roslaunch dzactuator dzrknnpt.launch
```

如果要看完整图像链路，额外把相机节点也拉起来，再看检测输出是否稳定。对于这套工程来说，视觉链路给后面提供的是更接近比赛动作的目标信息，而不是只给一张检测图片。

### 6.5 完整比赛链路

把整套比赛主链路一次性拉起来时，最常用的是这个入口：

```bash
roslaunch dzactuator all_launch.launch
```

这个入口里我已经把这些部分串起来了：

- `dzactuator/bringup.launch`
- `dzactuator/dzjudgment.launch`
- `dzactuator/dznavigation.launch`
- `transformer_servo/launch/bringup.launch`
- `pure_pursuit/launch/bringup.launch`

在这个基础上，再把行为树拉起：

```bash
roslaunch decision_behavior_tree bt_launch.launch
```

如果想更接近自动启动版本，也可以用：

```bash
roslaunch dzactuator autostart_all.launch
```

这条链路里会把基础层、AMCL 入口、导航、裁判系统和视觉都一起拉起来。虽然后面主用的是激光定位，不是 AMCL，但还是把这套入口保留在仓库里了。

---

## 7. 核心运行链路

这里可以简单把这套工程看成下面这条主链路：

**裁判系统 / 物资点状态 / 感知结果 → 行为树决策 → 巡航或导航模块 → 速度与转向转换 → 底盘执行与云台动作 → 状态回传**

如果把主要节点展开，通常可以按这个顺序理解：

1. `dzjudgment` 读入比赛状态
2. `rknn_pt` 或 `laser_aimer` 给出目标相关结果
3. `decision_behavior_tree` 根据当前状态决定机器人要进入哪种动作阶段
4. `pure_pursuit` 或 `move_base` 输出底盘控制量
5. `transformer_servo` 把控制量变成底盘和云台更直接可用的接口
6. `dzactuator` 把控制量通过串口送到底层控制器
7. 底盘把里程计、IMU、电源和其他状态继续回传到 ROS

这条链路里真正难的地方从来都不是“有没有导航”或者“有没有识别”，而是这些东西一旦放进同一套比赛系统里以后，能不能在时序上对齐，在接口上接住，在切换上不互相打架。

比赛机器人最难做的从来都不是单点炫技，而是系统整合。这个仓库对我最大的意义，也恰恰就在这里。

---

## 8. 整体架构解析

## 8.1 执行与状态层

这一层主要放在 `dzactuator` 里。

通过它完成了：

- 底盘串口驱动
- 速度指令下发
- 底层状态回传
- 里程计发布
- IMU 原始输入接入
- EKF 姿态融合前的准备工作
- 雷达与机体相关的基础 bringup

在 `dzactuator/launch/bringup.launch` 里直接拉起了 `dzactuator_node`，并且把串口名、波特率、里程计标定开关、编码器换算参数都放进了参数区。当前仓库里保留的是下面这组配置思路：

```yaml
serial_port_name: /dev/dzactutor
serial_baud_rate: 115200
odom_frame_id: odom
robot_frame_id: base_link
gyro_frame_id: imu_link
calibrate_lineSpeed: 0
ticksPerMeter: 1473
ticksPer2PI: 1111
```

在调底盘时，通常建议优先确认这几件事：

- 串口是否稳定
- `/cmd_vel` 下去之后车是否真的响应
- 编码器回传是否连续
- 里程计方向是否正确
- 转向和角速度符号是否一致

这一层一旦不稳，上层所有逻辑都会跟着一起飘。很多时候你以为是导航没调好，其实根源都在执行层和状态层。

在这一层里把 `imu_filter_madgwick` 和 `robot_pose_ekf` 也一起接上了。对比赛工程来说，更在意的是整个姿态链路够不够顺，而不是单独把某个姿态滤波器吹得多漂亮。只要最后 `odom`、`imu/data` 和 TF 能和车体实际状态对上，就可以基本认为这一层是合格的。

## 8.2 定位与导航层

这一层由 `jie_ware`、`move_base`、代价地图插件和若干恢复策略共同组成。

在 `dznavigation.launch` 里把这层真正需要的主入口都串好了：

- `jie_ware/lidar_loc.launch`
- `jie_ware/lidar_filter.launch`
- `map_server`
- `move_base`
- `clear_costmap_recovery/costmap_cleaner`
- `transformer_servo/cmd_vel_to_angle_node`

### 8.2.1 地图加载

当前的地图入口写在这里：

```bash
/home/duzhong/dzacs/src/resource/maps/newmap.yaml
```

### 8.2.2 定位方式

在这个工程里保留了 AMCL 入口，但主链路里实际更依赖 `jie_ware` 的激光定位能力。

这里感谢好朋友**B站ROS知名教学UP主《机器人工匠阿杰》倾心开源**的2D激光雷达许多常用功能的功能包。

AMCL 是一个保留着备用和对照的入口，而 `lidar_loc` 才是我更看重的那条比赛链。因为比赛现场里更关心快速、稳定、能接住当前雷达和底盘状态的定位结果。

### 8.2.3 全局与局部规划

在当前配置里主要使用：

- `global_planner/GlobalPlanner`
- `teb_local_planner/TebLocalPlannerROS`

这一组搭配是我在这套车上实际跑下来的主配置。仓库里也保留了其他几种选择：

- `hybrid_astar_planner/HybridAStarPlanner`
- `donothing_local_planner/DoNothingLocalPlanner`
- `cmu_local_planner`

这些内容不是单纯的历史残留。把它们留下来，就是因为我当时确实认真试过不同路线，也确实各有优劣，给后续的同学们更多尝试空间。

### 8.2.4 costmap 与恢复策略

在导航层里同时保留了这些能力：

- 常规 global / local costmap
- `clear_costmap_recovery`
- `decaying_obstacle_layer`
- `spatio_temporal_voxel_layer`
- `ackermann_recovery`

### 8.2.5 我对这一层的理解

这一层真正承担的是：

- 让机器人知道自己在哪
- 让机器人知道下一步往哪走
- 让机器人在运动过程中别撞上东西
- 让机器人在局部异常时尽量恢复
- 让上层逻辑能够把导航动作当成可靠模块反复调用

一旦这层稳定了，行为树和比赛逻辑才有资格往上长。否则一切高层决策都只是在假设系统是可控的。

## 8.3 巡航层

这一层我主要放在 `pure_pursuit` 里。

在这套工程里不要把巡航当成一个独立项目，而是把它作为完整比赛系统里的一个动作模块。也就是说，它并不是一直单独运行，而是在某些比赛阶段被行为树主动切进来，承担固定路径段的稳定运动。

我在这部分保留了两个很关键的源码：

- `src/pure_pursuit.cpp`
- `src/path_publisher.cpp`

这条链路在比赛里很重要，因为它解决的是“已知路线段怎么稳定走”的问题。相比每一段都交给导航自由求解，我更愿意在某些场景下直接交给纯跟踪。这样做的好处很直接：

- 路线可控
- 重复性高
- 参数收敛更快
- 现场行为更容易预判

比赛机器人不是全程都需要“最聪明”的规划器，很多时候更重要的是“最稳定的动作执行方式”。`pure_pursuit` 对我来说一直就是这种角色。

## 8.4 感知层

这一层主要由 `rknn_pt` 和 `laser_aimer` 组成。

### 8.4.1 rknn_pt

把 RKNN 推理链路放在这里。当前仓库里你能直接看到：

- `src/det_node.cc`
- `src/det/`
- `model/`
- `lib/`

通过这一层完成的是：

- 相机输入接入
- 模型推理
- 目标检测结果输出
- 目标偏移和后续对抗逻辑的拼接

这一层给上层的并不是“图像”，而是可以直接拿来驱动比赛动作的目标结果。

### 8.4.2 laser_aimer

我在 `laser_aimer` 里保留了：

- `laser_aimer_node.cpp`
- `point_sync_node.cpp`
- `launch/`
- `param/`

这一层对来说更偏向几何侧目标输入。它和视觉不是互相替代关系，而是互相补充关系。比赛现场里最怕的是单一感知源不稳定，所以我更愿意把不同来源的目标信息都接进系统里。

### 8.4.3 对这一层的理解

我不太喜欢把感知层写成“识别模块介绍”。

这一层真正的价值在于：

- 什么时候给出目标
- 给出的目标能不能接入动作链路
- 目标结果是不是足够稳定
- 感知结果和比赛时序能不能对上

感知一旦不能为动作服务，在比赛系统里就很难真正发挥价值。

## 8.5 比赛逻辑拼接层

这一层主要放在 `transformer_servo` 里。

这是整套工程里我个人非常看重的一层。因为它不是一个单独算法包，而是整个系统能不能真正配合起来的中间桥梁。

在这个包里保留的源码包括：

- `cmd_vel_to_angle.cpp`
- `angular_to_angle.cpp`
- `bond.cpp`
- `control_gimbal_to_material.cpp`
- `gimbal_tf.cpp`
- `material_processor.cpp`
- `material_tf_puber.cpp`
- `pub_robot_state.cpp`
- `scan_to_pcl2.cpp`
- `teb_tolerance_changer.cpp`

通过这一层完成的事情包括：

- 把导航输出转成底盘执行更容易接收的接口
- 把角速度或其他控制量进一步处理成底层需要的角度量
- 把物资点状态送进 TF 或者后续逻辑链
- 把云台控制和目标信息接起来
- 在运行中动态修改某些导航参数
- 把若干不同模块之间的接口节奏对齐

## 8.6 决策层

这一层放在 `decision_behavior_tree` 里。

在这个包里保存了多份行为树 XML：

- `main_tree.xml`
- `main_tree_pp.xml`
- `main_tree_default.xml`
- `main_tree_tested.xml`
- `test.xml`

把不同版本都留下来了，因为这对自己回头看项目非常有价值。能很直观地看到我在不同时期对比赛节奏的理解是怎么变化的。

当前这套行为树里，我已经接进来的动作和条件节点包括：

- `MoveBaseAction`
- `PurePursuitAction`
- `SwitchGoalTo`
- `SwitchNextGoal`
- `PubGotoMaterial`
- `WaitVoice`
- `IfDetectTarget`
- `IsAiming`
- `WaitShoot`
- `ChangeGoalTolerance`
- `GimbalCruise`
- `CompareBiggerInt`
- `IfEqualInt`
- `IfInGoalTolerance`
- `DelayAction`
- `PrintAction`

我在 `main_tree.xml` 里实际做的事情就说真实的比赛完整流程。你能看到：

- 开局等待首个物资目标
- 根据 `material_goal_id` 切换目标点
- 动态调整 goal tolerance
- 在 `PurePursuitAction` 和 `MoveBaseAction` 之间做回退
- 等待上层语音或模式切换
- 回到初始区域或公共区域
- 进入云台巡航
- 检测目标并等待射击
- 在物资点逻辑和常规对抗逻辑之间反复切换

行为树最大的价值在于它把整套比赛节奏结构化了。机器人不是一直在“导航”，也不是一直在“识别”，而是在不同任务状态之间切换。行为树正好把这种切换组织起来。

---

## 9. 主要功能包展开

## 9.1 dzactuator

这个包是整套系统的执行核心。

在里面处理了底盘串口、里程计、IMU、基础 bringup、地图保存相关入口和若干整车启动文件。这个包下面最关键的内容包括：

### 9.1.1 关键 launch

- `bringup.launch`
- `dznavigation.launch`
- `dzjudgment.launch`
- `dzrknnpt.launch`
- `all_launch.launch`
- `autostart_all.launch`
- `gmapping.launch`
- `map_saver.launch`
- `dzsavepath.launch`
- `amcl.launch`
- `dzcamera.launch`

### 9.1.2 最常用的入口

在日常联调里最常用的是这几个：

- `bringup.launch`
- `dznavigation.launch`
- `all_launch.launch`

### 9.1.3 在这个包里做的事情

- 拉起底盘驱动
- 配置串口参数
- 配置编码器换算参数
- 配置 IMU 滤波输入输出
- 启动雷达
- 配置静态 TF
- 启动 `robot_pose_ekf`
- 给整车提供一个统一的主入口

### 9.1.4 看这个包的方式

它更像这套系统的底座。比赛里所有上层能力，最后都得落在这个包代表的执行层上。

## 9.2 dzjudgment

这个包里做的是裁判系统通信。

当前源码主要包括：

- `dzjudgment.cpp`
- `judgment.cpp`
- `judgment.h`

通过这个包把比赛状态接进整套系统，包括但不限于：

- 比赛进行阶段
- 物资相关状态
- 机器人当前资源状态
- 其他和规则强相关的信息

在比赛链路里非常看重这一层，因为对抗赛机器人不能只靠自己猜环境状态，很多关键逻辑必须明确依赖裁判系统回传。

## 9.3 jie_ware

我在当前工程里使用它做了这些事情：

- 激光定位
- 激光数据预处理
- 地图与扫描匹配相关能力
- 若干辅助节点

在 `dznavigation.launch` 里，我直接 include 了：

- `jie_ware/launch/lidar_loc.launch`
- `jie_ware/launch/lidar_filter.launch`

这一层的存在让整个导航系统更贴近我的实际车体状态，而不是只停留在标准教程配置上。

## 9.4 move_base

这个包本身不需要过多介绍，但它在我的系统里承担的是导航主框架的角色。

在当前工程里把它和这些内容绑在一起：

- 地图加载
- 激光定位
- global planner
- local planner
- costmap
- 恢复行为
- 后续的控制量转换节点

tips：官方镜像里的`move_base`魔改过（而且写的很屎），只能运行轨迹追踪示教路径的导航流程，因此这里我重定向了原始的`move_base`。建议很多时候可以不用把它当成一个开箱即用的黑盒，而是把它当成能不断插拔和调参的平台。比赛工程真正能跑稳，靠的通常不是“用了 move_base”，而是“你把 move_base 周围的整套东西调顺了”。

## 9.5 pure_pursuit

这个包在比赛系统里承担固定路线巡航的动作角色。

关键源码包括：

- `pure_pursuit.cpp`
- `path_publisher.cpp`

我在行为树里实际把它作为 `PurePursuitAction` 来调用。在上层决策里已经把它视为可独立切换的动作模块，而不是只在底层默默跑着的控制器。

保留这一层，不是为了展示某种轨迹跟踪算法，而是因为比赛里确实有不少路段更适合用固定路径稳定通过。

## 9.6 transformer_servo

在这个包里主要做了：

- 导航控制量到底盘执行接口的转换
- 云台和物资点逻辑的接口拼接
- 材料处理和 TF 发布
- 动态控制参数切换
- 若干中间态消息的整理

## 9.7 rknn_pt

这个包是在视觉推理侧的重要模块。

- `src/det_node.cc`
- `src/det/`
- `model/`
- `lib/`

这意味着你在迁移这个工程时，除了代码本身，还得格外关注：

- 模型文件是否匹配当前硬件
- 推理库是否能在你的平台上加载
- 输入输出接口是否和上层逻辑一致

比赛里可用的视觉系统一定是工程整体的一部分，而不是只在开发机上能跑一张 demo 图。

## 9.8 laser_aimer

这个包里放了激光辅助瞄准相关内容。

核心源码包括：

- `laser_aimer_node.cpp`
- `point_sync_node.cpp`

在这套系统里把激光侧目标输入作为视觉侧的补充，让机器人在部分场景下不至于完全依赖单一路径的感知结果。我在比赛里的策略一直更偏向多源互补，而不是单点押注。

## 9.9 decision_behavior_tree

这个包是高层决策核心。

我在里面保存了行为树 XML、参数文件和主程序入口。当前最关键的文件包括：

- `src/main.cpp`
- `launch/bt_launch.launch`
- `behavior_tree/main_tree.xml`
- `behavior_tree/main_tree_pp.xml`
- `param/material_goal_poses.yaml`
- `param/seq_goal_poses.yaml`

把这部分单独抽出来，就是想让高层任务组织逻辑和底层导航、感知、执行链路保持一定边界。虽然比赛工程里不可能做到绝对解耦，但仍然希望各位对于“比赛怎么打”这件事能有一个清晰的中心位置。

## 9.10 area_checker

这个包里放了一部分区域判断相关内容。

- `launch/`
- `src/`
- `config/`
- `rviz/`

这部分很好理解，在比赛系统里也很自然，因为很多逻辑并不是只看一个目标点，而是要看机器人当前在哪个区域、接下来该进哪个逻辑分支。

## 9.11 dzsavepath

这个包里做了路径保存相关工作。

在 `dzsavepath.launch` 里，直接把保存路径写到了：

```text
/home/duzhong/dzacs/src/resource/road_paths/1.txt
```

## 9.12 其他导航插件与恢复模块

我在工作空间里还保留了这些内容：

- `ackermann_recovery`
- `clear_costmap_recovery`
- `cmu_local_planner`
- `decaying_obstacle_layer`
- `donothing_local_planner`
- `hybrid_astar_planner`
- `spatio_temporal_voxel_layer`

把这些内容留下来，一方面是因为它们在某些阶段确实参与过调试，另一方面也是因为比赛工程从来都不是一步成型的。真正做过完整系统的人都知道，很多最终版本背后都站着一串试过、换过、留下来的方案。希望这些内容也可以给各位排列组合尝试一下，万一有新效果就很赚了。

## 9.13 some_msgs

这个包里放的是整套系统需要的自定义消息。

---

## 10. 关键 launch 文件

在这个仓库里最关键的 launch 文件主要有这些：

### 10.1 `dzactuator/launch/bringup.launch`

在这里拉起整车底层：

- 底盘驱动
- IMU 滤波
- 雷达
- 静态 TF
- EKF

### 10.2 `dzactuator/launch/dznavigation.launch`

在这里拉起导航定位：

- `lidar_loc`
- `lidar_filter`
- `map_server`
- `move_base`
- `costmap_cleaner`
- `cmd_vel_to_angle_node`

### 10.3 `dzactuator/launch/dzjudgment.launch`

在这里拉起裁判系统通信。

### 10.4 `dzactuator/launch/dzrknnpt.launch`

在这里拉起目标检测节点。

### 10.5 `dzactuator/launch/all_launch.launch`

在这里把底层、裁判系统、导航、控制转换和路径巡航拉成一条主链。

### 10.6 `decision_behavior_tree/launch/bt_launch.launch`

在这里把行为树真正拉起来。当前入口里默认使用的是：

```xml
<param name="bt_xml_file" value="main_tree.xml" />
```

### 10.7 `dzactuator/launch/gmapping.launch`

在这里保留了一套建图入口。虽然这套工程现在更强调比赛执行链路，但我还是把建图能力完整留下来了。

### 10.8 `dzactuator/launch/map_saver.launch`

在这里把地图保存路径直接写到资源目录。

---

## 11. 关键话题、服务、动作与 TF

## 11.1 最常盯的控制与状态话题

我个人在联调时最常看的话题主要有这些：

- `/cmd_vel`
- `/cmd_vel_original`
- `/odom`
- `/imu/data`
- `/scan`
- `/move_base_simple/goal`
- `/move_base/status`
- `/material`
- `/judgment`
- `/shoot`
- `/gimbal`
- `/behavior_tree_log`

在我的调试习惯里，这几类话题对应的含义很明确：

### 控制类

- `/cmd_vel`
- `/cmd_vel_original`

可以用它们判断是导航没出控制量，还是控制量被中间转换节点改掉了，还是底盘压根没接住。

### 状态类

- `/odom`
- `/imu/data`
- `/scan`

建议先看这三条能不能稳定，再看导航和感知。因为底层状态一旦不准，上层所有现象都可能是假的。

### 比赛逻辑类

- `/material`
- `/judgment`
- `/behavior_tree_log`

可以用它们判断高层逻辑到底卡在哪个阶段，是物资没更新，还是裁判系统没收到，还是行为树条件没过。

### 执行类

- `/shoot`
- `/gimbal`

建议在对抗动作阶段重点盯这两条，因为这直接决定比赛动作是否真的落地。

## 11.2 最常查的服务与动作

这套工程里真正关键的服务和动作接口，主要围绕这些事情展开：

- 发送导航目标
- 清理代价地图
- 切换比赛状态
- 切换目标点
- 等待检测结果
- 等待射击动作完成

虽然我没有在这里把每一个接口名字全部罗列到最细，但实际调试时的思路其实一直很固定：

- 先确认行为树节点有没有进入目标动作
- 再确认底层 action 或 service 有没有收到请求
- 再确认动作结束条件有没有正确回传

## 11.3 TF 关系

在这套工程里最核心的 TF 关系主要包括：

- `map`
- `odom`
- `base_link`
- `base_center`
- `laser`
- `imu_link`

其中在 `bringup.launch` 里直接发布了这些静态 TF：

- `base_link -> imu_link`
- `base_link -> laser`
- `base_link -> base_center`

tips：调比赛的机器人系统时，TF 一直都是最优先检查的内容之一。导航不稳、巡航跑偏、目标点乱跳、瞄准方向奇怪，很多时候最后都会落到 TF 上。

---

## 12. 地图、路径、模型与资源文件

把地图、巡航路径、停止点和其他资源文件统一收进了 `resource/` 下。

当前主要保留了这些内容：

- `resource/maps/`
- `resource/road_paths/`
- `resource/stop_points/`

这套工程和具体场地、车体、安装位姿绑定得很深，所以我在自己调试时经常直接改这些内容：

- 地图文件
- 导航目标点
- 巡航路径文件
- 停止点坐标
- 车辆尺寸参数
- 传感器安装外参
- 视觉模型文件
- 物资点或比赛逻辑相关配置


```text
/home/duzhong/dzacs/src/resource/maps/newmap.yaml
/home/duzhong/dzacs/src/resource/maps/newmap
/home/duzhong/dzacs/src/resource/road_paths/1.txt
```

---

## 13. 实际联调时的理解方式

自己在现场调这套系统时，如果按链路来拆，可以这么看。

### 13.1 先看底层是否活着

- 雷达有没有扫描
- IMU 有没有发布
- 里程计有没有跳
- 底盘串口有没有通
- 车是否能响应 `/cmd_vel`

### 13.2 再看定位是否可信

- 地图是否加载正确
- 激光定位是否贴地图
- `odom` 到 `map` 的关系是否稳定
- 机器人在 RViz 里的姿态是否和真实车体一致

### 13.3 再看导航是否闭环

- `move_base` 是否能收到目标
- global path 是否正常生成
- local planner 是否持续输出控制量
- 底盘是否真正执行
- 轨迹是否稳定

### 13.4 再看感知与比赛逻辑是否接上

- 视觉链路是否能出目标
- 裁判系统是否持续更新
- 物资状态是否正常进入上层逻辑
- 行为树是否真的切到当前该执行的节点
- 云台和底盘是否在正确阶段响应

### 13.5 最后再看比赛节奏是否顺

- 开局阶段是否顺畅
- 物资点处理是否卡顿
- 巡航段和导航段切换是否自然
- 对抗阶段是否会出现互锁
- 射击等待条件是否过严或过松

建议第一次参赛or学习的同学养成这样看比赛机器人系统的习惯。因为单个节点跑起来只能说明“它存在”，整条链路顺起来才说明“它有用”。

---

## 14. 二次开发方向

建议各位看这套工程时，可以优先把二次开发大致分成下面几个方向想想（这也是之后的比赛有可能出现的场景）。

### 14.1 换车体

如果要把它迁到另一台车上，建议先改：

- `dzactuator` 里的底盘协议
- 串口名
- 编码器换算参数
- 速度上限
- 角速度上限
- 轮距和轴距相关参数
- TF 外参

然后再去看：

- 导航参数
- 巡航参数
- 行为树切换节奏

### 14.2 换场地

如果要换比赛场地，建议先改：

- 地图
- 初始位姿
- 导航目标点
- 巡航路径
- 停止点
- 物资点配置

千万不会一边换地图一边保留旧点位赌它能跑，因为这基本只会浪费时间。

### 14.3 换识别方案

如果要换视觉方案，建议优先改：

- `rknn_pt` 里的模型文件
- 输入预处理
- 输出结果格式
- 上层对目标偏移和检测结果的读取接口

换识别方案时，建议多在意新输出能不能和旧行为树无缝接上。

### 14.4 换比赛策略

如果要改比赛打法，建议优先改：

- `decision_behavior_tree` 的 XML
- 行为树节点对应的 action 逻辑
- 物资点切换规则
- 云台巡航规则
- 等待检测与等待射击条件

在这套工程里，比赛策略真正的中心位置就在行为树。

### 14.5 换局部规划器

如果要试新的局部规划器，建议重点检查：

- `/cmd_vel` 输出风格是否变化明显
- 中间控制量转换节点是否还能接住
- 车体实际响应是否更平稳
- 行为树动作超时是否需要重调

---

## 15. 可能的移植注意事项

我把自己最容易踩坑的地方都放在这里了（一定要注意看）。

### 15.1 环境问题

这套工程主要围绕 Noetic 整理，但不同机器上的系统库版本、OpenCV、第三方插件版本和编译器选项都可能带来差异。遇到问题时，建议先仔细检查环境一致性。

### 15.2 硬件耦合问题

这套工程和具体车体耦合很深。

建议在迁移时会优先确认：

- 底盘协议是否一致
- 云台控制接口是否一致
- 雷达安装位姿是否一致
- IMU 坐标方向是否一致
- 相机位置和朝向是否一致

### 15.3 地图与点位问题

只要场地变了，默认下面这些都要重来：

- 地图
- 初始位姿
- 目标点
- 巡航路径
- 物资点坐标
- 停止点

### 15.4 感知链路问题

只要硬件平台变了，我就默认下面这些都要重看：

- 相机分辨率
- 相机话题名
- RKNN 模型兼容性
- 推理速度
- 检测结果格式

### 15.5 行为树节奏问题

比赛工程里很多逻辑都带着时间和条件门槛。只要底层执行速度、导航输出风格或感知稳定性发生变化，默认行为树里的这些内容也要一起调：

- timeout
- delay
- tolerance
- retry 次数
- 切换条件

### 15.6 看似小、实际上很要命的问题

我踩过或者特别警惕的问题通常有这些：

- TF 方向反了
- 角速度符号反了
- 底盘控制和导航输出量纲不一致
- 目标点坐标系搞错
- 地图分辨率与现实比例没对齐
- 绝对路径换机后失效
- 串口名在不同机器上变化
- 行为树条件变量没有及时更新

这种问题往往最耗时间，因为它们看起来都不像“大错误”，但会让整套系统表现得非常诡异。

---

## 16. 致谢

在这套工作空间里，我整合了不少开源内容，也参考了很多社区项目、资料和已有工具链。

特别感谢开源作者们。很多时候，真正推动一个机器人项目往前走的，不只是论文和算法，还有这些能落地、能运行、能继续改的工程代码。

我也感谢那些把导航、雷达、规划、行为树、驱动、视觉相关功能包公开出来的作者。正是因为有这些工作的积累，我才能把更多精力放在比赛主链路的拼接、联调和整体打磨上。

---

## 17. 维护说明

这套工程是我在比赛环境里真正打磨出来的一版工作空间快照。

我把它开源出来，更多是希望把一套完整比赛链路留下来，而不是把它维护成一个长期持续更新的通用框架。所以我更愿意把它当作一个可复现、可参考、可继续改造的工程起点。

我欢迎大家：

- Fork
- 二次开发
- 自己继续迭代
- 结合自己的平台重构
- 在 Issues 里交流思路

我不希望把它包装成一个对所有人都开箱即用的仓库，因为比赛机器人本来就不是这种东西。真实、完整、能看出系统是怎么搭起来的，比“像产品”更重要。

---
