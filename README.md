# ros1_double_arm_robot

## 项目简介

这是双臂采摘机器人重构后的 ROS1 catkin 工作空间，已经从旧工程中单独拆出，当前建议放在：

```bash
/home/zmy/test/ros1_double_arm_robot
```

当前主链路如下：

```text
MoveIt demo.launch
  -> /Double_arm_robot/l_arm/follow_joint_trajectory
  -> /Double_arm_robot/r_arm/follow_joint_trajectory
  -> jaka_controller_L / jaka_controller_R
  -> command_L / command_R
  -> jaka_send_read_node_zong_L / jaka_send_read_node_zong_R
  -> 电机总线 + /joint_states
```

本仓库只保留当前重构后的核心代码，不包含旧工程里的冗余包和历史临时文件。

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- MoveIt
- `libmodbus`
- Python 3.8

如果本机存在 conda 环境，构建时建议显式指定 ROS Noetic 使用的 Python 和 empy 路径，避免被 conda 覆盖。

## 工作区构建

```bash
cd /home/zmy/test/ros1_double_arm_robot
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
source devel/setup.bash
```

构建后可以检查 4 个核心包是否能被 ROS 找到：

```bash
rospack find Double_arm_robot
rospack find Double_arm_robot_moveit
rospack find jaka_controller_tcp
rospack find moveit_exe
```

## 启动顺序

### 一键启动主流程

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch jaka_controller_tcp double_arm_core.launch
```

### 手动分终端启动

第一个终端启动 MoveIt 和机器人状态：

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch Double_arm_robot_moveit demo.launch
```

第二、三个终端启动左右臂轨迹 action server：

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
rosrun jaka_controller_tcp jaka_controller_L
```

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
rosrun jaka_controller_tcp jaka_controller_R
```

第四、五个终端启动左右臂硬件通信节点：

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
rosrun jaka_controller_tcp jaka_send_read_node_zong_L
```

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
rosrun jaka_controller_tcp jaka_send_read_node_zong_R
```

主系统启动后，再运行当前的双臂手动任务脚本：

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch moveit_exe manual_demo.launch
```

### 辅助启动脚本

仓库根目录下的脚本会在工作空间构建完成后，尝试打开硬件通信和 MoveIt 的终端窗口：

```bash
cd /home/zmy/test/ros1_double_arm_robot
./start_double_arm_windows.sh
```

## ROS 包说明

- `Double_arm_robot`：机器人 URDF、mesh 和基础描述文件。
- `Double_arm_robot_moveit`：MoveIt 配置、SRDF、控制器配置和 RViz 配置。
- `jaka_controller_tcp`：左右臂轨迹 action server、485/串口硬件通信节点和自定义消息。
- `moveit_exe`：MoveIt 任务脚本、自定义消息和 action，后续可接入视觉识别结果。

## 主要 Topic / Action

MoveIt simple controller manager 使用的 FollowJointTrajectory action：

```text
/Double_arm_robot/l_arm/follow_joint_trajectory
/Double_arm_robot/r_arm/follow_joint_trajectory
```

控制链路中的命令 topic：

```text
command_L
command_R
```

硬件通信节点会发布关节状态：

```text
/joint_states
```

## 硬件与外部依赖

当前硬件端口仍然写在代码中，调试真机前需要确认本机 `/dev/ttyUSB*` 映射是否一致。

- 左臂 Modbus 电机：`/dev/ttyUSB1`，57600，stopbits 2。
- 左臂串口电机：`/dev/ttyUSB0`，115200。
- 右臂 Modbus 电机：`/dev/ttyUSB3`，57600，stopbits 2。
- 右臂串口电机：`/dev/ttyUSB2`，115200。
- 末端执行器/夹爪：`moveit_exe/scripts/xxxcopy.py` 中的 `/dev/ttyUSB4`，9600。

如果启动硬件节点时报串口或 Modbus 连接失败，优先检查：

```bash
ls -l /dev/ttyUSB*
groups
dmesg | grep ttyUSB
```

普通用户通常需要加入 `dialout` 组后重新登录，才能稳定访问串口设备。

## 常见问题

### MoveIt 插件加载失败

如果 RViz 或 MoveIt 报缺少 `moveit_rviz_plugin`、`ompl`、`moveit_fake_controller_manager`、`moveit_simple_controller_manager`，通常是系统 ROS 包未安装完整。需要在本机确认对应的 `ros-noetic-*` 包是否已经安装。

### 构建时 Python 或 empy 报错

优先使用本文档中的构建命令，显式指定：

```bash
-DPYTHON_EXECUTABLE=/usr/bin/python3
-DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
```

这可以避免 conda 环境把 ROS Noetic 的 Python 工具链覆盖掉。

### 真机调试前的建议

先单独启动硬件通信节点，确认每个电机有反馈值，再用 MoveIt 做小幅度 Joint Goal 测试。不要一开始就执行大范围 Pose Goal。

## 后续开发建议

1. 将左右臂的 Modbus 和串口端口从源码中移到 launch 参数或 YAML。
2. 合并重复的左右臂控制代码，但要先确认角度单位、方向和零点转换。
3. 整理 `/joint_states` 发布逻辑，避免左右臂状态互相覆盖。
4. 将 `moveit_exe` 中的硬编码目标点替换为视觉识别 topic 或 action 输入。
