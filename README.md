# ros1_double_arm_robot

This repository is the extracted ROS1 catkin workspace for the dual-arm picking robot. It is intended to live at:

```bash
/home/zmy/test/ros1_double_arm_robot
```

The current runtime chain is:

```text
MoveIt demo.launch
  -> /Double_arm_robot/l_arm/follow_joint_trajectory
  -> /Double_arm_robot/r_arm/follow_joint_trajectory
  -> jaka_controller_L / jaka_controller_R
  -> command_L / command_R
  -> jaka_send_read_node_zong_L / jaka_send_read_node_zong_R
  -> motor buses + /joint_states
```

## Packages

- `Double_arm_robot`: URDF, meshes and robot description.
- `Double_arm_robot_moveit`: MoveIt configuration, SRDF, controllers and RViz config.
- `jaka_controller_tcp`: trajectory action servers and hardware communication nodes.
- `moveit_exe`: MoveIt task scripts and custom messages/actions for later vision integration.

## Build

```bash
cd /home/zmy/test/ros1_double_arm_robot
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
source devel/setup.bash
```

The explicit Python and empy paths avoid conda overriding ROS Noetic's Python 3.8 build tools.

## Start Main Flow

One-command startup:

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch jaka_controller_tcp double_arm_core.launch
```

Manual startup, equivalent to the current terminal order:

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch Double_arm_robot_moveit demo.launch
rosrun jaka_controller_tcp jaka_controller_L
rosrun jaka_controller_tcp jaka_controller_R
rosrun jaka_controller_tcp jaka_send_read_node_zong_L
rosrun jaka_controller_tcp jaka_send_read_node_zong_R
```

Run the current dual-arm manual task script after the core system is up:

```bash
cd /home/zmy/test/ros1_double_arm_robot
source devel/setup.bash
roslaunch moveit_exe manual_demo.launch
```

The helper script below opens the current hardware bridge and MoveIt flow in separate terminals after the workspace is built:

```bash
cd /home/zmy/test/ros1_double_arm_robot
./start_double_arm_windows.sh
```

## Hardware Ports In Current Code

- Left arm Modbus motors: `/dev/ttyUSB1`, 57600, stopbits 2.
- Left arm serial motors: `/dev/ttyUSB0`, 115200.
- Right arm Modbus motors: `/dev/ttyUSB3`, 57600, stopbits 2.
- Right arm serial motors: `/dev/ttyUSB2`, 115200.
- End-effector/gripper in `moveit_exe/scripts/xxxcopy.py`: `/dev/ttyUSB4`, 9600.

These are still hard-coded in the extracted code. A good next refactor is moving them into launch parameters or YAML.

## Notes

- `jaka_controller_L/R` expose the FollowJointTrajectory action names expected by `simple_moveit_controllers.yaml`.
- `jaka_send_read_node_zong_L/R` publish `/joint_states`, which is consumed by `joint_state_publisher` and MoveIt.
- Vision recognition can later feed the target points into `moveit_exe` instead of the current hard-coded demo points.
