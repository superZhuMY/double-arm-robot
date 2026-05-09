# Startup Flow

## Workspace Root

```bash
/home/zmy/test/ros1_double_arm_robot
```

Build and source before starting the runtime chain:

```bash
cd /home/zmy/test/ros1_double_arm_robot
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
source devel/setup.bash
```

## Current Core Order

1. Start MoveIt and robot state:

   ```bash
   roslaunch Double_arm_robot_moveit demo.launch
   ```

2. Start left and right trajectory action servers:

   ```bash
   rosrun jaka_controller_tcp jaka_controller_L
   rosrun jaka_controller_tcp jaka_controller_R
   ```

3. Start left and right hardware communication bridges:

   ```bash
   rosrun jaka_controller_tcp jaka_send_read_node_zong_L
   rosrun jaka_controller_tcp jaka_send_read_node_zong_R
   ```

4. Start the current manual picking/motion script:

   ```bash
   roslaunch moveit_exe manual_demo.launch
   ```

## Topic And Action Links

```text
MoveIt simple controller manager
  /Double_arm_robot/l_arm/follow_joint_trajectory
  /Double_arm_robot/r_arm/follow_joint_trajectory

jaka_controller_L
  subscribes action goal from MoveIt
  publishes command_L

jaka_controller_R
  subscribes action goal from MoveIt
  publishes command_R

jaka_send_read_node_zong_L
  subscribes command_L
  writes left motor buses
  publishes /joint_states with L_Joint_1 ... L_Joint_6

jaka_send_read_node_zong_R
  subscribes command_R
  writes right motor buses
  publishes /joint_states with R_Joint_1 ... R_Joint_6
```

## Refactor Targets

1. Move serial and Modbus port names to launch parameters.
2. Merge duplicate left/right controller code after confirming unit conversion.
3. Merge or coordinate `/joint_states` publishing so both arms are published in one complete message.
4. Replace hard-coded target points in `xxxcopy.py` with a vision-result topic/action.
