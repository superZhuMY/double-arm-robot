cmake_minimum_required(VERSION 3.0.2)
project(moveit_exe)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  io.msg
  zitai.msg
  zitai1.msg
)

add_action_files(
  FILES
  object_pose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime rospy std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/xxxcopy.py
    scripts/xxcopy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

