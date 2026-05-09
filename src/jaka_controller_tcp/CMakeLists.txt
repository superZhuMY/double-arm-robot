cmake_minimum_required(VERSION 3.0.2)
project(jaka_controller_tcp)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  message_generation
  moveit_exe
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  trajectory_msgs
)

add_message_files(
  FILES
  Command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jaka_controller_tcp modbus_485_pick
  CATKIN_DEPENDS
    actionlib
    control_msgs
    geometry_msgs
    message_runtime
    moveit_exe
    roscpp
    rospy
    sensor_msgs
    serial
    std_msgs
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/modbus
)

find_library(MODBUS_LIBRARY
  NAMES modbus
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

if(NOT MODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus was not found. Install libmodbus-dev or check /usr/local/lib/libmodbus.so.")
endif()

add_library(${PROJECT_NAME}
  src/cubicSpline.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(modbus_485_pick
  src/modbus_485_pick.cpp
)
add_dependencies(modbus_485_pick ${catkin_EXPORTED_TARGETS})
target_link_libraries(modbus_485_pick ${MODBUS_LIBRARY})

function(add_jaka_node target source)
  add_executable(${target} ${source})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    ${MODBUS_LIBRARY}
    modbus_485_pick
    ${PROJECT_NAME}
  )
endfunction()

add_jaka_node(jaka_controller_L src/jaka_controller_L.cpp)
add_jaka_node(jaka_controller_R src/jaka_controller_R.cpp)
add_jaka_node(jaka_send_read_node_zong_L src/jaka_send_read_node_zong_L.cpp)
add_jaka_node(jaka_send_read_node_zong_R src/jaka_send_read_node_zong_R.cpp)

