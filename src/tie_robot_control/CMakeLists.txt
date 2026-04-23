cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_control)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tie_robot_hw
  tie_robot_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_library(LIBMODBUS modbus REQUIRED)
if(NOT LIBMODBUS)
  message(FATAL_ERROR "libmodbus not found!")
endif()

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tie_robot_hw
    tie_robot_msgs
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/modbus
)

add_executable(moduanNode
  src/moduanNode.cpp
  src/moduan/runtime_state.cpp
  src/moduan/numeric_codec.cpp
  src/moduan/error_handling.cpp
  src/moduan/linear_module_executor.cpp
  src/moduan/moduan_ros_callbacks.cpp
)
add_dependencies(moduanNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(moduanNode
  ${catkin_LIBRARIES}
  ${LIBMODBUS}
  modbus
  tie_robot_hw_driver_core
)
