cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_control)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    diagnostic_updater
    actionlib
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

set(MODUAN_NODE_CORE_SOURCES
  src/moduan/runtime_state.cpp
  src/moduan/numeric_codec.cpp
  src/moduan/error_handling.cpp
  src/moduan/linear_module_executor.cpp
  src/moduan/moduan_ros_callbacks.cpp
)

set(MODUAN_NODE_LIBRARIES
  ${catkin_LIBRARIES}
  ${LIBMODBUS}
  modbus
  tie_robot_hw_driver_core
)

add_executable(moduanNode
  src/moduanNode.cpp
  ${MODUAN_NODE_CORE_SOURCES}
)
add_dependencies(moduanNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(moduanNode ${MODUAN_NODE_LIBRARIES})

add_executable(moduan_driver_node
  src/moduan_driver_node.cpp
  ${MODUAN_NODE_CORE_SOURCES}
)
add_dependencies(moduan_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(moduan_driver_node ${MODUAN_NODE_LIBRARIES})

add_executable(moduan_motion_controller_node
  src/moduan_motion_controller_node.cpp
  ${MODUAN_NODE_CORE_SOURCES}
)
add_dependencies(moduan_motion_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(moduan_motion_controller_node ${MODUAN_NODE_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  moduanNode
  moduan_driver_node
  moduan_motion_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
