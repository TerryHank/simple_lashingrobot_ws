cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_process)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  rosgraph_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tie_robot_hw
  tie_robot_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_library(LIBMODBUS modbus REQUIRED)
if(NOT LIBMODBUS)
  message(FATAL_ERROR "libmodbus not found!")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tie_robot_process_planning
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    rosgraph_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tie_robot_hw
    tie_robot_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/modbus
)

add_library(tie_robot_process_planning
  src/planning/dynamic_bind_planning.cpp
  src/planning/dynamic_bind_geometry.cpp
  src/planning/dynamic_bind_grouping.cpp
  src/planning/dynamic_bind_builder.cpp
)
add_dependencies(tie_robot_process_planning ${catkin_EXPORTED_TARGETS})
target_link_libraries(tie_robot_process_planning ${catkin_LIBRARIES})

add_executable(suoquNode
  src/suoqu/cabin_transport.cpp
  src/suoqu/area_execution.cpp
  src/suoqu/execution_memory_store.cpp
  src/suoqu/bind_path_store.cpp
  src/suoqu/pseudo_slam_markers.cpp
  src/suoqu/pseudo_slam_scan_processing.cpp
  src/suoqu/service_orchestration.cpp
  src/suoquNode.cpp
)
add_dependencies(suoquNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(suoquNode
  ${catkin_LIBRARIES}
  ${LIBMODBUS}
  modbus
  tie_robot_hw_driver_core
  tie_robot_process_planning
  -fopenmp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS tie_robot_process_planning suoquNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
