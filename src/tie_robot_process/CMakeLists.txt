cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_process)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
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
    actionlib
    actionlib_msgs
    diagnostic_updater
    geometry_msgs
    nav_msgs
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

set(SUOQU_NODE_CORE_SOURCES
  src/suoqu/cabin_transport.cpp
  src/suoqu/area_execution.cpp
  src/suoqu/execution_memory_store.cpp
  src/suoqu/bind_path_store.cpp
  src/suoqu/pseudo_slam_markers.cpp
  src/suoqu/pseudo_slam_scan_processing.cpp
  src/suoqu/service_orchestration.cpp
  src/suoquNode.cpp
)

set(SUOQU_NODE_LIBRARIES
  ${catkin_LIBRARIES}
  ${LIBMODBUS}
  modbus
  tie_robot_hw_driver_core
  tie_robot_process_planning
  -fopenmp
)

add_executable(suoquNode
  src/suoqu_node_compat.cpp
  ${SUOQU_NODE_CORE_SOURCES}
)
add_dependencies(suoquNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(suoquNode ${SUOQU_NODE_LIBRARIES})

add_executable(suoqu_driver_node
  src/suoqu_driver_node.cpp
  ${SUOQU_NODE_CORE_SOURCES}
)
add_dependencies(suoqu_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(suoqu_driver_node ${SUOQU_NODE_LIBRARIES})

add_executable(cabin_motion_controller_node
  src/cabin_motion_controller_node.cpp
  ${SUOQU_NODE_CORE_SOURCES}
)
add_dependencies(cabin_motion_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cabin_motion_controller_node ${SUOQU_NODE_LIBRARIES})

add_executable(bind_task_executor_node
  src/bind_task_executor_node.cpp
  ${SUOQU_NODE_CORE_SOURCES}
)
add_dependencies(bind_task_executor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bind_task_executor_node ${SUOQU_NODE_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  tie_robot_process_planning
  suoquNode
  suoqu_driver_node
  cabin_motion_controller_node
  bind_task_executor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/architecture_role_node.py
  scripts/global_bind_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dynamic_bind_planning
    test/test_dynamic_bind_planning.cpp
  )
  if(TARGET test_dynamic_bind_planning)
    target_include_directories(test_dynamic_bind_planning PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/planning
    )
    target_link_libraries(test_dynamic_bind_planning
      tie_robot_process_planning
      ${catkin_LIBRARIES}
      gtest_main
    )
  endif()
endif()
