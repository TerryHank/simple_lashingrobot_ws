cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_web)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  rosgraph_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tie_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    roscpp
    rosgraph_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tie_robot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(topictransNode
  src/topics_transfer.cpp
  src/web_bridge/runtime.cpp
  src/web_bridge/action_bridge.cpp
  src/web_bridge/process_control.cpp
  src/web_bridge/subscriber_callbacks.cpp
  src/web_bridge/node_app.cpp
)
add_dependencies(topictransNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topictransNode ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/system_log_mux.py
  scripts/workspace_picker_web_server.py
  scripts/workspace_picker_web_open.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS topictransNode
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
