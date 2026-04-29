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
  INCLUDE_DIRS include
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

set(WEB_ACTION_BRIDGE_NODE_SOURCES
  src/web_action_bridge.cpp
  src/web_bridge/runtime.cpp
  src/web_bridge/action_bridge.cpp
  src/web_bridge/process_control.cpp
  src/web_bridge/node_app.cpp
)

foreach(WEB_ACTION_BRIDGE_TARGET
    web_action_bridge_node
    webActionBridgeNode
)
  add_executable(${WEB_ACTION_BRIDGE_TARGET}
    ${WEB_ACTION_BRIDGE_NODE_SOURCES}
  )
  add_dependencies(${WEB_ACTION_BRIDGE_TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${WEB_ACTION_BRIDGE_TARGET} ${catkin_LIBRARIES})
endforeach()

catkin_install_python(PROGRAMS
  scripts/system_log_mux.py
  scripts/workspace_picker_web_server.py
  scripts/workspace_picker_web_open.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  web_action_bridge_node
  webActionBridgeNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
