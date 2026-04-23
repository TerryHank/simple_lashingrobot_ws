cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_perception)

add_definitions(-std=c++14)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tie_robot_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree)
find_library(
  SCEPTER_API_LIB
  NAMES libScepter_api.so
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Lib
  NO_DEFAULT_PATH
)

if(NOT SCEPTER_API_LIB)
  message(FATAL_ERROR "libScepter_api.so not found under dependencies/Lib")
endif()

add_service_files(
  FILES
  ConvertDepthToPointCloud.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Sceptertof_roscpp.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    camera_info_manager
    cv_bridge
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    image_transport
    message_runtime
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tie_robot_msgs
  DEPENDS OpenCV PCL
)

include_directories(
  include
  dependencies/Include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(scepter_camera
  src/camera/scepter_driver.cpp
  src/camera/scepter_manager.cpp
  src/camera/device_session.cpp
  src/camera/frame_publish.cpp
  src/camera/intrinsics.cpp
)

add_dependencies(scepter_camera ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scepter_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SCEPTER_API_LIB}
)

add_executable(scepter_world_coord_processor
  src/perception/scepter_world_coord_processor.cpp
  src/perception/scepter_world_coord_processor_node.cpp
)

add_dependencies(scepter_world_coord_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scepter_world_coord_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/pointAI.py
  scripts/gripper_tf_broadcaster.py
  scripts/stable_point_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch cfg dependencies config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS scepter_camera scepter_world_coord_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
