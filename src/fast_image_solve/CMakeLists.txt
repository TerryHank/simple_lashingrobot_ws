cmake_minimum_required(VERSION 3.0.2)
project(fast_image_solve)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  image_transport
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)

add_service_files(
  FILES
  image.srv
  ProcessImage.srv
  PlaneDetection.srv
  DetectAndSavePose.srv
)

add_message_files(
  FILES
  PointsArray.msg
  PointCoords.msg
  PointWithDepth.msg
  PointsWithDepthArray.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge tf2_ros image_transport message_runtime
)



include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



add_executable(fast_image_node src/fast_image_node.cpp src/fast_image.cpp include/fast_image_solve/fast_image.hpp)
target_link_libraries(fast_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

add_executable(client_node src/client.cpp src/fast_image.cpp include/fast_image_solve/fast_image.hpp)
target_link_libraries(client_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

add_executable(5.18auto_node src/5.18auto_node.cpp src/5.18auto.cpp include/fast_image_solve/5.18auto.hpp)
target_link_libraries(5.18auto_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

add_executable(sensors_mid_node src/sensors_mid_node.cpp)
target_link_libraries(sensors_mid_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

# install(TARGETS ${PROJECT_NAME} fast_image_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
