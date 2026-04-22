cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_hw)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_library(LIBMODBUS modbus REQUIRED)
if(NOT LIBMODBUS)
  message(FATAL_ERROR "libmodbus not found!")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tie_robot_hw_driver_core
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/modbus
)

add_library(tie_robot_hw_driver_core
  src/driver/cabin_tcp_transport.cpp
  src/driver/cabin_protocol.cpp
  src/driver/cabin_driver.cpp
  src/driver/modbus_transport.cpp
  src/driver/linear_module_protocol.cpp
  src/driver/linear_module_driver.cpp
)

add_dependencies(tie_robot_hw_driver_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tie_robot_hw_driver_core
  ${catkin_LIBRARIES}
  ${LIBMODBUS}
  modbus
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS tie_robot_hw_driver_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
