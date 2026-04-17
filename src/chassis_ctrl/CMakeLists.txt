cmake_minimum_required(VERSION 3.0.2)
project(chassis_ctrl)

find_package(catkin REQUIRED COMPONENTS
  fast_image_solve
  geometry_msgs
  rosgraph_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  message_generation
  actionlib_msgs
)

# 查找自定义库
find_library(LIBSA simulated_annealing PATHS /home/hyq/hyq-ros-wrapper/simple_lashingrobot_ws/devel/lib/libsimulated_annealing.so)
# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 查找 libmodbus 库
find_library(LIBMODBUS modbus REQUIRED)
if(NOT LIBMODBUS)
    message(FATAL_ERROR "libmodbus not found!")
endif()

add_service_files(
  FILES
  SingleMove.srv
  Pathguihua.srv
  linear_module_move.srv
  ExecuteBindPoints.srv
  ResetPLCWarning.srv
  Resetzero.srv
  MotionControl.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Move.action
)
# 生成消息
add_message_files(
  FILES
  motion.msg
  linear_module_move_single.msg
  linear_module_move_all.msg
  cabin_move_single.msg
  cabin_move_all.msg
  cabin_calibration.msg
  area_choose.msg
  linear_module_upload.msg
  cabin_upload.msg
  LashingArray.msg
  LashingCoords.msg
  Area_info.msg
  AreaProgress.msg
)

# 生成添加的消息和服务及其依赖项
generate_messages(
  DEPENDENCIES
  fast_image_solve
  geometry_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS fast_image_solve geometry_msgs rosgraph_msgs roscpp rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros message_runtime actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/modbus  # modbus 头文件路径
)

# 添加可执行文件
add_executable(moduanNode src/moduanNode.cpp)
add_executable(moduanNode_show src/moduanNode_show.cpp)
add_executable(client src/client.cpp)
add_executable(suoquNode src/suoquNode.cpp)
add_executable(topictransNode src/topics_transfer.cpp)

add_dependencies(moduanNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(moduanNode_show ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(suoquNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(topictransNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库（简化重复的链接设置）
set(COMMON_LIBS
  ${catkin_LIBRARIES}
  ${LIBSA}
  ${LIBMODBUS}
  modbus
)

target_link_libraries(moduanNode ${COMMON_LIBS})
target_link_libraries(moduanNode_show ${COMMON_LIBS})
target_link_libraries(client ${COMMON_LIBS})
target_link_libraries(suoquNode ${COMMON_LIBS} -fopenmp)
target_link_libraries(topictransNode ${COMMON_LIBS})
