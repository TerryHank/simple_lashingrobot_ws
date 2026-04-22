cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

add_service_files(
  FILES
  SingleMove.srv
  Pathguihua.srv
  linear_module_move.srv
  ProcessImage.srv
  PlaneDetection.srv
  ExecuteBindPoints.srv
  ResetPLCWarning.srv
  Resetzero.srv
  MotionControl.srv
  StartGlobalWork.srv
  StartPseudoSlamScan.srv
  SetExecutionMode.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Move.action
  StartPseudoSlamScanTask.action
  StartGlobalWorkTask.action
  RunBindPathDirectTestTask.action
)

add_message_files(
  FILES
  motion.msg
  linear_module_move_single.msg
  linear_module_move_all.msg
  PointCoords.msg
  PointsArray.msg
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
  path_planning_result.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    rosgraph_msgs
    sensor_msgs
    std_msgs
    std_srvs
)
