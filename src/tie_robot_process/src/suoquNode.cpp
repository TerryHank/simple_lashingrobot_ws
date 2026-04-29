#include <ctime>  
#include <cmath> 
#include <thread>
#include <vector>
#include <fcntl.h>
#include <iomanip> // 用于std::put_time 
#include <stdio.h>
#include <csignal> 
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <tie_robot_process/json.hpp>
#include <iostream>  
#include <memory>
#include <cctype>
#include <cstdio>
#include <tuple>
#include <limits>
#include <mutex>
#include <atomic>
#include <sstream>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <pthread.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <algorithm>
#include <numeric>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <omp.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>
#include <tie_robot_msgs/motion.h>
#include "tie_robot_msgs/cabin_upload.h" //用于上传索驱模块反馈
#include "tie_robot_msgs/MotionControl.h"
#include <std_msgs/Float32MultiArray.h>
#include "tie_robot_msgs/cabin_move_all.h"
#include "tie_robot_msgs/cabin_move_single.h"
#include "tie_robot_msgs/cabin_calibration.h"
#include "tie_robot_msgs/linear_module_move_all.h"
#include "tie_robot_msgs/linear_module_upload.h"
#include "tie_robot_msgs/AreaProgress.h"
#include <tie_robot_msgs/ProcessImage.h>
#include "tie_robot_msgs/PointCoords.h"
#include <tie_robot_msgs/ProcessImage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "tie_robot_msgs/area_choose.h"
#include "tie_robot_msgs/ExecuteBindPoints.h"
#include "std_srvs/Trigger.h" 
#include <tie_robot_msgs/Pathguihua.h>
#include <tie_robot_msgs/SingleMove.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tie_robot_process/planning/dynamic_bind_planning.hpp"
#include "tie_robot_process/suoqu/cabin_transport.hpp"
#include "tie_robot_process/suoqu/suoqu_node_app.hpp"
#include "tie_robot_hw/driver/cabin_driver.hpp"
#include <tie_robot_process/common.hpp>
#include "suoqu/suoqu_runtime_internal.hpp"

using namespace std;
using namespace tie_robot_process::suoqu;

std::unique_ptr<tie_robot_hw::driver::CabinDriver> g_cabin_driver;
extern int sockfd;

std::string path_points_json_file = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/path_points.json";
std::string pseudo_slam_points_json_file = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_points.json";
std::string pseudo_slam_bind_path_json_file = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_bind_path.json";
const std::string kBindExecutionMemoryJsonPath =
    "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/bind_execution_memory.json";
const std::string kCabinLastFatalErrorDetailFile =
    "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/cabin_last_fatal_error.txt";

#define AXIS_X 0 //索驱轴体代表宏定义
#define AXIS_Y 3
#define AXIS_Z 4

// 定义互斥锁来保护 cabin_state 的读写操作
std::mutex cabin_state_mutex;
// 定义互斥锁避免同一时刻对TCP写入产生粘包现象
std::mutex socket_mutex;
// 串行化 pseudo-slam 扫描/执行主链，避免并发读写同一批扫描产物与执行账本。
std::mutex pseudo_slam_workflow_mutex;
std::mutex pseudo_slam_tf_points_mutex;
std::mutex pseudo_slam_ir_image_mutex;
std::mutex pseudo_slam_marker_state_mutex;
std::mutex pseudo_slam_marker_path_origin_mutex;
std::mutex cabin_last_error_detail_mutex;
std::string last_cabin_transport_error_detail;
std::string last_cabin_execution_wait_error_detail;

[[noreturn]] void emergency_exit_with_flush(int exit_code)
{
    fflush(stdout);
    fflush(stderr);
    _exit(exit_code);
}

void clear_pseudo_slam_marker_execution_state();

//右上角标定区域坐标
float marking_x;
float marking_y;
// //作业区域x轴和y轴的长度,单位毫米
// float zone_x=2800;
// float zone_y=2500;
// //机器人在x轴和y轴移动的步长
// float robot_x_step=370;
// float robot_y_step=320;
//作业区域x轴和y轴的长度,单位毫米
float zone_x;
float zone_y;
//机器人在x轴和y轴移动的步长
float robot_x_step;
float robot_y_step;
int   point_count;
float height_sum ;
float height_avg ;

float cabin_start_x;
float cabin_start_y;
float cabin_start_height;
float global_cabin_speed = 300.0;

float get_global_cabin_move_speed_mm_per_sec()
{
    return global_cabin_speed > 0.0f ? global_cabin_speed : 300.0f;
}

// 发送接收，字符左高右低
// uint8_t robot_inquire[6]={0xEB,0x90,0x00,0x01,0x7C,0x01};//查询索驱状态
uint8_t TCP_Normal_Connection[14]={0xEB,0x90,0x00,0x01};//和索驱常态通讯的数据帧，包括位置查询和将机身的xy姿态角信息
uint8_t motor_enable[8]={0xEB,0x90,0x00,0x02,0xFF,0x00,0x7C,0x02};//8个电机使能
uint8_t motor_disable[8]={0xEB,0x90,0x00,0x02,0x00,0xFF,0x7C,0x02};//8个电机失能
uint8_t motor_inverse_enable[8]={0xEB,0x90,0x00,0x06,0x05,0x00,0x86,0x01};//电机逆解激活
uint8_t motor_inverse_disable[8]={0xEB,0x90,0x00,0x06,0x0A,0x00,0x8B,0x01};//电机逆解禁用
uint8_t TCP_stop[8]={0xEB,0x90,0x00,0x13,0x01,0x00,0x8F,0x01};//运动停止
uint8_t TCP_Move_Frame[36]={0};
uint8_t FtoU_register[4]={0};//用于暂时储存float数转为成的uint数
// 数组各个值依次为速度（mm/s），xyz轴运动到的位置（mm），三个转轴的位置（角度）
float TCP_Move[7]={200,0,0,400,0,0,0};//速度,x,y,z,a,b,c
constexpr int TCP_TIMEOUT_SEC = 5;
constexpr int RECV_BUFFER_SIZE = 256;
constexpr int WRITE_DELAY_MS = 300;
constexpr int HEARTBEAT_WRITE_DELAY_MS = 0;
constexpr int kMotionWaitTimeoutSec = 30;
constexpr int kMotionWaitLogIntervalSec = 2;

tie_robot_process::planning::DynamicBindPlannerConfig build_dynamic_bind_planner_config()
{
    tie_robot_process::planning::DynamicBindPlannerConfig config;
    config.tcp_max_x_mm = kTravelMaxXMm;
    config.tcp_max_y_mm = kTravelMaxYMm;
    config.tcp_max_z_mm = kTravelMaxZMm;
    config.bind_execution_cabin_min_z_mm = kBindExecutionCabinMinZMm;
    config.template_center_x_mm = kDynamicBindTemplateCenterXMm;
    config.template_center_y_mm = kDynamicBindTemplateCenterYMm;
    config.template_center_z_mm = kDynamicBindTemplateCenterZMm;
    config.matrix_row_threshold_mm = 40.0f;
    config.matrix_column_threshold_mm = 45.0f;
    config.snake_row_tolerance_mm = kDynamicBindSnakeRowToleranceMm;
    config.seed_neighbor_count = kDynamicBindSeedNeighborCount;
    return config;
}

std::atomic<int> global_execution_mode{static_cast<int>(GlobalExecutionMode::kLiveVisual)};
std::atomic<bool> cabin_driver_enabled{true};
std::atomic<double> cabin_driver_last_state_stamp_sec{0.0};
std::atomic<uint16_t> pending_tcp_status_word{0};
std::atomic<bool> pending_tcp_status_word_valid{false};
std::atomic<bool> use_remote_cabin_driver{false};
std::string suoqu_node_role = "compat_all";
std::unique_ptr<diagnostic_updater::Updater> g_cabin_diagnostic_updater;

namespace {

constexpr const char* kCabinDiagnosticHardwareId = "tie_robot/chassis_driver";

bool is_suoqu_driver_role()
{
    return suoqu_node_role == "driver" || suoqu_node_role == "compat_all";
}

bool is_suoqu_cabin_motion_controller_role()
{
    return suoqu_node_role == "cabin_motion_controller" || suoqu_node_role == "compat_all";
}

bool is_suoqu_bind_task_executor_role()
{
    return suoqu_node_role == "bind_task_executor" || suoqu_node_role == "compat_all";
}

const char* cabin_connection_state_label(tie_robot_hw::driver::ConnectionState state)
{
    switch (state) {
        case tie_robot_hw::driver::ConnectionState::kConnecting:
            return "connecting";
        case tie_robot_hw::driver::ConnectionState::kReady:
            return "ready";
        case tie_robot_hw::driver::ConnectionState::kReconnecting:
            return "reconnecting";
        case tie_robot_hw::driver::ConnectionState::kFault:
            return "fault";
        case tie_robot_hw::driver::ConnectionState::kDisconnected:
        default:
            return "disconnected";
    }
}

void produce_cabin_driver_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    const bool enabled = cabin_driver_enabled.load();
    const auto transport_state = g_cabin_driver
        ? g_cabin_driver->connectionState()
        : tie_robot_hw::driver::ConnectionState::kDisconnected;
    const std::string transport_error = g_cabin_driver ? g_cabin_driver->lastErrorText() : std::string();
    const std::string failure_detail = tie_robot_process::suoqu::get_last_cabin_failure_detail();
    const double last_state_stamp = cabin_driver_last_state_stamp_sec.load();
    const double now_sec = ros::Time::now().toSec();
    const double state_age_sec =
        (last_state_stamp > 0.0 && now_sec >= last_state_stamp) ? (now_sec - last_state_stamp) : -1.0;

    if (!enabled) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "索驱驱动已关闭");
    } else if (transport_state == tie_robot_hw::driver::ConnectionState::kReady && state_age_sec >= 0.0 && state_age_sec <= 1.5) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "索驱驱动已连接");
    } else if (!failure_detail.empty() || !transport_error.empty()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "索驱驱动通信异常");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "索驱驱动未连接");
    }

    stat.hardware_id = kCabinDiagnosticHardwareId;
    stat.add("enabled", enabled ? "true" : "false");
    stat.add("transport_state", cabin_connection_state_label(transport_state));
    stat.add("socket_fd", sockfd);
    stat.add("state_age_sec", state_age_sec);
    stat.add("transport_error", transport_error);
    stat.add("failure_detail", failure_detail);
    stat.add("x_mm", cabin_state.X);
    stat.add("y_mm", cabin_state.Y);
    stat.add("z_mm", cabin_state.Z);
    stat.add("motion_status", cabin_state.motion_status);
    stat.add("device_alarm", cabin_state.device_alarm);
    stat.add("internal_calc_error", cabin_state.internal_calc_error);
}

void cabin_diagnostic_timer_callback(const ros::TimerEvent&)
{
    if (g_cabin_diagnostic_updater) {
        g_cabin_diagnostic_updater->force_update();
    }
}

}  // namespace

const char* global_execution_mode_name(GlobalExecutionMode mode)
{
    switch (mode) {
        case GlobalExecutionMode::kLiveVisual:
            return "live_visual";
        case GlobalExecutionMode::kSlamPrecomputed:
        default:
            return "slam_precomputed";
    }
}

PseudoSlamScanStrategy normalize_pseudo_slam_scan_strategy(uint8_t raw_strategy)
{
    switch (raw_strategy) {
        case 2:
            return PseudoSlamScanStrategy::kFixedManualWorkspace;
        case 1:
            return PseudoSlamScanStrategy::kMultiPose;
        case 0:
        default:
            return PseudoSlamScanStrategy::kSingleCenter;
    }
}

// 索驱 TCP 协议解析、错误诊断和驱动下发已拆到 tie_robot_process/suoqu/cabin_transport.*。

struct PseudoSlamAreaEntry
{
    int area_index;
    Cabin_Point cabin_point;
    std::vector<tie_robot_msgs::PointCoords> bind_points_world;
};

using PseudoSlamBindGroup = tie_robot_process::planning::PseudoSlamBindGroup;
using PseudoSlamGroupedAreaEntry = tie_robot_process::planning::PseudoSlamGroupedAreaEntry;
using BindExecutionPathOriginPose = tie_robot_process::planning::BindExecutionPathOriginPose;

struct DynamicBindPlanningCandidatePose
{
    float cabin_x = 0.0f;
    float cabin_y = 0.0f;
    float cabin_z = 0.0f;
};

const char* kBindExecutionMemoryUnreadableError =
    "bind_execution_memory.json已存在但不可用（读取失败或内容损坏），已阻止执行以避免重复绑扎";

struct PseudoSlamCaptureGateConfig
{
    int scan_min_point_count = kPseudoSlamScanMinPointCount;
    double scan_retry_interval_sec = kPseudoSlamScanRetryIntervalSec;
    int stable_sample_count = kPseudoSlamCaptureGateStableSampleCount;
    double poll_interval_sec = kPseudoSlamCaptureGatePollIntervalSec;
    double image_mean_diff_threshold = kPseudoSlamCaptureGateImageMeanDiffThreshold;
    double log_interval_sec = kPseudoSlamCaptureGateLogIntervalSec;
    float target_tolerance_mm = kPseudoSlamCaptureGateTargetToleranceMm;
    float pose_delta_tolerance_mm = kPseudoSlamCaptureGatePoseDeltaToleranceMm;
    int roi_min_x = kPseudoSlamCaptureGateRoiMinX;
    int roi_max_x = kPseudoSlamCaptureGateRoiMaxX;
    int roi_min_y = kPseudoSlamCaptureGateRoiMinY;
    int roi_max_y = kPseudoSlamCaptureGateRoiMaxY;
};

std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_columns(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_planning_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
);
std::vector<tie_robot_msgs::PointCoords> collect_pseudo_slam_planning_z_outliers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_secondary_plane_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_points_near_outlier_secondary_plane_members(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& secondary_plane_outlier_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_line_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::unordered_set<int> collect_pseudo_slam_outlier_column_neighbor_blocked_global_indices(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::vector<tie_robot_msgs::PointCoords>& outlier_points
);
std::unordered_map<int, PseudoSlamCheckerboardInfo> build_checkerboard_info_by_global_index(
    const std::vector<tie_robot_msgs::PointCoords>& world_points,
    const Cabin_Point& path_origin
);
std::unordered_map<int, PseudoSlamCheckerboardInfo> sync_merged_checkerboard_membership_with_planning(
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& merged_checkerboard_info_by_idx,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& planning_checkerboard_info_by_idx
);
std::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_non_checkerboard_points(
    const std::vector<tie_robot_msgs::PointCoords>& planning_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx
);

std::vector<tie_robot_msgs::PointCoords> pseudo_slam_marker_points;
std::unordered_set<int> pseudo_slam_marker_outlier_global_indices;
std::unordered_set<int> pseudo_slam_marker_outlier_secondary_plane_global_indices;
std::unordered_set<int> pseudo_slam_marker_outlier_line_global_indices;
std::unordered_set<int> pseudo_slam_marker_outlier_column_neighbor_global_indices;
PseudoSlamMarkerExecutionState pseudo_slam_marker_execution_state;
Cabin_Point pseudo_slam_marker_path_origin{};
bool pseudo_slam_marker_path_origin_valid = false;
std::atomic<float> pseudo_slam_marker_last_outlier_threshold_mm{kPseudoSlamPlanningZOutlierMm};
std::atomic<float> pseudo_slam_marker_last_outlier_secondary_plane_threshold_mm{kPseudoSlamPlanningZOutlierMm};
std::atomic<float> pseudo_slam_marker_last_outlier_secondary_plane_neighbor_tolerance_mm{
    kPseudoSlamOutlierSecondaryPlaneNeighborToleranceMm
};

struct ScopedPseudoSlamMarkerExecutionStateClear
{
    ~ScopedPseudoSlamMarkerExecutionStateClear()
    {
        clear_pseudo_slam_marker_execution_state();
    }
};

PseudoSlamCaptureGateConfig load_pseudo_slam_capture_gate_config()
{
    PseudoSlamCaptureGateConfig config;
    ros::param::param("~pseudo_slam_scan_min_point_count", config.scan_min_point_count, kPseudoSlamScanMinPointCount);
    ros::param::param("~pseudo_slam_scan_retry_interval_sec", config.scan_retry_interval_sec, kPseudoSlamScanRetryIntervalSec);
    ros::param::param("~pseudo_slam_capture_gate_stable_sample_count", config.stable_sample_count, kPseudoSlamCaptureGateStableSampleCount);
    ros::param::param("~pseudo_slam_capture_gate_poll_interval_sec", config.poll_interval_sec, kPseudoSlamCaptureGatePollIntervalSec);
    ros::param::param("~pseudo_slam_capture_gate_image_mean_diff_threshold", config.image_mean_diff_threshold, kPseudoSlamCaptureGateImageMeanDiffThreshold);
    ros::param::param("~pseudo_slam_capture_gate_log_interval_sec", config.log_interval_sec, kPseudoSlamCaptureGateLogIntervalSec);
    ros::param::param("~pseudo_slam_capture_gate_target_tolerance_mm", config.target_tolerance_mm, kPseudoSlamCaptureGateTargetToleranceMm);
    ros::param::param("~pseudo_slam_capture_gate_pose_delta_tolerance_mm", config.pose_delta_tolerance_mm, kPseudoSlamCaptureGatePoseDeltaToleranceMm);
    ros::param::param("~pseudo_slam_capture_gate_roi_min_x", config.roi_min_x, kPseudoSlamCaptureGateRoiMinX);
    ros::param::param("~pseudo_slam_capture_gate_roi_max_x", config.roi_max_x, kPseudoSlamCaptureGateRoiMaxX);
    ros::param::param("~pseudo_slam_capture_gate_roi_min_y", config.roi_min_y, kPseudoSlamCaptureGateRoiMinY);
    ros::param::param("~pseudo_slam_capture_gate_roi_max_y", config.roi_max_y, kPseudoSlamCaptureGateRoiMaxY);

    config.scan_min_point_count = std::max(1, config.scan_min_point_count);
    config.scan_retry_interval_sec = std::max(0.01, config.scan_retry_interval_sec);
    config.stable_sample_count = std::max(1, config.stable_sample_count);
    config.poll_interval_sec = std::max(0.01, config.poll_interval_sec);
    config.image_mean_diff_threshold = std::max(0.0, config.image_mean_diff_threshold);
    config.log_interval_sec = std::max(0.1, config.log_interval_sec);
    config.target_tolerance_mm = std::max(0.0f, config.target_tolerance_mm);
    config.pose_delta_tolerance_mm = std::max(0.0f, config.pose_delta_tolerance_mm);
    return config;
}

float load_pseudo_slam_planning_z_outlier_threshold_mm()
{
    float threshold_mm = kPseudoSlamPlanningZOutlierMm;
    ros::param::param("~pseudo_slam_planning_z_outlier_mm", threshold_mm, kPseudoSlamPlanningZOutlierMm);
    return std::max(0.0f, threshold_mm);
}

float load_pseudo_slam_outlier_secondary_plane_threshold_mm()
{
    float threshold_mm = kPseudoSlamPlanningZOutlierMm;
    ros::param::param(
        "~pseudo_slam_outlier_secondary_plane_z_threshold_mm",
        threshold_mm,
        kPseudoSlamPlanningZOutlierMm
    );
    return std::max(0.0f, threshold_mm);
}

float load_pseudo_slam_outlier_secondary_plane_neighbor_tolerance_mm()
{
    float tolerance_mm = kPseudoSlamOutlierSecondaryPlaneNeighborToleranceMm;
    ros::param::param(
        "~pseudo_slam_outlier_secondary_plane_neighbor_xy_tolerance_mm",
        tolerance_mm,
        kPseudoSlamOutlierSecondaryPlaneNeighborToleranceMm
    );
    return std::max(0.0f, tolerance_mm);
}

GlobalExecutionMode get_global_execution_mode()
{
    const int mode_value = global_execution_mode.load();
    if (mode_value == static_cast<int>(GlobalExecutionMode::kLiveVisual)) {
        return GlobalExecutionMode::kLiveVisual;
    }
    return GlobalExecutionMode::kSlamPrecomputed;
}

void set_global_execution_mode(GlobalExecutionMode mode)
{
    global_execution_mode.store(static_cast<int>(mode));
}

// 由计算模块得出的索驱路径点数量
int cabin_path_num;
// 全局变量，tcp套接字
int sockfd;
// 定义 cabin_state_buffer
uint8_t cabin_state_buffer[256];
// 判断线性模组是否回到原点的标志位
bool linear_module_get_back_origin =true;
// 转递消息的全局变量
// transform_msg是D453_3.py发送的包含待绑扎点位的消息，由pub发布，gpio.cpp中的 ros::Subscriber cabin_sub = nh_.subscribe("/cabin/lashing_request", 5, &GPIO_Lashing)
// 接收并执行回调函数GPIO_Lashing，完成绑扎
tie_robot_msgs::motion transform_msg;
//  绑扎动作的发布对象
ros::Publisher pub;
//  急停动作的发布对象
ros::Publisher pub_forced_stop;
//  索驱状态获取的发布者对象
ros::Publisher pub_cabin_data_upload;
ros::Publisher pub_test;
ros::Publisher pub_area_progress;
ros::Publisher pub_pseudo_slam_markers;

// 索驱状态被发布的全局变量
tie_robot_msgs::cabin_upload cabin_data_upload;
tie_robot_msgs::MotionControl motion_srv;
ros::ServiceClient AI_client;
ros::ServiceClient sg_live_visual_client;
ros::ServiceClient sg_precomputed_client;
ros::ServiceClient sg_precomputed_fast_client;
ros::ServiceClient motion_client;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;
std::vector<tie_robot_msgs::PointCoords> pseudo_slam_tf_points;
std::vector<uint8_t> pseudo_slam_ir_roi_frame;
ros::Time pseudo_slam_ir_roi_stamp;

// 暂停中断标志位 0为未启用暂停中断或恢复，1为启用暂停中断
int handle_pause_interrupt = 0;
std::mutex error_msg;
 // 等待恢复信号
bool stop_flag = false;
std::atomic<bool> moduan_work_flag{false};
bool checkerboard_jump_bind_enabled = false;

Cabin_State cabin_state;

bool lookup_gripper_from_base_link_transform(tf2::Transform& gripper_from_base_link);
bool lookup_current_gripper_from_cabin_transform(tf2::Transform& gripper_from_cabin);

// 机器人姿态反馈的结构体
// typedef struct {
//     float robot_pitch;
//     float robot_roll;
//     float robot_yaw;
// } Robot_Attitude;
// Robot_Attitude  robot_attitude;

//
void getSuoquTestTimeTxt(float pos)
{
    // 获取当前 OpenMP 墙钟时间
    double cur_time = omp_get_wtime();

    // 打开文件（追加模式）
    FILE *fp = fopen("/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/time_y_300.txt", "a");
    if (fp == NULL) {
        return;
    }

    // 写入：时间（秒）、传入的 time 参数、位置 pos
    fprintf(fp, "%.3f %.3f\n", cur_time, pos);

    fclose(fp);
    return;
}

void publish_area_progress(
    int current_area_index,
    int total_area_count,
    int just_finished_area_index,
    bool ready_for_next_area,
    bool all_done)
{
    tie_robot_msgs::AreaProgress progress_msg;
    progress_msg.current_area_index = current_area_index;
    progress_msg.total_area_count = total_area_count;
    progress_msg.just_finished_area_index = just_finished_area_index;
    progress_msg.ready_for_next_area = ready_for_next_area;
    progress_msg.all_done = all_done;
    pub_area_progress.publish(progress_msg);
}

std::vector<uint8_t> build_pseudo_slam_ir_roi_frame(const sensor_msgs::ImageConstPtr& msg)
{
    if (!msg || msg->width == 0 || msg->height == 0 || msg->data.empty()) {
        return {};
    }

    const PseudoSlamCaptureGateConfig config = load_pseudo_slam_capture_gate_config();
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const int roi_min_x = std::max(0, std::min(config.roi_min_x, width));
    const int roi_max_x = std::max(roi_min_x, std::min(config.roi_max_x, width));
    const int roi_min_y = std::max(0, std::min(config.roi_min_y, height));
    const int roi_max_y = std::max(roi_min_y, std::min(config.roi_max_y, height));
    if (roi_min_x >= roi_max_x || roi_min_y >= roi_max_y || msg->step == 0) {
        return {};
    }

    const std::string encoding = msg->encoding;
    const bool is_mono16 = encoding == "mono16" || encoding == "16UC1" || encoding == "16SC1";
    const bool is_color8 =
        encoding == "rgb8" || encoding == "bgr8" || encoding == "rgba8" || encoding == "bgra8";
    const size_t inferred_bytes_per_pixel =
        width > 0 ? std::max<size_t>(1, static_cast<size_t>(msg->step) / static_cast<size_t>(width)) : 1;
    const size_t bytes_per_pixel = is_mono16 ? 2 : inferred_bytes_per_pixel;

    std::vector<uint8_t> roi_frame;
    roi_frame.reserve(static_cast<size_t>(roi_max_x - roi_min_x) * static_cast<size_t>(roi_max_y - roi_min_y));

    for (int y = roi_min_y; y < roi_max_y; ++y) {
        const size_t row_offset = static_cast<size_t>(y) * static_cast<size_t>(msg->step);
        if (row_offset >= msg->data.size()) {
            break;
        }
        for (int x = roi_min_x; x < roi_max_x; ++x) {
            const size_t pixel_offset = row_offset + static_cast<size_t>(x) * bytes_per_pixel;
            if (pixel_offset >= msg->data.size()) {
                roi_frame.push_back(0);
                continue;
            }

            uint8_t gray_value = 0;
            if (is_mono16) {
                if (pixel_offset + 1 < msg->data.size()) {
                    const uint16_t mono16_value =
                        static_cast<uint16_t>(msg->data[pixel_offset]) |
                        (static_cast<uint16_t>(msg->data[pixel_offset + 1]) << 8);
                    gray_value = static_cast<uint8_t>(mono16_value >> 8);
                }
            } else if (is_color8 && pixel_offset + 2 < msg->data.size()) {
                const uint16_t channel_sum =
                    static_cast<uint16_t>(msg->data[pixel_offset]) +
                    static_cast<uint16_t>(msg->data[pixel_offset + 1]) +
                    static_cast<uint16_t>(msg->data[pixel_offset + 2]);
                gray_value = static_cast<uint8_t>(channel_sum / 3);
            } else {
                gray_value = msg->data[pixel_offset];
            }
            roi_frame.push_back(gray_value);
        }
    }
    return roi_frame;
}

void pseudo_slam_ir_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    std::vector<uint8_t> roi_frame = build_pseudo_slam_ir_roi_frame(msg);
    if (roi_frame.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(pseudo_slam_ir_image_mutex);
    pseudo_slam_ir_roi_frame = std::move(roi_frame);
    pseudo_slam_ir_roi_stamp = msg->header.stamp;
}

double compute_mean_abs_diff(
    const std::vector<uint8_t>& lhs,
    const std::vector<uint8_t>& rhs
)
{
    if (lhs.empty() || rhs.empty() || lhs.size() != rhs.size()) {
        return std::numeric_limits<double>::infinity();
    }

    double diff_sum = 0.0;
    for (size_t i = 0; i < lhs.size(); ++i) {
        diff_sum += std::abs(static_cast<int>(lhs[i]) - static_cast<int>(rhs[i]));
    }
    return diff_sum / static_cast<double>(lhs.size());
}

bool wait_for_pseudo_slam_capture_gate(
    int area_index,
    const Cabin_Point& cabin_point,
    float cabin_height
)
{
    printCurrentTime();
    ros_log_printf("Cabin_log: pseudo_slam scan_only区域%d等待最终采集门通过（机械静止+画面静止）。\n", area_index);

    bool has_previous_state = false;
    bool has_previous_frame = false;
    Cabin_State previous_state{};
    std::vector<uint8_t> previous_roi_frame;
    ros::Time previous_roi_stamp;
    int stable_sample_count = 0;
    double latest_mean_diff = std::numeric_limits<double>::infinity();
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;

    while (ros::ok()) {
        const PseudoSlamCaptureGateConfig config = load_pseudo_slam_capture_gate_config();
        Cabin_State current_state{};
        {
            std::lock_guard<std::mutex> lock(cabin_state_mutex);
            current_state = cabin_state;
        }

        std::vector<uint8_t> current_roi_frame;
        ros::Time current_roi_stamp;
        {
            std::lock_guard<std::mutex> lock(pseudo_slam_ir_image_mutex);
            current_roi_frame = pseudo_slam_ir_roi_frame;
            current_roi_stamp = pseudo_slam_ir_roi_stamp;
        }

        const bool target_reached =
            std::abs(current_state.X - cabin_point.x) <= config.target_tolerance_mm &&
            std::abs(current_state.Y - cabin_point.y) <= config.target_tolerance_mm &&
            std::abs(current_state.Z - cabin_height) <= config.target_tolerance_mm;
        const bool motion_stopped = current_state.motion_status == 0;
        const bool pose_delta_stable =
            has_previous_state &&
            std::abs(current_state.X - previous_state.X) <= config.pose_delta_tolerance_mm &&
            std::abs(current_state.Y - previous_state.Y) <= config.pose_delta_tolerance_mm &&
            std::abs(current_state.Z - previous_state.Z) <= config.pose_delta_tolerance_mm;
        const bool image_sample_updated = current_roi_stamp != previous_roi_stamp;

        bool image_stable = false;
        if (!current_roi_frame.empty() &&
            has_previous_frame &&
            image_sample_updated &&
            current_roi_frame.size() == previous_roi_frame.size()) {
            latest_mean_diff = compute_mean_abs_diff(current_roi_frame, previous_roi_frame);
            image_stable = latest_mean_diff <= config.image_mean_diff_threshold;
        } else if (current_roi_frame.empty()) {
            latest_mean_diff = std::numeric_limits<double>::infinity();
        }

        const bool capture_gate_sample_stable =
            target_reached && motion_stopped && pose_delta_stable && image_stable;
        if (capture_gate_sample_stable) {
            stable_sample_count++;
        } else if (image_sample_updated || current_roi_frame.empty()) {
            stable_sample_count = 0;
        }

        if (stable_sample_count >= config.stable_sample_count) {
            printCurrentTime();
            ros_log_printf("Cabin_log: pseudo_slam scan_only区域%d最终采集门已通过，开始请求视觉。\n", area_index);
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        const double log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - last_log_time).count();
        if (log_elapsed_sec >= config.log_interval_sec) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam scan_only区域%d等待稳定中，motion_status=%d，目标误差(dx,dy,dz)=(%.2f,%.2f,%.2f)，图像均差=%.3f，连续稳定样本=%d/%d。\n",
                area_index,
                current_state.motion_status,
                std::abs(current_state.X - cabin_point.x),
                std::abs(current_state.Y - cabin_point.y),
                std::abs(current_state.Z - cabin_height),
                latest_mean_diff,
                stable_sample_count,
                config.stable_sample_count
            );
            last_log_time = now;
        }

        if (has_previous_frame && !image_sample_updated) {
            previous_state = current_state;
            has_previous_state = true;
            ros::Duration(config.poll_interval_sec).sleep();
            continue;
        }

        previous_state = current_state;
        has_previous_state = true;
        if (!current_roi_frame.empty()) {
            previous_roi_frame = std::move(current_roi_frame);
            previous_roi_stamp = current_roi_stamp;
            has_previous_frame = true;
        }
        ros::Duration(config.poll_interval_sec).sleep();
    }
    return false;
}

typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float last_error;  // 上一次误差
    float prev_error;  // 上上次误差
    float prev_output; // 上一次输出
    float integral;    // 积分累积值
    float integral_max; // 积分上限
    float integral_min; // 积分下限
} PID_Controller;

// 更新PID控制器初始化，添加积分限位
PID_Controller pid_x = {1, 0.3, 0.3, 0, 0, 0, 0, 5.0, -15.0};  // 添加积分限位±5.0
PID_Controller pid_y = {1, 0.3, 0.3, 0, 0, 0, 0, 5.0, -15.0};  // 添加积分限位±5.0

std::vector<Cabin_Point> new_path;

// 退出信号函数
void signalHandler_cc(int signum)
{
    // 向其它节点发布急停信号
    printCurrentTime();
    ros_log_printf("Cabin_log:检测到ctrl+c，正在向索驱发送停止运动指令，关闭索驱系统，并广播急停消息。\n");
    // std_msgs::Float32 forced_stop_flag;
    // forced_stop_flag.data = 0;
    // pub_forced_stop.publish(forced_stop_flag);
    
    // 创建JSON对象并保存cabin_state的X,Y,Z值
    nlohmann::json state_json;
    {
        std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        state_json["X"] = cabin_state.X;
        state_json["Y"] = cabin_state.Y;
        state_json["Z"] = cabin_state.Z;
    }
    
    // 将JSON转换为字符串并保存到文件
    std::string json_str = state_json.dump();
    std::ofstream outfile("/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/cabin_state.json");
    if(outfile.is_open()) {
        outfile << json_str;
        outfile.close();
        ros_log_printf("Cabin_state JSON已保存到文件\n");
    } else {
        ros_log_printf("无法打开文件保存JSON数据\n");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    exit(2);
}

// float calculatePID(PID_Controller* pid, float setpoint, float feedback) {
//     float error = setpoint - feedback;
    
//     if(fabs(error) < 0.5f) {
//         return 0.0f;
//     }
    
//     // 积分累积
//     pid->integral += error;
    
//     // 积分限位
//     if(pid->integral > pid->integral_max) {
//         pid->integral = pid->integral_max;
//     } else if(pid->integral < pid->integral_min) {
//         pid->integral = pid->integral_min;
//     }
    
//     // 增量式PID计算
//     float delta_output = pid->Kp * (error - pid->last_error) +
//                         pid->Ki * error +
//                         pid->Kd * (error - 2 * pid->last_error + pid->prev_error);
    
//     // 计算当前输出
//     float output = pid->prev_output + delta_output;
//     if(output > 9.5) {
//         output = 9.5;
//     } else if(output < -9.5) {
//         output = -9.5;
//     }
//     // 更新状态
//     pid->prev_error = pid->last_error;
//     pid->last_error = error;
//     pid->prev_output = output;
    
//     // 打印调试信息
//     ros_log_printf("增量式PID调试 - 设定值: %.2f, 反馈值: %.2f, 误差: %.2f\n",
//            setpoint, feedback, error);
//     ros_log_printf("积分累积: %.2f (限位: %.2f~%.2f)\n",
//            pid->integral, pid->integral_min, pid->integral_max);
//     ros_log_printf("增量输出: %.2f (P: %.2f, I: %.2f, D: %.2f)\n",
//            delta_output, 
//            pid->Kp * (error - pid->last_error),
//            pid->Ki * error,
//            pid->Kd * (error - 2 * pid->last_error + pid->prev_error));
    
//     return output;
// }

float degreesToRadians(const float& degrees) {
    return degrees * (M_PI / 180.0);
}
void FtoU(float* myfloat)
{
    uint32_t temp32=*(uint32_t*)myfloat;
    FtoU_register[0]=temp32>>24;
    FtoU_register[1]=temp32>>16;
    FtoU_register[2]=temp32>>8;
    FtoU_register[3]=temp32;
}
void printTCPMoveFrame() {
    std::ostringstream frame_stream;
    frame_stream << std::uppercase << std::hex << std::setfill('0');
    for (int i = 0; i < 36; i++) {
        frame_stream << std::setw(2) << static_cast<unsigned int>(TCP_Move_Frame[i]);
        if (i < 35) {
            frame_stream << ' ';
        }
    }
    ros_log_printf("Cabin_log: TCP move frame: %s\n", frame_stream.str().c_str());
}

void printFrameBytes(const char* prefix, const uint8_t* data, size_t len)
{
    if (prefix == nullptr || data == nullptr) {
        return;
    }

    std::ostringstream frame_stream;
    frame_stream << prefix << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i) {
        frame_stream << std::setw(2) << static_cast<unsigned int>(data[i]);
        if (i + 1 < len) {
            frame_stream << ' ';
        }
    }
    ros_log_printf("%s\n", frame_stream.str().c_str());
}

std::vector<Cabin_Point> path_point_generate(float &marking_x,float &marking_y,float &zone_x,float &zone_y,float &robot_x_step,float &robot_y_step,float &cabin_height,float &cabin_speed)
{
    std::vector<Cabin_Point> Cabin_Coor;
    int x_coordinate_num=ceil(zone_x/robot_x_step);
    int y_coordinate_num=ceil(zone_y/robot_y_step);
    //x轴和y轴的坐标数
    Cabin_Coor.reserve(x_coordinate_num*y_coordinate_num);


    //从右上角出发开始绑扎,生成索驱xy移动的坐标点
    for(int i=0;i<y_coordinate_num;++i)
    {
        Cabin_Point cabin_point;
        if(i%2==0)
        {
            for(int j=0;j<x_coordinate_num;++j)
            {

                cabin_point.x=marking_x+robot_x_step*j;
                cabin_point.y=marking_y+robot_y_step*i;
                Cabin_Coor.push_back(cabin_point);
                // 新增：将路径点保存为JSON
                nlohmann::json path_json;
                path_json["path_points"] = nlohmann::json::array(); // 创建空数组
                for (const auto& point : Cabin_Coor) {
                    path_json["path_points"].push_back({  // 填充每个点的坐标
                        {"x", point.x},
                        {"y", point.y}
                    });
                }
                // 添加 cabin_height 和 cabin_speed 到 JSON 对象
                path_json["cabin_height"] = cabin_height;
                path_json["cabin_speed"] = cabin_speed;
                path_json["marking_x"] = marking_x;
                path_json["marking_y"] = marking_y;
                path_json["zone_x"] = zone_x;
                path_json["zone_y"] = zone_y;
                path_json["robot_x_step"] = robot_x_step;
                path_json["robot_y_step"] = robot_y_step;

                // 写入文件（路径可根据需求调整）
                std::ofstream json_file(path_points_json_file);
                if (json_file.is_open()) {
                    json_file << path_json.dump(4);  // 格式化输出（缩进4空格）
                    json_file.close();
                    // ros_log_printf("Cabin_log: 路径点已成功保存为JSON文件\n");
                } else {
                    ros_log_printf("Cabin_Error: 无法打开文件保存路径点\n");
                }
            }
        }
        else
        {
            for(int j=x_coordinate_num-1;j>=0;--j)
            {
               cabin_point.x=(marking_x+robot_x_step*j);
               cabin_point.y=(marking_y+robot_y_step*i);
               Cabin_Coor.push_back(cabin_point);

               // 新增：将路径点保存为JSON
                nlohmann::json path_json;
                path_json["path_points"] = nlohmann::json::array(); // 创建空数组
                for (const auto& point : Cabin_Coor) {
                    path_json["path_points"].push_back({  // 填充每个点的坐标
                        {"x", point.x},
                        {"y", point.y}
                    });
                }
                // 添加 cabin_height 和 cabin_speed 到 JSON 对象
                path_json["cabin_height"] = cabin_height;
                path_json["cabin_speed"] = cabin_speed;
                path_json["marking_x"] = marking_x;
                path_json["marking_y"] = marking_y;
                path_json["zone_x"] = zone_x;
                path_json["zone_y"] = zone_y;
                path_json["robot_x_step"] = robot_x_step;
                path_json["robot_y_step"] = robot_y_step;

                // 写入文件（路径可根据需求调整）
                std::ofstream json_file(path_points_json_file);
                if (json_file.is_open()) {
                    json_file << path_json.dump(4);  // 格式化输出（缩进4空格）
                    json_file.close();
                    // ros_log_printf("Cabin_log: 路径点已成功保存为JSON文件\n");
                } else {
                    ros_log_printf("Cabin_Error: 无法打开文件保存路径点\n");
                }
            }
        }

    }
    return Cabin_Coor;
}

void set_pseudo_slam_tf_points(const std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    (void)world_points;
    std::lock_guard<std::mutex> lock(pseudo_slam_tf_points_mutex);
    pseudo_slam_tf_points.clear();
}

// pseudo_slam点TF停发：RViz绑扎点TF只显示pointAI当前帧的Scepter_depth_frame原始点。

// pseudo_slam Marker 状态、扫描算法与棋盘格分类已抽到 src/suoqu/pseudo_slam_markers.cpp 和 src/suoqu/pseudo_slam_scan_processing.cpp。

bool lookup_gripper_from_base_link_transform(tf2::Transform& gripper_from_base_link)
{
    if (!tf_buffer_ptr) {
        return false;
    }

    try {
        const geometry_msgs::TransformStamped transform_msg = tf_buffer_ptr->lookupTransform(
            "gripper_frame",
            "base_link",
            ros::Time(0),
            ros::Duration(0.2)
        );
        tf2::fromMsg(transform_msg.transform, gripper_from_base_link);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: base_link->gripper_frame变换查找失败: %s", ex.what());
        return false;
    }
}

bool lookup_current_gripper_from_cabin_transform(tf2::Transform& gripper_from_cabin)
{
    if (!tf_buffer_ptr) {
        return false;
    }

    try {
        const geometry_msgs::TransformStamped transform_msg = tf_buffer_ptr->lookupTransform(
            "gripper_frame",
            "map",
            ros::Time(0),
            ros::Duration(0.2)
        );
        tf2::fromMsg(transform_msg.transform, gripper_from_cabin);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: map->gripper_frame变换查找失败: %s", ex.what());
        return false;
    }
}

bool transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const Cabin_Point& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_base_link,
    tie_robot_msgs::PointCoords& gripper_point
)
{
    tf2::Transform cabin_from_base_link;
    cabin_from_base_link.setIdentity();
    cabin_from_base_link.setOrigin(tf2::Vector3(
        static_cast<double>(cabin_point.x) / 1000.0,
        static_cast<double>(cabin_point.y) / 1000.0,
        static_cast<double>(cabin_height) / 1000.0
    ));
    const tf2::Transform gripper_from_cabin =
        gripper_from_base_link * cabin_from_base_link.inverse();
    const tf2::Vector3 point_in_map(
        static_cast<double>(world_point.World_coord[0]) / 1000.0,
        static_cast<double>(world_point.World_coord[1]) / 1000.0,
        static_cast<double>(world_point.World_coord[2]) / 1000.0
    );
    const tf2::Vector3 point_in_gripper_frame = gripper_from_cabin * point_in_map;

    gripper_point = world_point;
    gripper_point.World_coord[0] = static_cast<float>(point_in_gripper_frame.x() * 1000.0);
    gripper_point.World_coord[1] = static_cast<float>(point_in_gripper_frame.y() * 1000.0);
    gripper_point.World_coord[2] = static_cast<float>(point_in_gripper_frame.z() * 1000.0);
    return true;
}

bool transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const tie_robot_process::planning::CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_base_link,
    tie_robot_msgs::PointCoords& gripper_point
)
{
    const Cabin_Point legacy_cabin_point{cabin_point.x, cabin_point.y};
    return transform_cabin_world_point_to_planned_gripper_point(
        world_point,
        legacy_cabin_point,
        cabin_height,
        gripper_from_base_link,
        gripper_point
    );
}

// 规划分组、预计算局部点和执行账本读写已抽到独立实现片段，先做纯等价结构化收口。
#include "suoqu/planned_bind_and_memory.inc"

void moveTCPPosition(uint16_t Command_Word,float* TCP) {
    uint32_t MyTCP = 0;
    uint16_t checknum = 0;
    TCP_Move_Frame[0] = 0xEB;   // 设置通讯包头
    TCP_Move_Frame[1] = 0x90;  // 设置 TCP 位置运动启动指令
    TCP_Move_Frame[2] = 0x00;
    TCP_Move_Frame[3] = 0x12;
    TCP_Move_Frame[4]=Command_Word;
    TCP_Move_Frame[5]=0x00;
    
    for(int i=0;i<7;i++)
    {
        MyTCP=*(uint32_t*)(TCP+i);
        TCP_Move_Frame[4*i+9]=((MyTCP & 0xFF000000) >> 24);
        TCP_Move_Frame[4*i+8]=((MyTCP & 0x00FF0000) >> 16);
        TCP_Move_Frame[4*i+7]=((MyTCP & 0x0000FF00) >> 8);
        TCP_Move_Frame[4*i+6]=((MyTCP & 0xFF));
    }
    
    for(int j=0;j<34;j++)
    {
        checknum+=TCP_Move_Frame[j];
    }

    TCP_Move_Frame[35]=(checknum&0xFF00)>>8;
    TCP_Move_Frame[34]=checknum&0xFF;
    printTCPMoveFrame();
}

/*************************************************************************************************************************************************************
功能：驱动层函数，TCP连接初始化函数
输入：
输出：
**************************************************************************************************************************************************************/
bool connectToServer()
{
    if (!cabin_driver_enabled.load()) {
        sync_global_socket_fd_from_cabin_driver();
        const std::string error_detail = "索驱驱动已关闭，拒绝建立TCP连接";
        update_last_cabin_transport_error_detail(error_detail);
        log_cabin_warn_ros(error_detail);
        return false;
    }
    if (!g_cabin_driver) {
        g_cabin_driver = std::make_unique<tie_robot_hw::driver::CabinDriver>();
    }

    tie_robot_hw::driver::DriverError driver_error;
    if (!g_cabin_driver->start(&driver_error)) {
        sync_global_socket_fd_from_cabin_driver();
        const std::string error_detail =
            compose_cabin_driver_error_message("索驱TCP驱动连接失败", driver_error);
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return false;
    }

    sockfd = g_cabin_driver->socketFd();
    if (sockfd < 0) {
        const std::string error_detail = "索驱TCP驱动连接成功但socket句柄无效";
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return false;
    }

    clear_last_cabin_transport_error_detail();
    cabin_driver_last_state_stamp_sec.store(ros::Time::now().toSec());
    printCurrentTime();
    ros_log_printf("Cabin_log: TCP连接成功。\n");
    return true;
}

/*************************************************************************************************************************************************************
功能：驱动层函数，索驱系统指令集
输入：命令字 发送字节长度 接收字节长度
输出：
**************************************************************************************************************************************************************/
// int Frame_Generate(uint8_t* Control_Word ,int Tlen,int Rlen,int socket = sockfd) {
//     // 写操作带超时控制
//     fd_set writefds;
//     FD_ZERO(&writefds);
//     FD_SET(socket, &writefds);

//     struct timeval timeout;
//     timeout.tv_sec = 5;  // 设置超时时间为5秒
//     timeout.tv_usec = 0;

//     // 等待写操作准备好
//     int ret = select(socket + 1, NULL, &writefds, NULL, &timeout);
//     if (ret < 0) {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error: 无法向write中使用select方法。\n");
//         return -1; // 错误
//     } else if (ret == 0) {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error: TCP写入超时。\n");
//         return -1; // 超时
//     } else {
//         if (FD_ISSET(socket, &writefds)) {
//             ssize_t send_len = send(socket, Control_Word, Tlen,0);
//             if (send_len < 0) {
//                 printCurrentTime();
//                 ros_log_printf("Cabin_Error: TCP指令写入失败。\n");
//                 return -1; // 返回失败
//             }
//         }
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(300));  

//     // 读操作带超时控制
//     uint8_t buffer[256];
//     fd_set readfds;
//     FD_ZERO(&readfds);
//     FD_SET(socket, &readfds);

//     // 等待读操作准备好
//     ret = select(socket + 1, &readfds, NULL, NULL, &timeout);
//     if (ret < 0) {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error: 无法向read中使用select方法。\n");
//         return -1; // 错误
//     } else if (ret == 0) {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error: TCP读取超时。\n");
//         return -1; // 超时
//     } else {
//         if (FD_ISSET(socket, &readfds)) {
//             ssize_t recv_len = recv(socket, buffer, Rlen,0);
//             if (recv_len < 0) {
//                 printCurrentTime();
//                 ros_log_printf("Cabin_Error: TCP反馈读取失败。\n");
//                 return -1; // 返回失败
//             }
//             buffer[recv_len] = '\0'; // 添加字符串结束符
//         }
//     }
//     if (Control_Word == TCP_Normal_Connection)
//         for (int i = 0;i<256;i++)
//             cabin_state_buffer[i] = buffer[i];

//     return 0; // 成功
// }

int Frame_Generate(uint8_t* Control_Word, int Tlen, int Rlen, int socket = sockfd) 
{
    // 1. 参数检查
    if (Control_Word == nullptr || Tlen <= 0 || socket < 0) {
        ros_log_printf("Cabin_Error: Invalid input parameters.\n");
        return -1;
    }

    const uint16_t command_word =
        (static_cast<uint16_t>(Control_Word[2]) << 8) |
        static_cast<uint16_t>(Control_Word[3]);
    const std::string command_debug_context =
        build_tcp_command_debug_context(command_word, Control_Word, Tlen);

    const bool is_heartbeat_frame =
        Tlen == static_cast<int>(sizeof(TCP_Normal_Connection)) &&
        memcmp(Control_Word, TCP_Normal_Connection, Tlen) == 0;

    // 2. 写操作带超时控制 (Select)
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(socket, &writefds);

    struct timeval timeout{TCP_TIMEOUT_SEC, 0};

    int ret = select(socket + 1, nullptr, &writefds, nullptr, &timeout);
    if (ret < 0) {
        const std::string error_detail =
            "TCP写就绪等待失败: " + std::string(strerror(errno)) + "，" + command_debug_context;
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }
    if (ret == 0) {
        std::ostringstream oss;
        oss << "TCP写就绪等待超时(" << TCP_TIMEOUT_SEC << "s)，" << command_debug_context;
        const std::string error_detail = oss.str();
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }

    // 3. 循环发送，确保所有数据发出
    ssize_t total_sent = 0;
    while (total_sent < Tlen) {
        ssize_t sent = send(socket, Control_Word + total_sent, Tlen - total_sent, 0);
        if (sent == 0) {
            const std::string error_detail =
                "TCP发送过程中连接被关闭，" + command_debug_context;
            update_last_cabin_transport_error_detail(error_detail);
            printCurrentTime();
            ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
            log_cabin_error_ros(error_detail);
            return -1;
        }
        if (sent < 0) {
            const std::string error_detail =
                "TCP发送失败: " + std::string(strerror(errno)) + "，" + command_debug_context;
            update_last_cabin_transport_error_detail(error_detail);
            printCurrentTime();
            ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
            log_cabin_error_ros(error_detail);
            return -1;
        }
        total_sent += sent;
    }
    if (!is_heartbeat_frame) {
        printCurrentTime();
        printFrameBytes("Cabin_log: TCP sent frame: ", Control_Word, static_cast<size_t>(Tlen));
    }

    // 4. 写后延时：心跳查询帧不再额外阻塞，缩短 cabin_state 更新延迟。
    const int post_send_delay_ms = is_heartbeat_frame ? HEARTBEAT_WRITE_DELAY_MS : WRITE_DELAY_MS;
    if (post_send_delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(post_send_delay_ms));
    }

    // 5. 读操作带超时控制
    uint8_t buffer[RECV_BUFFER_SIZE];
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket, &readfds);

    timeout.tv_sec = TCP_TIMEOUT_SEC;
    timeout.tv_usec = 0;
    ret = select(socket + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        const std::string error_detail =
            "TCP读就绪等待失败: " + std::string(strerror(errno)) + "，" + command_debug_context;
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }
    if (ret == 0) {
        std::ostringstream oss;
        oss << "TCP读取等待超时(" << TCP_TIMEOUT_SEC << "s)，" << command_debug_context;
        const std::string error_detail = oss.str();
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }

    // 6. 循环接收或限制最大接收长度
    ssize_t recv_len = recv(socket, buffer, sizeof(buffer) - 1, 0); // 预留1字节给'\0'
    if (recv_len == 0) {
        const std::string error_detail =
            "TCP连接被对端关闭，" + command_debug_context;
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }
    if (recv_len < 0) {
        const std::string error_detail =
            "TCP接收失败: " + std::string(strerror(errno)) + "，" + command_debug_context;
        update_last_cabin_transport_error_detail(error_detail);
        printCurrentTime();
        ros_log_printf("Cabin_Error: %s\n", error_detail.c_str());
        log_cabin_error_ros(error_detail);
        return -1;
    }
    
    // 安全添加字符串结束符
    buffer[recv_len] = '\0';
    if (!is_heartbeat_frame) {
        printCurrentTime();
        printFrameBytes("Cabin_log: TCP recv frame: ", buffer, static_cast<size_t>(recv_len));
        const auto decoded_status = decode_tcp_protocol_status(command_word, buffer, recv_len);
        if (decoded_status.has_error) {
            cache_pending_tcp_status_error(decoded_status.status_word);
            const std::string protocol_status_message =
                format_tcp_protocol_status_message(decoded_status);
            const std::string error_detail =
                "索驱协议返回异常，" + protocol_status_message
                + "，recv_len=" + std::to_string(recv_len)
                + "，" + command_debug_context;
            update_last_cabin_transport_error_detail(error_detail);
            printCurrentTime();
            ros_log_printf("Cabin_Warn: %s。\n", error_detail.c_str());
            log_cabin_warn_ros(error_detail);
            return -2;
        } else {
            pending_tcp_status_word.store(0, std::memory_order_relaxed);
            pending_tcp_status_word_valid.store(false, std::memory_order_release);
            clear_last_cabin_transport_error_detail();
        }
    }

    // 7. 状态缓存逻辑（建议后续改为指令ID判断）
    // 假设 TCP_Normal_Connection 是一个特定的指令码，而不是指针
    if (is_heartbeat_frame) {
        memcpy(cabin_state_buffer, buffer, recv_len + 1);
    }

    return 0; // 成功
}


/*************************************************************************************************************************************************************
功能：驱动层函数，索驱系统指令集重写
输入：命令字 发送字节长度 接收字节长度
输出：
**************************************************************************************************************************************************************/
int Frame_Generate_With_Retry(uint8_t* Control_Word ,int Tlen,int Rlen,int socket = sockfd)
{
    int j = 0;
    bool reconnect_flag = false; //避免多次重连的标志位
    const bool keep_retrying_on_status_reject = is_motion_move_command_frame(Control_Word, Tlen);
    auto last_motion_reject_log_time =
        std::chrono::steady_clock::now() - std::chrono::seconds(MOTION_COMMAND_STATUS_RETRY_LOG_INTERVAL_SEC);
    for(int i = 0; i < 5; i++){
        int frame_result = 0;
        while (true) {
            frame_result = Frame_Generate(Control_Word, Tlen, Rlen,socket);
            if (frame_result == -2)
            {
                const uint16_t status_word = pending_tcp_status_word.load(std::memory_order_relaxed);
                const std::string recent_tcp_error_detail = get_last_cabin_failure_detail();
                if (keep_retrying_on_status_reject) {
                    const auto now = std::chrono::steady_clock::now();
                    const auto log_elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
                        now - last_motion_reject_log_time
                    ).count();
                    if (log_elapsed_sec >= MOTION_COMMAND_STATUS_RETRY_LOG_INTERVAL_SEC) {
                        printCurrentTime();
                        if (!recent_tcp_error_detail.empty()) {
                            ros_log_printf(
                                "Cabin_Warn: 索驱上位机暂未接受当前运动指令，最近一次底层错误：%s，继续等待索驱上位机接受当前运动指令后重试。\n",
                                recent_tcp_error_detail.c_str()
                            );
                        } else {
                            ros_log_printf(
                                "Cabin_Warn: 索驱上位机暂未接受当前运动指令，status_word=0x%04X，继续等待索驱上位机接受当前运动指令后重试。\n",
                                status_word
                            );
                        }
                        last_motion_reject_log_time = now;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                printCurrentTime();
                if (!recent_tcp_error_detail.empty()) {
                    ros_log_printf(
                        "Cabin_Error: 索驱上位机拒绝当前运动指令，最近一次底层错误：%s，立即停止本次等待。\n",
                        recent_tcp_error_detail.c_str()
                    );
                } else {
                    ros_log_printf(
                        "Cabin_Error: 索驱上位机拒绝当前运动指令，status_word=0x%04X，立即停止本次等待。\n",
                        status_word
                    );
                }
                return -2;
            }
            break;
        }
        if (frame_result < 0) 
        {
            const std::string recent_tcp_error_detail = get_last_cabin_failure_detail();
            printCurrentTime();
            if (!recent_tcp_error_detail.empty()) {
                ros_log_printf(
                    "Cabin_Error:第%d次发送指令失败。最近一次底层错误：%s\n",
                    i + 1,
                    recent_tcp_error_detail.c_str()
                );
            } else {
                ros_log_printf("Cabin_Error:第%d次发送指令失败。\n",i+1);
            }
            if(reconnect_flag == false)
            {
                reconnect_flag = true;
                printCurrentTime();
                ros_log_printf("Cabin_Error:正在尝试与索驱上位机重新创建TCP连接。\n");
                for(; j < 5; j++)
                {
                    if (!connectToServer()) 
                    {
                        printCurrentTime();
                        ros_log_printf("Cabin_Error:重新连接失败，正在尝试第%d次重新连接。\n",j+1);
                    }
                    else
                    {
                        printCurrentTime();
                        ros_log_printf("Cabin_Error:重新连接成功，正在尝试重新发送指令。\n");
                        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 延时0.2秒重试
                        j = 0;
                        break;
                    }
                }
                if(j == 5)
                {
                    std::string fatal_error_message =
                        "重新连接失败超过5次，可能存在网络问题，无法控制索驱系统";
                    const std::string failure_detail = get_last_cabin_failure_detail();
                    if (!failure_detail.empty()) {
                        fatal_error_message += "。最近一次底层错误：" + failure_detail;
                    }
                    printCurrentTime();
                    ros_log_printf("Cabin_Error:%s，正在紧急退出程序。\n", fatal_error_message.c_str());
                    log_cabin_error_ros(fatal_error_message);
                    persist_last_cabin_fatal_error_detail(fatal_error_message);
                    // std_msgs::Float32 forced_stop_flag;
                    // forced_stop_flag.data = 0;
                    // pub_forced_stop.publish(forced_stop_flag);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    emergency_exit_with_flush(4); //直接发布急停信号并关闭节点
                }
            }
        }
        else { 
        // std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
            clear_last_cabin_transport_error_detail();
            return 0; // 成功发送命令后退出函数
        }
    }     
    std::string fatal_error_message = "重新发送命令失败超过5次，可能无法控制索驱系统";
    const std::string failure_detail = get_last_cabin_failure_detail();
    if (!failure_detail.empty()) {
        fatal_error_message += "。最近一次底层错误：" + failure_detail;
    }
    printCurrentTime();
    ros_log_printf("Cabin_Error:%s，正在紧急退出程序。\n", fatal_error_message.c_str());
    log_cabin_error_ros(fatal_error_message);
    persist_last_cabin_fatal_error_detail(fatal_error_message);
    // std_msgs::Float32 forced_stop_flag;
    // forced_stop_flag.data = 0;
    // pub_forced_stop.publish(forced_stop_flag);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    emergency_exit_with_flush(4); //直接发布急停信号并关闭节点
}

void solve_stop(bool &stop_flag)
{
    if(stop_flag)
    {
        while (stop_flag)
        {
            
            {
                std::lock_guard<std::mutex> lock1(error_msg);
                stop_flag = (bool)handle_pause_interrupt;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 添加延时，避免CPU空转
        }
        {
            // 收到恢复信号
            printCurrentTime();
            ros_log_printf("Cabin_log:索驱暂停中断恢复,重新发送目标位置。\n");

            std::string driver_error_message;
            if (!move_cabin_pose_via_driver(
                    TCP_Move[0],
                    TCP_Move[1],
                    TCP_Move[2],
                    TCP_Move[3],
                    &driver_error_message)) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Error: 索驱暂停恢复时重新下发目标位置失败：%s\n",
                    driver_error_message.c_str()
                );
            }
        }
    }
    return;
}
/*************************************************************************************************************************************************************
功能: 等待索驱某个轴到达目标位置，并用状态字/运动状态确认结果
输入: 轴 位置
输出:
**************************************************************************************************************************************************************/
bool wait_cabin_axis_arrival(int Axis, double Target_position)
{
    clear_last_execution_wait_error_detail();
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;

    while (1)
    {
        uint16_t pending_tcp_status_word = 0;
        if (consume_pending_tcp_status_error(pending_tcp_status_word)) {
            std::ostringstream oss;
            oss << "等待轴" << Axis << "到位前检测到索驱状态字异常0x"
                << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
                << pending_tcp_status_word << std::dec
                << "，目标位置=" << Target_position;
            set_last_execution_wait_error_detail(oss.str());
            printCurrentTime();
            ros_log_printf(
                "Cabin_Error: 等待轴%d到位前检测到索驱状态字异常0x%04X，目标位置 %.2f，立即停止等待。\n",
                Axis,
                pending_tcp_status_word,
                Target_position
            );
            return false;
        }
        
        // ros_log_printf("cabin_state - X: %.2f, Y: %.2f, Z: %.2f, motion_status: %.2d, Target target_coordinate: %.2f\n, Axis: %d\n",
        // cabin_state.X, cabin_state.Y, cabin_state.Z, cabin_state.motion_status,Target_position,Axis);
        {
            // 如果当前轴体的坐标位置等于目标位置,且索驱完全停下来时运动结束。
            std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        // float cur_pos_x = cabin_state.X;
        // float cur_pos_y = cabin_state.Y;
        // getSuoquTestTimeTxt(cur_pos_y);
            if(Axis == AXIS_X && abs(cabin_state.X - Target_position)<25 && cabin_state.motion_status == 0)
                return true;
            if(Axis == AXIS_Y && abs(cabin_state.Y - Target_position)<25 && cabin_state.motion_status == 0)
                return true;
            if(Axis == AXIS_Z && abs(cabin_state.Z - Target_position)<25 && cabin_state.motion_status == 0)
                return true;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_sec >= kMotionWaitTimeoutSec) {
            float cur_x = 0.0f;
            float cur_y = 0.0f;
            float cur_z = 0.0f;
            int cur_motion_status = 0;
            {
                std::lock_guard<std::mutex> lock1(cabin_state_mutex);
                cur_x = cabin_state.X;
                cur_y = cabin_state.Y;
                cur_z = cabin_state.Z;
                cur_motion_status = cabin_state.motion_status;
            }
            std::ostringstream oss;
            oss << "等待轴" << Axis << "到位超时，目标位置=" << Target_position
                << "，当前(X,Y,Z)=(" << cur_x << "," << cur_y << "," << cur_z
                << ")，motion_status=" << cur_motion_status;
            set_last_execution_wait_error_detail(oss.str());
            printCurrentTime();
            ros_log_printf(
                "Cabin_Error: 等待轴%d到位超时，目标位置 %.2f，当前(X,Y,Z)=(%.2f,%.2f,%.2f)，motion_status=%d。\n",
                Axis,
                Target_position,
                cur_x,
                cur_y,
                cur_z,
                cur_motion_status
            );
            return false;
        }

        const auto log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (log_elapsed_sec >= kMotionWaitLogIntervalSec) {
            float cur_x = 0.0f;
            float cur_y = 0.0f;
            float cur_z = 0.0f;
            int cur_motion_status = 0;
            {
                std::lock_guard<std::mutex> lock1(cabin_state_mutex);
                cur_x = cabin_state.X;
                cur_y = cabin_state.Y;
                cur_z = cabin_state.Z;
                cur_motion_status = cabin_state.motion_status;
            }
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: 等待轴%d到位中，目标位置 %.2f，当前(X,Y,Z)=(%.2f,%.2f,%.2f)，motion_status=%d。\n",
                Axis,
                Target_position,
                cur_x,
                cur_y,
                cur_z,
                cur_motion_status
            );
            last_log_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); 

        {
            std::lock_guard<std::mutex> lock1(error_msg);
            stop_flag = bool(handle_pause_interrupt);
        }
        solve_stop(stop_flag);
    }
}

bool wait_cabin_axis_stable_arrival(int Axis, double Target_position, double target_tolerance_mm = kExecutionArrivalToleranceMm)
{
    clear_last_execution_wait_error_detail();
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    const double normalized_tolerance_mm = std::max(target_tolerance_mm, 1.0);
    int stable_sample_count = 0;
    double previous_axis_position = std::numeric_limits<double>::quiet_NaN();

    while (1)
    {
        uint16_t pending_tcp_status_word = 0;
        if (consume_pending_tcp_status_error(pending_tcp_status_word)) {
            std::ostringstream oss;
            oss << "执行层等待轴" << Axis << "到位前检测到索驱状态字异常0x"
                << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
                << pending_tcp_status_word << std::dec
                << "，目标位置=" << Target_position;
            set_last_execution_wait_error_detail(oss.str());
            printCurrentTime();
            ros_log_printf(
                "Cabin_Error: 执行层等待轴%d到位前检测到索驱状态字异常0x%04X，目标位置 %.2f，立即停止等待。\n",
                Axis,
                pending_tcp_status_word,
                Target_position
            );
            return false;
        }

        double current_axis_position = 0.0;
        float cur_x = 0.0f;
        float cur_y = 0.0f;
        float cur_z = 0.0f;
        int cur_motion_status = 0;
        {
            std::lock_guard<std::mutex> lock1(cabin_state_mutex);
            cur_x = cabin_state.X;
            cur_y = cabin_state.Y;
            cur_z = cabin_state.Z;
            cur_motion_status = cabin_state.motion_status;
            if (Axis == AXIS_X) {
                current_axis_position = static_cast<double>(cabin_state.X);
            } else if (Axis == AXIS_Y) {
                current_axis_position = static_cast<double>(cabin_state.Y);
            } else {
                current_axis_position = static_cast<double>(cabin_state.Z);
            }
        }

        const double axis_error_mm = std::abs(current_axis_position - Target_position);
        const double pose_delta_mm = std::isfinite(previous_axis_position)
            ? std::abs(current_axis_position - previous_axis_position)
            : std::numeric_limits<double>::infinity();
        previous_axis_position = current_axis_position;

        if (axis_error_mm < normalized_tolerance_mm &&
            (!std::isfinite(pose_delta_mm) || pose_delta_mm <= kExecutionArrivalPoseDeltaToleranceMm)) {
            stable_sample_count++;
            if (stable_sample_count >= kExecutionArrivalStableSampleCount) {
                clear_last_execution_wait_error_detail();
                return true;
            }
        } else {
            stable_sample_count = 0;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_sec >= kMotionWaitTimeoutSec) {
            std::ostringstream oss;
            oss << "执行层等待轴" << Axis << "到位超时，目标位置=" << Target_position
                << "，容差=" << normalized_tolerance_mm << "mm"
                << "，连续稳定样本=" << stable_sample_count << "/" << kExecutionArrivalStableSampleCount
                << "，当前位置变化量=" << (std::isfinite(pose_delta_mm) ? pose_delta_mm : -1.0)
                << "mm，当前(X,Y,Z)=(" << cur_x << "," << cur_y << "," << cur_z
                << ")，motion_status=" << cur_motion_status;
            set_last_execution_wait_error_detail(oss.str());
            printCurrentTime();
            ros_log_printf(
                "Cabin_Error: 执行层等待轴%d到位超时，目标位置 %.2f，容差%.1fmm，连续稳定样本=%d/%d，当前位置变化量=%.2fmm，当前(X,Y,Z)=(%.2f,%.2f,%.2f)，motion_status=%d。\n",
                Axis,
                Target_position,
                normalized_tolerance_mm,
                stable_sample_count,
                kExecutionArrivalStableSampleCount,
                std::isfinite(pose_delta_mm) ? pose_delta_mm : -1.0,
                cur_x,
                cur_y,
                cur_z,
                cur_motion_status
            );
            return false;
        }

        const auto log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (log_elapsed_sec >= kMotionWaitLogIntervalSec) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: 执行层等待轴%d到位中，目标位置 %.2f，容差%.1fmm，连续稳定样本=%d/%d，轴误差=%.2fmm，当前位置变化量=%.2fmm，当前(X,Y,Z)=(%.2f,%.2f,%.2f)，motion_status=%d。\n",
                Axis,
                Target_position,
                normalized_tolerance_mm,
                stable_sample_count,
                kExecutionArrivalStableSampleCount,
                axis_error_mm,
                std::isfinite(pose_delta_mm) ? pose_delta_mm : -1.0,
                cur_x,
                cur_y,
                cur_z,
                cur_motion_status
            );
            last_log_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 规划路径前下发速度
void change_cabin_speed_callback(const std_msgs::Float32 &debug_mes)
{
    global_cabin_speed = debug_mes.data > 0.0f ? debug_mes.data : 300.0f;
    printCurrentTime();
    ros_log_printf("Cabin_log: 修改索驱速度指令，速度为: %.2f\n", global_cabin_speed);
    return;
}

void checkerboard_jump_bind_callback(const std_msgs::Bool &debug_mes)
{
    checkerboard_jump_bind_enabled = debug_mes.data;
    printCurrentTime();
    if (checkerboard_jump_bind_enabled) {
        ros_log_printf("Cabin_log: 全局棋盘格跳绑已开启，仅执行checkerboard_parity==0的点。\n");
    } else {
        ros_log_printf("Cabin_log: 全局棋盘格跳绑已关闭，恢复全绑。\n");
    }
}

bool setExecutionModeService(
    tie_robot_msgs::SetExecutionMode::Request& req,
    tie_robot_msgs::SetExecutionMode::Response& res)
{
    if (req.execution_mode == 1) {
        set_global_execution_mode(GlobalExecutionMode::kLiveVisual);
    } else if (req.execution_mode == 0) {
        set_global_execution_mode(GlobalExecutionMode::kSlamPrecomputed);
    } else if (req.execution_mode != 0) {
        res.success = false;
        res.message = "未知的execution_mode，支持 0=slam_precomputed，1=live_visual";
        printCurrentTime();
        ros_log_printf("Cabin_Warn: 收到未知全局执行模式请求=%u。\n", req.execution_mode);
        return true;
    }

    const GlobalExecutionMode mode = get_global_execution_mode();
    res.success = true;
    res.message = std::string("全局执行模式已切换为") + global_execution_mode_name(mode);
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 全局执行模式已切换为%s。\n",
        global_execution_mode_name(mode)
    );
    return true;
}

// 路径规划
bool planGlobalMovePath(tie_robot_msgs::Pathguihua::Request &req, tie_robot_msgs::Pathguihua::Response &res)
{
    float mx = req.marking_x;
    float my = req.marking_y;
    float zxl = req.zone_x;
    float zyl = req.zone_y;
    float rxs = req.robot_x_step;
    float rys = req.robot_y_step;
    float cabin_height = req.height;
    float cabin_speed = get_global_cabin_move_speed_mm_per_sec();
    printCurrentTime();
    ros_log_printf("Cabin_log: 当前机器人起点位置为：(%f,%f,%f)\n",mx,my,cabin_height);
    // 生成路径点
    new_path = path_point_generate(mx, my, zxl, zyl, rxs, rys, cabin_height,cabin_speed);
    // 可视化路径 （待开发）
    res.success = true;
    res.message = "路径成功生成";
    printCurrentTime();
    ros_log_printf("Cabin_log: 新路径配置生成，包含 %zu 个路径点\n", new_path.size());
    return res.success;
}


void moduan_work_Callback(const std_msgs::Bool &debug_mes)
{
    moduan_work_flag.store(debug_mes.data, std::memory_order_release);
}
/*
    函数功能：暂停中断的回调函数,收到暂停中断信号后，设置暂停中断标志位为1，等待人工恢复。
*/
void pause_interrupt_Callback(const std_msgs::Float32 &debug_mes)
{
    if (debug_mes.data == 1.0 && !moduan_work_flag.load(std::memory_order_acquire))
    {
        printCurrentTime();
        ros_log_printf("Cabin_log: 索驱人工暂停，正在等待人工恢复。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 1;
            std::string driver_error_message;
            stop_cabin_motion_via_driver(&driver_error_message);
        }
    }
    else
    {
        printCurrentTime();
        ros_log_printf("Cabin_log: 绑扎正在进行，不可暂停索驱。\n");
    }

    return;
}

void solve_stop_Callback(const std_msgs::Float32 &debug_mes)
{
    if (debug_mes.data == 1.0 && !moduan_work_flag.load(std::memory_order_acquire))
    {
        printCurrentTime();
        ros_log_printf("Cabin_log: 恢复索驱运动，重置暂停标志。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 0;
        }
    }
    else
    {
        printCurrentTime();
        ros_log_printf("Cabin_log: 绑扎正在进行，不可恢复索驱中断。\n");
    }
    return;
}

/*
    函数功能：急停的回调函数,收到急停信号后，强制停止索驱运动，关闭节点。
*/
void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    ros_log_printf("Cabin_log:急停信号已被触发，正在强制停止索驱运动，关闭节点。\n");

    std::string driver_error_message;
    stop_cabin_motion_via_driver(&driver_error_message);
    if (g_cabin_driver) {
        g_cabin_driver->stop();
    }
    sync_global_socket_fd_from_cabin_driver();
    ros::shutdown();
}
/*
    函数功能：索驱单点运动的回调函数
*/
bool cabin_single_move(tie_robot_msgs::SingleMove::Request& req, tie_robot_msgs::SingleMove::Response& res)
{
    float cabin_x = req.x;
    float cabin_y = req.y;
    float cabin_height = req.z;
    float cabin_speed = req.speed;

    TCP_Move[0] = cabin_speed;
    TCP_Move[1] = cabin_x;
    TCP_Move[2] = cabin_y;
    TCP_Move[3] = cabin_height;
    if (cabin_height < 350) {
        printCurrentTime(); 
        ros_log_printf("Cabin_Error: 索驱Z轴距离设置超限。\n");
        res.message = "Cabin_log: 索驱单点运动失败（Z轴距离设置超限），位置为(" + std::to_string(cabin_x) + "," + std::to_string(cabin_y) + "," + std::to_string(cabin_height) + ")。";
        res.success = false;
        return true;
    }
// getSuoquTestTimeTxt(cabin_y);
    std::string driver_error_message;
    if (!move_cabin_pose_via_driver(cabin_speed, cabin_x, cabin_y, cabin_height, &driver_error_message)) {
        res.message = "Cabin_log: 索驱单点运动失败，原因：" + driver_error_message;
        res.success = false;
        return true;
    }
// getSuoquTestTimeTxt(cabin_y);
    if (!wait_cabin_axis_arrival(AXIS_X, cabin_x) ||
        !wait_cabin_axis_arrival(AXIS_Y, cabin_y) ||
        !wait_cabin_axis_arrival(AXIS_Z, cabin_height)) {
        const std::string wait_error_detail = get_last_cabin_failure_detail();
        res.message = wait_error_detail.empty()
            ? "Cabin_log: 索驱单点运动失败，原因：等待到位未确认完成。"
            : "Cabin_log: 索驱单点运动失败，原因：" + wait_error_detail;
        res.success = false;
        return true;
    }

    printCurrentTime();
    ros_log_printf("Cabin_log: 索驱单点运动完成，位置为(%lf,%lf,%lf)。\n",cabin_x,cabin_y,cabin_height);
    res.message = "Cabin_log: 索驱单点运动完成，位置为(" + std::to_string(cabin_x) + "," + std::to_string(cabin_y) + "," + std::to_string(cabin_height) + ")。";
    res.success = true;
    return true;
}

bool cabin_driver_raw_move_service(tie_robot_msgs::SingleMove::Request& req, tie_robot_msgs::SingleMove::Response& res)
{
    std::string driver_error_message;
    res.success = move_cabin_pose_via_driver(req.speed, req.x, req.y, req.z, &driver_error_message);
    res.message = res.success
        ? "索驱驱动层 raw move 已下发。"
        : driver_error_message;
    return true;
}

/*
    函数功能：索驱单轴运动
// */
// void atom_cabin_move_single(const tie_robot_msgs::cabin_move_single::ConstPtr& msg)
// {
//     char C;
//     if(msg->Axis ==AXIS_X && (msg->target_distance < -1500 || msg->target_distance > 1500))
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱X轴距离设3置超限。\n");
//         return ;
//     }
//     if(msg->Axis ==AXIS_Y && (msg->target_distance < -3000 || msg->target_distance > 3000))
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱Y轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->Axis ==AXIS_Z && (msg->target_distance < -60 || msg->target_distance > 1200))
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱Z轴距离设置超限。\n");
//         return ;
//     }
    
//     {
//         std::lock_guard<std::mutex> lock1(cabin_state_mutex);
//         TCP_Move[1] = cabin_state.X;
//         TCP_Move[2] = cabin_state.Y;
//         TCP_Move[3] = cabin_state.Z;
//     }

//     printCurrentTime();
//     ros_log_printf("Cabin_log:索驱单轴运动请求已被触发，");
//     if(msg->Axis == AXIS_X)
//     {
//         ros_log_printf("X轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[1] = msg->target_distance;
//         C= 'X';
//     }
//     if(msg->Axis == AXIS_Y)
//     {
//         ros_log_printf("Y轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[2] = msg->target_distance;
//         C= 'Y';
//     }
//     if(msg->Axis == AXIS_Z)
//     {
//         ros_log_printf("Z轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[3] = msg->target_distance;
//         C= 'Z';
//     }
//     else
//     {
//         ros_log_printf("轴体参数不正确。\n");
//         return ;
//     }
//     // 移动索驱单轴
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     wait_cabin_axis_arrival(msg->Axis,msg->target_distance);
//     printCurrentTime();
//     ros_log_printf("Cabin_log:索驱%C轴已移动至目标位置%lfmm处。\n",C,msg->target_distance);
//     return ;
// }

/*
    函数功能：索驱三轴运动调试
*/
// void atom_cabin_move_all(const tie_robot_msgs::cabin_move_all::ConstPtr& msg)
// {
//     if(msg->cabin_X_distance < -1500 || msg->cabin_X_distance > 1500)
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱X轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->cabin_Y_distance < -3000 || msg->cabin_Y_distance > 3000)
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱Y轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->cabin_Z_distance < -60 || msg->cabin_Z_distance > 1200)
//     {
//         printCurrentTime();
//         ros_log_printf("Cabin_Error:索驱Z轴距离设置超限。\n");
//         return ;
//     }
    
//     printCurrentTime();
//     ros_log_printf("Cabin_log:索驱正在移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
    
//     // 先将Z轴回到原点处
//     {
//         std::lock_guard<std::mutex> lock1(cabin_state_mutex);
//         TCP_Move[1] = cabin_state.X;
//         TCP_Move[2] = cabin_state.Y;
//     }
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         TCP_Move[3] = 0;
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     wait_cabin_axis_arrival(AXIS_Z,0);
//     // 当Z轴回到原点后，再移动X轴和Y轴
//     TCP_Move[1] = msg->cabin_X_distance;
//     TCP_Move[2] = msg->cabin_Y_distance;
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     wait_cabin_axis_arrival(AXIS_X,msg->cabin_X_distance);
//     wait_cabin_axis_arrival(AXIS_Y,msg->cabin_Y_distance);
//     // 当X,Y轴抵达位置后，再移动Z轴运动
//     TCP_Move[3] = msg->cabin_Z_distance;
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     wait_cabin_axis_arrival(AXIS_Z,msg->cabin_Z_distance);
//     printCurrentTime();
//     ros_log_printf("Cabin_log:索驱已移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
//     return;
// }

// 订阅外部姿态传感器计算的索驱姿态
void cabin_gesture_get(const tie_robot_msgs::linear_module_upload& msg)
{
    cabin_state.cabin_x_gesture = msg.x_gesture;
    cabin_state.cabin_y_gesture = msg.y_gesture;
}

void cabin_data_upload_callback(const tie_robot_msgs::cabin_upload::ConstPtr& msg)
{
    if (!msg) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(cabin_state_mutex);
        cabin_state.X = static_cast<float>(msg->cabin_state_X);
        cabin_state.Y = static_cast<float>(msg->cabin_state_Y);
        cabin_state.Z = static_cast<float>(msg->cabin_state_Z);
        cabin_state.motion_status = msg->motion_status;
        cabin_state.device_alarm = msg->device_alarm;
        cabin_state.internal_calc_error = msg->internal_calc_error;
    }
    cabin_driver_last_state_stamp_sec.store(ros::Time::now().toSec());
}

bool load_configured_path(
    std::vector<Cabin_Point>& con_path,
    float& cabin_height,
    float& cabin_speed
)
{
    std::ifstream infile(path_points_json_file);
    if (!infile.is_open()) {
        throw std::runtime_error("无法打开路径点JSON文件");
    }

    nlohmann::json path_json;
    infile >> path_json;
    if (!path_json.contains("path_points") || !path_json["path_points"].is_array()) {
        throw std::runtime_error("JSON文件格式错误，缺少'path_points'数组");
    }
    if (!path_json.contains("cabin_height") || !path_json.contains("cabin_speed")) {
        throw std::runtime_error("JSON文件格式错误，缺少'cabin_height'或'cabin_speed'");
    }

    cabin_height = path_json["cabin_height"].get<float>();
    cabin_speed = path_json["cabin_speed"].get<float>();
    con_path = new_path;
    if (con_path.empty()) {
        printCurrentTime();
        ros_log_printf("Cabin_log: 使用path_points.json中的历史路径。\n");
        for (const auto& json_point : path_json["path_points"]) {
            Cabin_Point point;
            point.x = json_point["x"].get<float>();
            point.y = json_point["y"].get<float>();
            con_path.push_back(point);
        }
    }
    return !con_path.empty();
}

std::string build_path_signature(
    const std::vector<Cabin_Point>& con_path,
    float cabin_height,
    float cabin_speed
)
{
    std::ostringstream canonical_path_stream;
    canonical_path_stream << std::fixed << std::setprecision(3);
    canonical_path_stream << "height=" << cabin_height
                          << "|speed=" << cabin_speed
                          << "|count=" << con_path.size();
    for (const auto& cabin_point : con_path) {
        canonical_path_stream << "|" << cabin_point.x << "," << cabin_point.y;
    }

    const std::string canonical_path = canonical_path_stream.str();
    unsigned long long signature_hash = 1469598103934665603ULL;
    for (const unsigned char ch : canonical_path) {
        signature_hash ^= static_cast<unsigned long long>(ch);
        signature_hash *= 1099511628211ULL;
    }

    std::ostringstream signature_stream;
    signature_stream << "fnv1a64:" << std::hex << std::nouppercase << signature_hash;
    return signature_stream.str();
}

bool load_current_path_signature_for_execution(
    std::string& current_path_signature,
    std::string& error_message
)
{
    current_path_signature.clear();
    error_message.clear();

    try {
        std::vector<Cabin_Point> con_path;
        float cabin_height = 0.0f;
        float cabin_speed = 0.0f;
        if (!load_configured_path(con_path, cabin_height, cabin_speed)) {
            error_message = "当前路径为空，无法校验path_signature";
            return false;
        }
        current_path_signature = build_path_signature(con_path, cabin_height, cabin_speed);
        return true;
    } catch (const std::exception& ex) {
        error_message = std::string("无法加载当前路径配置，无法校验path_signature：") + ex.what();
        return false;
    }
}

bool transform_scepter_camera_point_to_map(
    const tie_robot_msgs::PointCoords& point,
    tie_robot_msgs::PointCoords& world_point
)
{
    if (!tf_buffer_ptr) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: Scepter_depth_frame->map变换失败: tf_buffer未初始化");
        return false;
    }

    geometry_msgs::PointStamped camera_point;
    camera_point.header.stamp = ros::Time(0);
    camera_point.header.frame_id = "Scepter_depth_frame";
    camera_point.point.x = static_cast<double>(point.World_coord[0]) / 1000.0;
    camera_point.point.y = static_cast<double>(point.World_coord[1]) / 1000.0;
    camera_point.point.z = static_cast<double>(point.World_coord[2]) / 1000.0;

    try {
        const geometry_msgs::PointStamped map_point = tf_buffer_ptr->transform(
            camera_point,
            "map",
            ros::Duration(0.2)
        );
        world_point = point;
        world_point.World_coord[0] = static_cast<float>(map_point.point.x * 1000.0);
        world_point.World_coord[1] = static_cast<float>(map_point.point.y * 1000.0);
        world_point.World_coord[2] = static_cast<float>(map_point.point.z * 1000.0);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: Scepter_depth_frame->map点坐标变换失败: %s", ex.what());
        return false;
    }
}

bool build_world_point_from_scan_response(
    const tie_robot_msgs::PointCoords& point,
    int point_index,
    tie_robot_msgs::PointCoords& world_point
)
{
    if (!transform_scepter_camera_point_to_map(point, world_point)) {
        return false;
    }
    world_point.idx = point_index;
    return true;
}

void assign_global_indices(std::vector<tie_robot_msgs::PointCoords>& world_points)
{
    int global_index = 1;
    for (auto& world_point : world_points) {
        world_point.idx = global_index++;
    }
}

bool should_persist_pseudo_slam_bind_artifacts(PseudoSlamScanStrategy scan_strategy)
{
    return scan_strategy == PseudoSlamScanStrategy::kFixedManualWorkspace;
}

bool run_pseudo_slam_scan(
    std::vector<Cabin_Point> con_path,
    float cabin_height,
    float cabin_speed,
    PseudoSlamScanStrategy scan_strategy,
    bool enable_capture_gate,
    std::string& message)
{
    std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
    std::vector<tie_robot_msgs::PointCoords> merged_world_points;
    set_pseudo_slam_tf_points({});
    clear_pseudo_slam_markers();
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_ir_image_mutex);
        pseudo_slam_ir_roi_frame.clear();
        pseudo_slam_ir_roi_stamp = ros::Time();
    }
    const int total_area_count = static_cast<int>(con_path.size());
    if (total_area_count == 0) {
        message = "没有可执行区域，无法扫描建图";
        return false;
    }
    Cabin_Point path_origin = con_path.front();
    set_pseudo_slam_marker_path_origin(path_origin);
    int total_scan_area_count = 0;
    switch (scan_strategy) {
        case PseudoSlamScanStrategy::kFixedManualWorkspace: {
            Cabin_Point fixed_scan_pose{};
            fixed_scan_pose.x = kPseudoSlamFixedManualWorkspaceScanXmm;
            fixed_scan_pose.y = kPseudoSlamFixedManualWorkspaceScanYmm;
            const float fixed_scan_height = kPseudoSlamFixedManualWorkspaceScanZmm;
            const float fixed_scan_speed = get_global_cabin_move_speed_mm_per_sec();
            path_origin = fixed_scan_pose;
            set_pseudo_slam_marker_path_origin(path_origin);

            total_scan_area_count = 1;
            publish_area_progress(1, total_scan_area_count, 0, false, false);
            TCP_Move[0] = fixed_scan_speed;
            TCP_Move[1] = fixed_scan_pose.x;
            TCP_Move[2] = fixed_scan_pose.y;
            TCP_Move[3] = fixed_scan_height;
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam固定工作区扫描移动到(%f,%f,%f)，固定速度=%.1f。\n",
                TCP_Move[1],
                TCP_Move[2],
                TCP_Move[3],
                TCP_Move[0]
            );
            std::string driver_error_message;
            if (!move_cabin_pose_via_driver(
                    fixed_scan_speed,
                    fixed_scan_pose.x,
                    fixed_scan_pose.y,
                    fixed_scan_height,
                    &driver_error_message)) {
                message = compose_cabin_failure_message("固定工作区扫描下发索驱移动指令失败");
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_X, fixed_scan_pose.x)) {
                message = "固定工作区扫描因索驱X轴到位超时而中止";
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_Y, fixed_scan_pose.y)) {
                message = "固定工作区扫描因索驱Y轴到位超时而中止";
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_Z, fixed_scan_height)) {
                message = "固定工作区扫描因索驱Z轴到位超时而中止";
                return false;
            }
            if (enable_capture_gate) {
                if (!wait_for_pseudo_slam_capture_gate(1, fixed_scan_pose, fixed_scan_height)) {
                    printCurrentTime();
                    ros_log_printf("Cabin_Warn: pseudo_slam固定工作区扫描等待最终采集门时ROS关闭，结束扫描。\n");
                    message = "固定工作区扫描在等待最终采集门时被中断";
                    return false;
                }
            } else {
                printCurrentTime();
                ros_log_printf("Cabin_log: pseudo_slam固定工作区扫描已关闭最终采集门，直接请求一次视觉。\n");
            }

            tie_robot_msgs::ProcessImage scan_srv;
            scan_srv.request.request_mode = kProcessImageModeScanOnly;
            if (!AI_client.call(scan_srv) || !scan_srv.response.success) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: pseudo_slam固定工作区扫描单帧视觉失败，消息：%s\n",
                    scan_srv.response.message.c_str()
                );
                message = "固定工作区扫描视觉请求失败";
                return false;
            }

            std::vector<tie_robot_msgs::PointCoords> frame_world_points;
            int point_index = 1;
            for (const auto& point : scan_srv.response.PointCoordinatesArray) {
                tie_robot_msgs::PointCoords world_point;
                if (build_world_point_from_scan_response(point, point_index, world_point)) {
                    frame_world_points.push_back(world_point);
                    point_index++;
                }
            }
            const int raw_frame_point_count = static_cast<int>(frame_world_points.size());
            const std::vector<tie_robot_msgs::PointCoords> scan_history_points = merged_world_points;
            std::vector<tie_robot_msgs::PointCoords> accepted_scan_points =
                filter_new_scan_points_against_existing_xy_tolerance(
                    frame_world_points,
                    scan_history_points,
                    kPseudoSlamScanDuplicateXYToleranceMm
                );
            merged_world_points.insert(
                merged_world_points.end(),
                accepted_scan_points.begin(),
                accepted_scan_points.end()
            );
            if (merged_world_points.empty()) {
                message = "固定工作区扫描视觉返回0个可用点";
                return false;
            }

            assign_global_indices(merged_world_points);
            set_pseudo_slam_tf_points(merged_world_points);
            publish_pseudo_slam_markers(merged_world_points);

            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam固定工作区扫描识别完成，固定位姿(%f,%f,%f)，当前帧原始点%d个，去重后保留%d个。\n",
                fixed_scan_pose.x,
                fixed_scan_pose.y,
                fixed_scan_height,
                raw_frame_point_count,
                static_cast<int>(accepted_scan_points.size())
            );
            break;
        }

        case PseudoSlamScanStrategy::kSingleCenter: {
            Cabin_Point global_scan_center{};
            float global_scan_height = 0.0f;
            std::string global_scan_pose_error;
            if (!compute_pseudo_slam_global_scan_pose(
                    con_path,
                    cabin_height,
                    global_scan_center,
                    global_scan_height,
                    global_scan_pose_error)) {
                message = global_scan_pose_error;
                return false;
            }
            path_origin = global_scan_center;
            set_pseudo_slam_marker_path_origin(path_origin);

            total_scan_area_count = 1;
            publish_area_progress(1, total_scan_area_count, 0, false, false);
            TCP_Move[0] = cabin_speed;
            TCP_Move[1] = global_scan_center.x;
            TCP_Move[2] = global_scan_center.y;
            TCP_Move[3] = global_scan_height;
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam全局中心扫描移动到(%f,%f,%f)，规划起点高度=%.1f，扫描高度偏移=%.1f。\n",
                TCP_Move[1],
                TCP_Move[2],
                TCP_Move[3],
                cabin_height,
                kPseudoSlamGlobalScanHeightOffsetMm
            );
            std::string driver_error_message;
            if (!move_cabin_pose_via_driver(
                    cabin_speed,
                    global_scan_center.x,
                    global_scan_center.y,
                    global_scan_height,
                    &driver_error_message)) {
                message = compose_cabin_failure_message("扫描建图下发索驱移动指令失败");
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_X, global_scan_center.x)) {
                message = "扫描建图因索驱X轴到位超时而中止";
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_Y, global_scan_center.y)) {
                message = "扫描建图因索驱Y轴到位超时而中止";
                return false;
            }
            if (!wait_cabin_axis_arrival(AXIS_Z, global_scan_height)) {
                message = "扫描建图因索驱Z轴到位超时而中止";
                return false;
            }
            if (enable_capture_gate) {
                if (!wait_for_pseudo_slam_capture_gate(1, global_scan_center, global_scan_height)) {
                    printCurrentTime();
                    ros_log_printf("Cabin_Warn: pseudo_slam全局中心扫描等待最终采集门时ROS关闭，结束扫描。\n");
                    message = "扫描建图在等待最终采集门时被中断";
                    return false;
                }
            } else {
                printCurrentTime();
                ros_log_printf("Cabin_log: pseudo_slam全局中心扫描已关闭最终采集门，直接请求一次视觉。\n");
            }

            tie_robot_msgs::ProcessImage scan_srv;
            scan_srv.request.request_mode = kProcessImageModeScanOnly;
            if (!AI_client.call(scan_srv) || !scan_srv.response.success) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: pseudo_slam全局中心单帧视觉失败，消息：%s\n",
                    scan_srv.response.message.c_str()
                );
                message = "扫描建图单帧视觉请求失败";
                return false;
            }

            std::vector<tie_robot_msgs::PointCoords> frame_world_points;
            int point_index = 1;
            for (const auto& point : scan_srv.response.PointCoordinatesArray) {
                tie_robot_msgs::PointCoords world_point;
                if (build_world_point_from_scan_response(point, point_index, world_point)) {
                    frame_world_points.push_back(world_point);
                    point_index++;
                }
            }
            const int raw_frame_point_count = static_cast<int>(frame_world_points.size());
            const std::vector<tie_robot_msgs::PointCoords> scan_history_points = merged_world_points;
            std::vector<tie_robot_msgs::PointCoords> accepted_scan_points =
                filter_new_scan_points_against_existing_xy_tolerance(
                    frame_world_points,
                    scan_history_points,
                    kPseudoSlamScanDuplicateXYToleranceMm
                );
            merged_world_points.insert(
                merged_world_points.end(),
                accepted_scan_points.begin(),
                accepted_scan_points.end()
            );
            if (merged_world_points.empty()) {
                message = "扫描建图单帧视觉返回0个可用点";
                return false;
            }

            assign_global_indices(merged_world_points);
            set_pseudo_slam_tf_points(merged_world_points);
            publish_pseudo_slam_markers(merged_world_points);

            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam全局中心单帧识别完成，工作区中心(%f,%f)，扫描高度=%f，当前帧原始点%d个，去重后保留%d个。\n",
                global_scan_center.x,
                global_scan_center.y,
                global_scan_height,
                raw_frame_point_count,
                static_cast<int>(accepted_scan_points.size())
            );
            break;
        }

        case PseudoSlamScanStrategy::kMultiPose: {
            total_scan_area_count = static_cast<int>(con_path.size());
            std::vector<PseudoSlamOverlapCluster> scan_clusters;
            int area_index = 0;
            for (const auto& cabin_point : con_path) {
                ++area_index;
                const int scan_pose_index = area_index;
                publish_area_progress(area_index, total_scan_area_count, 0, false, false);
                TCP_Move[0] = cabin_speed;
                TCP_Move[1] = cabin_point.x;
                TCP_Move[2] = cabin_point.y;
                TCP_Move[3] = cabin_height;
                printCurrentTime();
                ros_log_printf(
                    "Cabin_log: pseudo_slam多扫描位区域%d移动到(%f,%f,%f)。\n",
                    area_index,
                    TCP_Move[1],
                    TCP_Move[2],
                    TCP_Move[3]
                );
                std::string driver_error_message;
                if (!move_cabin_pose_via_driver(
                        cabin_speed,
                        cabin_point.x,
                        cabin_point.y,
                        cabin_height,
                        &driver_error_message)) {
                    message = compose_cabin_failure_message("扫描建图下发索驱移动指令失败");
                    return false;
                }
                if (!wait_cabin_axis_arrival(AXIS_X, cabin_point.x)) {
                    message = "扫描建图因索驱X轴到位超时而中止";
                    return false;
                }
                if (!wait_cabin_axis_arrival(AXIS_Y, cabin_point.y)) {
                    message = "扫描建图因索驱Y轴到位超时而中止";
                    return false;
                }
                if (!wait_cabin_axis_arrival(AXIS_Z, cabin_height)) {
                    message = "扫描建图因索驱Z轴到位超时而中止";
                    return false;
                }
                if (enable_capture_gate) {
                    if (!wait_for_pseudo_slam_capture_gate(area_index, cabin_point, cabin_height)) {
                        printCurrentTime();
                        ros_log_printf(
                            "Cabin_Warn: pseudo_slam多扫描位区域%d等待最终采集门时ROS关闭，结束扫描。\n",
                            area_index
                        );
                        message = "扫描建图在等待最终采集门时被中断";
                        return false;
                    }
                } else {
                    printCurrentTime();
                    ros_log_printf(
                        "Cabin_log: pseudo_slam多扫描位区域%d已关闭最终采集门，直接请求一次视觉。\n",
                        area_index
                    );
                }

                tie_robot_msgs::ProcessImage scan_srv;
                scan_srv.request.request_mode = kProcessImageModeScanOnly;
                if (!AI_client.call(scan_srv) || !scan_srv.response.success) {
                    printCurrentTime();
                    ros_log_printf(
                        "Cabin_Warn: pseudo_slam多扫描位区域%d单帧视觉失败，消息：%s\n",
                        area_index,
                        scan_srv.response.message.c_str()
                    );
                    message = "扫描建图单帧视觉请求失败";
                    return false;
                }

                std::vector<tie_robot_msgs::PointCoords> frame_world_points;
                int point_index = 1;
                for (const auto& point : scan_srv.response.PointCoordinatesArray) {
                    tie_robot_msgs::PointCoords world_point;
                    if (build_world_point_from_scan_response(point, point_index, world_point)) {
                        frame_world_points.push_back(world_point);
                        point_index++;
                    }
                }
                const int raw_frame_point_count = static_cast<int>(frame_world_points.size());
                merge_frame_points_into_overlap_clusters(
                    frame_world_points,
                    scan_pose_index,
                    scan_clusters,
                    kPseudoSlamScanDuplicateXYToleranceMm
                );
                merged_world_points = build_scan_cluster_representatives(scan_clusters);
                assign_global_indices(merged_world_points);
                set_pseudo_slam_tf_points(merged_world_points);
                publish_pseudo_slam_markers(merged_world_points);

                printCurrentTime();
                ros_log_printf(
                    "Cabin_log: pseudo_slam多扫描位区域%d识别完成，当前帧原始点%d个，当前累计代表点%d个。\n",
                    area_index,
                    raw_frame_point_count,
                    static_cast<int>(merged_world_points.size())
                );
            }
            if (merged_world_points.empty()) {
                message = "多扫描位建图结束，但没有形成任何可用代表点";
                return false;
            }
            break;
        }
    }

    assign_global_indices(merged_world_points);
    set_pseudo_slam_tf_points(merged_world_points);
    publish_pseudo_slam_markers(merged_world_points);
    std::unordered_map<int, PseudoSlamCheckerboardInfo> merged_checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(merged_world_points, path_origin);
    const std::vector<tie_robot_msgs::PointCoords> planning_z_outlier_points =
        collect_pseudo_slam_planning_z_outliers(merged_world_points);
    const std::unordered_set<int> outlier_secondary_plane_global_indices =
        collect_pseudo_slam_outlier_secondary_plane_global_indices(planning_z_outlier_points);
    std::vector<tie_robot_msgs::PointCoords> secondary_plane_outlier_points;
    secondary_plane_outlier_points.reserve(planning_z_outlier_points.size());
    for (const auto& outlier_point : planning_z_outlier_points) {
        if (outlier_secondary_plane_global_indices.count(outlier_point.idx) > 0) {
            secondary_plane_outlier_points.push_back(outlier_point);
        }
    }
    const std::unordered_set<int> outlier_line_global_indices =
        collect_pseudo_slam_outlier_line_global_indices(planning_z_outlier_points);
    std::vector<tie_robot_msgs::PointCoords> planning_world_points = filter_pseudo_slam_planning_outliers(merged_world_points);
    planning_world_points = filter_pseudo_slam_points_near_outlier_secondary_plane_members(
        planning_world_points,
        secondary_plane_outlier_points
    );
    const std::unordered_set<int> outlier_column_neighbor_blocked_global_indices =
        collect_pseudo_slam_outlier_column_neighbor_blocked_global_indices(
            planning_world_points,
            planning_z_outlier_points
        );
    planning_world_points = filter_pseudo_slam_points_near_outlier_columns(
        planning_world_points,
        planning_z_outlier_points
    );
    std::unordered_map<int, PseudoSlamCheckerboardInfo> checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    planning_world_points = filter_pseudo_slam_non_checkerboard_points(
        planning_world_points,
        checkerboard_info_by_idx
    );
    checkerboard_info_by_idx = build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    merged_checkerboard_info_by_idx = sync_merged_checkerboard_membership_with_planning(
        merged_checkerboard_info_by_idx,
        checkerboard_info_by_idx
    );
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_marker_state_mutex);
        pseudo_slam_marker_outlier_global_indices.clear();
        pseudo_slam_marker_outlier_secondary_plane_global_indices =
            outlier_secondary_plane_global_indices;
        pseudo_slam_marker_outlier_line_global_indices = outlier_line_global_indices;
        pseudo_slam_marker_outlier_column_neighbor_global_indices =
            outlier_column_neighbor_blocked_global_indices;
        for (const auto& world_point : merged_world_points) {
            const auto merged_checkerboard_it = merged_checkerboard_info_by_idx.find(world_point.idx);
            if (merged_checkerboard_it == merged_checkerboard_info_by_idx.end() ||
                !merged_checkerboard_it->second.is_checkerboard_member) {
                if (outlier_column_neighbor_blocked_global_indices.count(world_point.idx) == 0) {
                    pseudo_slam_marker_outlier_global_indices.insert(world_point.idx);
                }
            }
        }
    }
    tf2::Transform gripper_from_base_link;
    if (!lookup_gripper_from_base_link_transform(gripper_from_base_link)) {
        message = "无法获取base_link到gripper_frame的TF变换，动态绑扎规划失败";
        return false;
    }

    const auto dynamic_bind_planner_config = build_dynamic_bind_planner_config();
    std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries =
        tie_robot_process::planning::build_dynamic_bind_area_entries_from_scan_world(
            planning_world_points,
            {path_origin.x, path_origin.y},
            cabin_height,
            gripper_from_base_link,
            dynamic_bind_planner_config
        );
    int bind_group_count = 0;
    int bind_point_count = 0;
    const int grouped_area_count = static_cast<int>(bind_area_entries.size());
    for (const auto& area_entry : bind_area_entries) {
        for (const auto& bind_group : area_entry.bind_groups) {
            bind_group_count++;
            bind_point_count += static_cast<int>(bind_group.bind_points_world.size());
        }
    }

    tie_robot_process::planning::sort_bind_area_entries_by_snake_rows(
        bind_area_entries,
        dynamic_bind_planner_config.snake_row_tolerance_mm
    );
    BindExecutionPathOriginPose execution_path_origin =
        tie_robot_process::planning::build_dynamic_bind_execution_path_origin(
            bind_area_entries,
            {path_origin.x, path_origin.y},
            cabin_height,
            dynamic_bind_planner_config.bind_execution_cabin_min_z_mm
        );
    if (!bind_area_entries.empty()) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: pseudo_slam绑扎路径已改为按索驱坐标系左下角原点蛇形排序，首个执行区域area_index=%d，执行路径原点=(%f,%f,%f)。\n",
            bind_area_entries.front().area_index,
            execution_path_origin.x,
            execution_path_origin.y,
            execution_path_origin.z
        );
    }

    if (!should_persist_pseudo_slam_bind_artifacts(scan_strategy)) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_Warn: 当前不是固定识别位姿扫描，已拒绝写入本地绑扎点JSON；"
            "仅固定识别位姿扫描允许更新pseudo_slam_points.json、"
            "pseudo_slam_bind_path.json和bind_execution_memory.json。\n"
        );
        message =
            "当前不是固定识别位姿扫描，本次视觉结果不会覆盖本地绑扎点JSON；"
            "请通过固定识别位姿扫描入口更新pseudo_slam_bind_path.json";
        return false;
    }

    std::string pseudo_slam_points_error;
    std::string pseudo_slam_bind_path_error;
    const std::string scan_session_id = std::to_string(ros::Time::now().toNSec());
    const std::string path_signature = build_path_signature(con_path, cabin_height, cabin_speed);
    const bool pseudo_slam_points_written = write_pseudo_slam_points_json(
        merged_world_points,
        merged_checkerboard_info_by_idx,
        checkerboard_info_by_idx,
        outlier_secondary_plane_global_indices,
        outlier_line_global_indices,
        outlier_column_neighbor_blocked_global_indices,
        scan_session_id,
        path_signature,
        &pseudo_slam_points_error
    );
    const bool pseudo_slam_bind_path_written = write_pseudo_slam_bind_path_json(
        bind_area_entries,
        checkerboard_info_by_idx,
        execution_path_origin,
        cabin_height,
        cabin_speed,
        scan_session_id,
        path_signature,
        &pseudo_slam_bind_path_error
    );
    if (!pseudo_slam_points_written || !pseudo_slam_bind_path_written) {
        if (!pseudo_slam_points_written) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam_points.json写入失败：%s\n",
                pseudo_slam_points_error.c_str()
            );
        }
        if (!pseudo_slam_bind_path_written) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: pseudo_slam_bind_path.json写入失败：%s\n",
                pseudo_slam_bind_path_error.c_str()
            );
        }
        std::ostringstream oss;
        oss << "扫描建图完成，但扫描产物发布不完整";
        if (!pseudo_slam_points_written) {
            oss << "，pseudo_slam_points.json写入失败";
        }
        if (!pseudo_slam_bind_path_written) {
            oss << "，pseudo_slam_bind_path.json写入失败";
        }
        oss << "；扫描完成后未重置bind_execution_memory.json";
        message = oss.str();
        return false;
    }

    BindExecutionMemory bind_execution_memory =
        reset_bind_execution_memory_for_scan_session(scan_session_id, path_signature, path_origin);
    bind_execution_memory.path_origin.x = execution_path_origin.x;
    bind_execution_memory.path_origin.y = execution_path_origin.y;
    std::string bind_execution_memory_error;
    if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: 扫描完成后重置bind_execution_memory.json失败：%s\n",
            bind_execution_memory_error.c_str()
        );
        message =
            "扫描建图完成，但bind_execution_memory.json重置失败；为避免旧执行记忆与新扫描结果混用，已将本次扫描判定为失败";
        return false;
    }
    publish_area_progress(total_scan_area_count, total_scan_area_count, total_scan_area_count, false, true);
    std::ostringstream oss;
    oss << "扫描建图完成，pseudo_slam_points.json=" << merged_world_points.size()
        << "个点，pseudo_slam_bind_path.json=" << grouped_area_count
        << "个区域/" << bind_group_count << "个分组/" << bind_point_count << "个绑扎点";
    message = oss.str();
    return true;
}

std::vector<tie_robot_msgs::PointCoords> load_bind_points_from_group_json(
    const nlohmann::json& group_json
)
{
    std::vector<tie_robot_msgs::PointCoords> points;
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        return points;
    }

    int point_index = 1;
    for (const auto& point_json : group_json["points"]) {
        tie_robot_msgs::PointCoords point;
        point.idx = point_json.value("local_idx", point_json.value("idx", point_index));
        point.Pix_coord[0] = 0;
        point.Pix_coord[1] = 0;
        point.World_coord[0] = point_json.value("world_x", point_json.value("x", 0.0f));
        point.World_coord[1] = point_json.value("world_y", point_json.value("y", 0.0f));
        point.World_coord[2] = point_json.value("world_z", point_json.value("z", 0.0f));
        point.Angle = point_json.value("angle", -45.0f);
        point.is_shuiguan = false;
        points.push_back(point);
        point_index++;
    }
    return points;
}

const nlohmann::json* find_planned_bind_area_json_by_area_index(
    const nlohmann::json& bind_path_json,
    int area_index
)
{
    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        return nullptr;
    }

    for (const auto& area_json : bind_path_json["areas"]) {
        if (area_json.value("area_index", -1) == area_index) {
            return &area_json;
        }
    }
    return nullptr;
}

tie_robot_msgs::PointCoords build_world_point_from_bind_point_json(
    const nlohmann::json& point_json,
    int fallback_idx = 1
)
{
    tie_robot_msgs::PointCoords point;
    point.idx = point_json.value(
        "global_idx",
        point_json.value("local_idx", point_json.value("idx", fallback_idx))
    );
    point.Pix_coord[0] = 0;
    point.Pix_coord[1] = 0;
    point.World_coord[0] = point_json.value("world_x", point_json.value("x", 0.0f));
    point.World_coord[1] = point_json.value("world_y", point_json.value("y", 0.0f));
    point.World_coord[2] = point_json.value("world_z", point_json.value("z", 0.0f));
    point.Angle = point_json.value("angle", -45.0f);
    point.is_shuiguan = false;
    return point;
}

std::unordered_map<int, tie_robot_msgs::PointCoords> collect_planned_area_world_points_by_global_index(
    const nlohmann::json& area_json
)
{
    std::unordered_map<int, tie_robot_msgs::PointCoords> planned_points_by_global_index;
    if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
        return planned_points_by_global_index;
    }

    int fallback_idx = 1;
    for (const auto& group_json : area_json["groups"]) {
        if (!group_json.contains("points") || !group_json["points"].is_array()) {
            continue;
        }
        for (const auto& point_json : group_json["points"]) {
            const int global_idx = point_json.value("global_idx", -1);
            if (global_idx <= 0) {
                fallback_idx++;
                continue;
            }
            if (planned_points_by_global_index.count(global_idx) > 0) {
                fallback_idx++;
                continue;
            }
            planned_points_by_global_index.emplace(
                global_idx,
                build_world_point_from_bind_point_json(point_json, fallback_idx)
            );
            fallback_idx++;
        }
    }
    return planned_points_by_global_index;
}

nlohmann::json build_live_visual_execution_points_from_planned_area(
    int area_index,
    const nlohmann::json& planned_area_json,
    const nlohmann::json& classified_points_json,
    const std::unordered_map<int, tie_robot_msgs::PointCoords>& planned_points_by_global_index
)
{
    nlohmann::json execution_points = nlohmann::json::array();
    if (!planned_area_json.contains("groups") || !planned_area_json["groups"].is_array()) {
        return execution_points;
    }

    Cabin_Point planned_cabin_point{};
    float planned_cabin_z = 0.0f;
    const bool has_planned_cabin_pose =
        planned_area_json.contains("cabin_pose") &&
        planned_area_json["cabin_pose"].is_object();
    if (has_planned_cabin_pose) {
        const auto& cabin_pose_json = planned_area_json["cabin_pose"];
        planned_cabin_point.x = cabin_pose_json.value("x", 0.0f);
        planned_cabin_point.y = cabin_pose_json.value("y", 0.0f);
        planned_cabin_z = cabin_pose_json.value("z", 0.0f);
    }
    tf2::Transform gripper_from_base_link;
    const bool has_gripper_transform =
        has_planned_cabin_pose && lookup_gripper_from_base_link_transform(gripper_from_base_link);

    std::unordered_map<int, nlohmann::json> best_live_point_by_global_index;
    std::unordered_map<int, double> best_live_point_score_by_global_index;
    int fallback_idx = 1;
    for (const auto& point_json : classified_points_json) {
        const int global_idx = point_json.value("global_idx", -1);
        const auto planned_point_it = planned_points_by_global_index.find(global_idx);
        if (planned_point_it == planned_points_by_global_index.end()) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: live_visual区域%d点global_idx=%d不在当前区域扫描参考点中，跳过该点。\n",
                area_index,
                global_idx
            );
            fallback_idx++;
            continue;
        }

        const tie_robot_msgs::PointCoords live_world_point =
            build_world_point_from_bind_point_json(point_json, fallback_idx);
        const double refine_dx_mm = std::fabs(
            static_cast<double>(live_world_point.World_coord[0]) -
            static_cast<double>(planned_point_it->second.World_coord[0])
        );
        const double refine_dy_mm = std::fabs(
            static_cast<double>(live_world_point.World_coord[1]) -
            static_cast<double>(planned_point_it->second.World_coord[1])
        );
        const double refine_dz_mm = std::fabs(
            static_cast<double>(live_world_point.World_coord[2]) -
            static_cast<double>(planned_point_it->second.World_coord[2])
        );
        if (refine_dx_mm > kLiveVisualMicroAdjustXYToleranceMm ||
            refine_dy_mm > kLiveVisualMicroAdjustXYToleranceMm ||
            refine_dz_mm > kLiveVisualMicroAdjustZToleranceMm) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: live_visual区域%d点global_idx=%d超出xyz微调范围(dx=%.1fmm,dy=%.1fmm,dz=%.1fmm；xy阈值=%.1fmm,z阈值=%.1fmm)，忽略本次视觉修正，保留扫描参考点。\n",
                area_index,
                global_idx,
                refine_dx_mm,
                refine_dy_mm,
                refine_dz_mm,
                static_cast<double>(kLiveVisualMicroAdjustXYToleranceMm),
                static_cast<double>(kLiveVisualMicroAdjustZToleranceMm)
            );
            fallback_idx++;
            continue;
        }

        const double refine_score = refine_dx_mm + refine_dy_mm + refine_dz_mm;
        const auto best_live_point_it = best_live_point_score_by_global_index.find(global_idx);
        if (best_live_point_it == best_live_point_score_by_global_index.end() ||
            refine_score < best_live_point_it->second) {
            best_live_point_score_by_global_index[global_idx] = refine_score;
            best_live_point_by_global_index[global_idx] = point_json;
        }
        fallback_idx++;
    }

    for (const auto& group_json : planned_area_json["groups"]) {
        if (!group_json.contains("points") || !group_json["points"].is_array()) {
            continue;
        }
        for (const auto& point_json : group_json["points"]) {
            nlohmann::json execution_point_json = point_json;
            const int global_idx = point_json.value("global_idx", -1);
            const auto live_point_it = best_live_point_by_global_index.find(global_idx);
            if (live_point_it != best_live_point_by_global_index.end()) {
                const auto& live_point_json = live_point_it->second;
                const float live_world_x = live_point_json.value("world_x", live_point_json.value("x", 0.0f));
                const float live_world_y = live_point_json.value("world_y", live_point_json.value("y", 0.0f));
                const float live_world_z = live_point_json.value("world_z", live_point_json.value("z", 0.0f));
                execution_point_json["world_x"] = live_world_x;
                execution_point_json["world_y"] = live_world_y;
                execution_point_json["world_z"] = live_world_z;
            }
            if (has_gripper_transform) {
                assign_planned_gripper_coords_to_bind_point_json(
                    execution_point_json,
                    planned_cabin_point,
                    planned_cabin_z,
                    gripper_from_base_link
                );
            }
            execution_points.push_back(execution_point_json);
        }
    }

    return execution_points;
}

bool load_precomputed_local_points_from_group_json(
    const nlohmann::json& group_json,
    std::vector<tie_robot_msgs::PointCoords>& local_points,
    std::string& failure_reason
)
{
    local_points.clear();
    failure_reason.clear();
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        failure_reason = "当前组缺少points";
        return false;
    }

    int point_index = 1;
    int missing_local_coord_count = 0;
    int out_of_range_local_coord_count = 0;
    for (const auto& point_json : group_json["points"]) {
        const int point_idx = point_json.value("local_idx", point_json.value("idx", point_index));
        const bool has_local_x = point_json.contains("x") && point_json["x"].is_number();
        const bool has_local_y = point_json.contains("y") && point_json["y"].is_number();
        const bool has_local_z = point_json.contains("z") && point_json["z"].is_number();
        if (!has_local_x || !has_local_y || !has_local_z) {
            missing_local_coord_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: 预生成组点idx=%d缺少TCP局部坐标x/y/z，跳过当前点。\n",
                point_idx
            );
            point_index++;
            continue;
        }

        tie_robot_msgs::PointCoords point;
        point.idx = point_idx;
        point.Pix_coord[0] = 0;
        point.Pix_coord[1] = 0;
        point.World_coord[0] = point_json["x"].get<float>();
        point.World_coord[1] = point_json["y"].get<float>();
        point.World_coord[2] = point_json["z"].get<float>();
        point.Angle = point_json.value("angle", -45.0f);
        point.is_shuiguan = false;
        if (!is_local_bind_point_in_range(point)) {
            out_of_range_local_coord_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: 预生成组点idx=%d的TCP局部坐标(%.2f,%.2f,%.2f)超出虎口范围[x:0~%.2f,y:0~%.2f,z:0~%.2f]，跳过当前点。\n",
                point.idx,
                point.World_coord[0],
                point.World_coord[1],
                point.World_coord[2],
                kTravelMaxXMm,
                kTravelMaxYMm,
                kTravelMaxZMm
            );
            point_index++;
            continue;
        }

        local_points.push_back(point);
        point_index++;
    }

    if (!local_points.empty()) {
        return true;
    }

    if (missing_local_coord_count > 0 && out_of_range_local_coord_count <= 0) {
        failure_reason = "当前组缺少TCP局部坐标x/y/z";
        return false;
    }
    if (out_of_range_local_coord_count > 0 && missing_local_coord_count <= 0) {
        failure_reason = "当前组JSON局部点全部超出虎口范围";
        return false;
    }
    if (missing_local_coord_count > 0 || out_of_range_local_coord_count > 0) {
        failure_reason = "当前组JSON局部点无可执行点";
        return false;
    }

    failure_reason = "当前组无预生成局部点";
    return false;
}

nlohmann::json collect_dispatched_precomputed_point_jsons(
    const nlohmann::json& group_json,
    const std::vector<tie_robot_msgs::PointCoords>& dispatched_points
)
{
    nlohmann::json dispatched_point_jsons = nlohmann::json::array();
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        return dispatched_point_jsons;
    }

    std::unordered_set<int> dispatched_point_indices;
    for (const auto& dispatched_point : dispatched_points) {
        dispatched_point_indices.insert(dispatched_point.idx);
    }

    for (const auto& point_json : group_json["points"]) {
        const int point_idx = point_json.value("local_idx", point_json.value("idx", -1));
        if (dispatched_point_indices.count(point_idx) <= 0) {
            continue;
        }
        dispatched_point_jsons.push_back(point_json);
    }

    return dispatched_point_jsons;
}

bool find_nearest_bind_area_for_current_cabin_pose(
    const nlohmann::json& areas_json,
    float current_cabin_x,
    float current_cabin_y,
    int& nearest_area_json_index,
    float& nearest_distance_mm
)
{
    nearest_area_json_index = -1;
    nearest_distance_mm = 0.0f;
    double best_distance_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < areas_json.size(); ++i) {
        const auto& area_json = areas_json[i];
        if (!area_json.contains("cabin_pose") || !area_json["cabin_pose"].is_object()) {
            continue;
        }
        const auto& cabin_pose = area_json["cabin_pose"];
        const double dx = static_cast<double>(cabin_pose.value("x", 0.0f)) - static_cast<double>(current_cabin_x);
        const double dy = static_cast<double>(cabin_pose.value("y", 0.0f)) - static_cast<double>(current_cabin_y);
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            nearest_area_json_index = static_cast<int>(i);
        }
    }

    if (nearest_area_json_index < 0) {
        return false;
    }

    nearest_distance_mm = static_cast<float>(std::sqrt(best_distance_sq));
    return true;
}

float get_current_area_bind_test_cabin_speed()
{
    const float base_speed = get_global_cabin_move_speed_mm_per_sec();
    return std::max(
        base_speed * kCurrentAreaBindTestCabinSpeedMultiplier,
        kCurrentAreaBindTestMinCabinSpeedMmPerSec
    );
}

bool run_current_area_bind_from_scan_test(std::string& message)
{
    clear_pseudo_slam_marker_execution_state();
    ScopedPseudoSlamMarkerExecutionStateClear scoped_marker_execution_state_clear;
    std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
    nlohmann::json points_json;
    nlohmann::json bind_path_json;
    BindExecutionMemory bind_execution_memory;
    std::string current_path_signature;
    std::string bind_execution_memory_error;
    if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error)) {
        message = bind_execution_memory_error;
        return false;
    }
    if (!load_current_path_signature_for_execution(current_path_signature, message)) {
        return false;
    }
    if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message)) {
        return false;
    }
    const std::unordered_set<int> blocked_global_indices =
        collect_blocked_execution_global_indices_from_points_json(points_json);
    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        message = "pseudo_slam_bind_path.json格式错误";
        return false;
    }

    const auto& areas_json = bind_path_json["areas"];
    if (areas_json.empty()) {
        message = "pseudo_slam_bind_path.json没有可执行区域，请先确认扫描分组成功";
        return false;
    }

    float current_cabin_x = 0.0f;
    float current_cabin_y = 0.0f;
    {
        std::lock_guard<std::mutex> lock(cabin_state_mutex);
        current_cabin_x = cabin_state.X;
        current_cabin_y = cabin_state.Y;
    }

    int nearest_area_json_index = -1;
    float nearest_distance_mm = 0.0f;
    if (!find_nearest_bind_area_for_current_cabin_pose(
            areas_json,
            current_cabin_x,
            current_cabin_y,
            nearest_area_json_index,
            nearest_distance_mm)) {
        message = "无法根据当前索驱位置匹配pseudo_slam区域";
        return false;
    }

    const auto& area_json = areas_json[nearest_area_json_index];
    const int total_area_count = static_cast<int>(areas_json.size());
    const int area_index = area_json.value("area_index", nearest_area_json_index + 1);
    const auto& cabin_pose = area_json["cabin_pose"];
    const float cabin_x = cabin_pose.value("x", 0.0f);
    const float cabin_y = cabin_pose.value("y", 0.0f);
    const float cabin_height = cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
    const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_height);
    const float fast_cabin_speed = get_current_area_bind_test_cabin_speed();

    publish_area_progress(area_index, total_area_count, 0, false, false);
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 当前区域预计算直执行选择区域%d，当前位置(%f,%f)，目标区域(%f,%f,%f)，距离%.1fmm，索驱快速度%.1f。\n",
        area_index,
        current_cabin_x,
        current_cabin_y,
        cabin_x,
        cabin_y,
        move_cabin_z,
        nearest_distance_mm,
        fast_cabin_speed
    );

    std::string driver_error_message;
    if (!move_cabin_pose_via_driver(
            fast_cabin_speed,
            cabin_x,
            cabin_y,
            move_cabin_z,
            &driver_error_message)) {
        message = compose_cabin_failure_message("当前区域预计算直执行下发索驱移动指令失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_X, cabin_x)) {
        message = compose_cabin_failure_message("当前区域预计算直执行因索驱X轴到位失败而中止");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Y, cabin_y)) {
        message = compose_cabin_failure_message("当前区域预计算直执行因索驱Y轴到位失败而中止");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_cabin_z)) {
        message = compose_cabin_failure_message("当前区域预计算直执行因索驱Z轴到位失败而中止");
        return false;
    }

    if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
        message = "当前区域缺少groups，无法执行预计算绑扎测试";
        return false;
    }

    const std::unordered_set<int> area_global_indices = collect_global_indices_from_area_json(area_json);
    set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});

    int executed_group_count = 0;
    int skipped_group_count = 0;
    int group_index = 0;
    for (const auto& group_json : area_json["groups"]) {
        group_index++;
        const std::string group_type = group_json.value("group_type", std::string("unknown_group"));
        nlohmann::json execution_group_json = group_json;
        execution_group_json["points"] = filter_precomputed_group_points_for_execution(
            group_json,
            bind_execution_memory,
            blocked_global_indices,
            checkerboard_jump_bind_enabled
        );
        std::vector<tie_robot_msgs::PointCoords> local_points;
        std::string prepare_failure_reason;
        if (!load_precomputed_local_points_from_group_json(
                execution_group_json,
                local_points,
                prepare_failure_reason
            )) {
            skipped_group_count++;
            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组(%s)无法装载JSON局部点，跳过当前组。原因：%s\n",
                group_index,
                group_type.c_str(),
                prepare_failure_reason.c_str()
            );
            continue;
        }

        tie_robot_msgs::ExecuteBindPoints bind_srv;
        bind_srv.request.points = local_points;
        const auto dispatched_point_jsons = collect_dispatched_precomputed_point_jsons(
            execution_group_json,
            local_points
        );
        const std::unordered_set<int> active_dispatch_global_indices =
            collect_global_indices_from_group_json(dispatched_point_jsons);
        set_pseudo_slam_marker_execution_state(
            area_index,
            area_global_indices,
            active_dispatch_global_indices
        );
        if (!sg_precomputed_fast_client.call(bind_srv) || !bind_srv.response.success) {
            skipped_group_count++;
            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组失败，跳过当前组，消息：%s\n",
                group_index,
                bind_srv.response.message.c_str()
            );
            continue;
        }

        set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
        for (const auto& dispatched_point_json : dispatched_point_jsons) {
            record_successful_execution_point(bind_execution_memory, dispatched_point_json);
        }
        std::string bind_execution_memory_error;
        if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组成功后写入bind_execution_memory.json失败：%s\n",
                group_index,
                bind_execution_memory_error.c_str()
            );
            std::string invalidate_scan_artifacts_error;
            const bool scan_artifacts_invalidated =
                invalidate_current_scan_artifacts_after_execution_memory_write_failure(
                    bind_execution_memory_error,
                    &invalidate_scan_artifacts_error
                );
            if (!scan_artifacts_invalidated) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: 当前区域预计算直执行账本写盘失败后，失效化当前扫描产物session也失败：%s\n",
                    invalidate_scan_artifacts_error.c_str()
                );
                message =
                    "当前区域预计算直执行已实际执行成功，但bind_execution_memory.json写入失败，且当前扫描产物session失效化失败；"
                    "为避免重启后重复绑扎，请立即重新扫描/重新建图，并人工确认pseudo_slam_points.json/"
                    "pseudo_slam_bind_path.json不可继续使用";
            } else {
                message =
                    "当前区域预计算直执行已实际执行成功，但bind_execution_memory.json写入失败；"
                    "为避免重启后重复绑扎，已将当前扫描产物session失效化，需要重新扫描/重新建图后再执行";
            }
            return false;
        }
        executed_group_count++;
    }

    publish_area_progress(area_index < total_area_count ? area_index + 1 : area_index, total_area_count, area_index, true, false);

    if (executed_group_count <= 0) {
        std::ostringstream oss;
        oss << "当前区域预计算直执行未找到满足虎口执行范围的绑扎组，已跳过"
            << skipped_group_count << "组";
        message = oss.str();
        return false;
    }

    std::ostringstream oss;
    oss << "当前区域预计算直执行完成，区域" << area_index
        << "成功执行" << executed_group_count << "组，跳过" << skipped_group_count
        << "组";
    message = oss.str();
    return true;
}

bool run_bind_path_direct_test(std::string& message)
{
    clear_pseudo_slam_marker_execution_state();
    ScopedPseudoSlamMarkerExecutionStateClear scoped_marker_execution_state_clear;
    std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);

    nlohmann::json bind_path_json;
    if (!load_scan_artifact_json(
            pseudo_slam_bind_path_json_file,
            "pseudo_slam_bind_path.json",
            bind_path_json,
            message
        )) {
        return false;
    }
    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        message = "pseudo_slam_bind_path.json格式错误";
        return false;
    }

    const float cabin_speed = get_global_cabin_move_speed_mm_per_sec();
    const auto& areas_json = bind_path_json["areas"];
    if (areas_json.empty()) {
        message = "pseudo_slam_bind_path.json没有可执行区域，请先确认扫描分组成功";
        return false;
    }
    const int total_area_count = static_cast<int>(areas_json.size());

    float path_origin_x = 0.0f;
    float path_origin_y = 0.0f;
    float path_origin_z = bind_path_json.value("cabin_height", 500.0f);
    if (bind_path_json.contains("path_origin") && bind_path_json["path_origin"].is_object()) {
        path_origin_x = bind_path_json["path_origin"]["x"].get<float>();
        path_origin_y = bind_path_json["path_origin"]["y"].get<float>();
        path_origin_z = bind_path_json["path_origin"]["z"].get<float>();
    } else {
        const auto& first_cabin_pose = areas_json.front()["cabin_pose"];
        path_origin_x = first_cabin_pose.value("x", 0.0f);
        path_origin_y = first_cabin_pose.value("y", 0.0f);
        path_origin_z = first_cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
    }
    align_execution_path_origin_xy_to_first_area_if_needed(
        areas_json,
        path_origin_x,
        path_origin_y,
        "bind_path_direct_test"
    );

    const float move_path_origin_z = clamp_bind_execution_cabin_z(path_origin_z);
    TCP_Move[0] = cabin_speed;
    TCP_Move[1] = path_origin_x;
    TCP_Move[2] = path_origin_y;
    TCP_Move[3] = move_path_origin_z;
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: bind_path_direct_test先回到规划原点(%f,%f,%f)。\n",
        TCP_Move[1],
        TCP_Move[2],
        TCP_Move[3]
    );
    std::string path_origin_driver_error_message;
    if (!move_cabin_pose_via_driver(
            cabin_speed,
            path_origin_x,
            path_origin_y,
            move_path_origin_z,
            &path_origin_driver_error_message)) {
        message = compose_cabin_failure_message("bind_path_direct_test回到规划原点时下发索驱移动指令失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_X, path_origin_x)) {
        message = compose_cabin_failure_message("bind_path_direct_test回原点时索驱X轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Y, path_origin_y)) {
        message = compose_cabin_failure_message("bind_path_direct_test回原点时索驱Y轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_path_origin_z)) {
        message = compose_cabin_failure_message("bind_path_direct_test回原点时索驱Z轴到位失败");
        return false;
    }

    int executed_group_count = 0;
    int skipped_group_count = 0;
    int area_index = 0;
    for (const auto& area_json : areas_json) {
        area_index++;
        publish_area_progress(area_index, total_area_count, 0, false, false);
        const auto cabin_pose = area_json["cabin_pose"];
        const float cabin_x = cabin_pose.value("x", 0.0f);
        const float cabin_y = cabin_pose.value("y", 0.0f);
        const float cabin_z = cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
        const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_z);
        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_x;
        TCP_Move[2] = cabin_y;
        TCP_Move[3] = move_cabin_z;
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: bind_path_direct_test区域%d移动到(%f,%f,%f)。\n",
            area_index,
            TCP_Move[1],
            TCP_Move[2],
            TCP_Move[3]
        );
        std::string area_driver_error_message;
        if (!move_cabin_pose_via_driver(
                cabin_speed,
                cabin_x,
                cabin_y,
                move_cabin_z,
                &area_driver_error_message)) {
            message = compose_cabin_failure_message(
                "bind_path_direct_test区域" + std::to_string(area_index) + "下发索驱移动指令失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_X, cabin_x)) {
            message = compose_cabin_failure_message(
                "bind_path_direct_test区域" + std::to_string(area_index) + "索驱X轴到位失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_Y, cabin_y)) {
            message = compose_cabin_failure_message(
                "bind_path_direct_test区域" + std::to_string(area_index) + "索驱Y轴到位失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_cabin_z)) {
            message = compose_cabin_failure_message(
                "bind_path_direct_test区域" + std::to_string(area_index) + "索驱Z轴到位失败"
            );
            return false;
        }

        if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
            printCurrentTime();
            ros_log_printf("Cabin_Warn: bind_path_direct_test区域%d缺少groups，跳过当前区域。\n", area_index);
            continue;
        }

        const std::unordered_set<int> area_global_indices = collect_global_indices_from_area_json(area_json);
        set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});

        int group_index = 0;
        for (const auto& group_json : area_json["groups"]) {
            group_index++;
            const std::string group_type = group_json.value("group_type", std::string("unknown_group"));
            std::vector<tie_robot_msgs::PointCoords> local_points;
            std::string prepare_failure_reason;
            if (!load_precomputed_local_points_from_group_json(
                    group_json,
                    local_points,
                    prepare_failure_reason
                )) {
                skipped_group_count++;
                set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: bind_path_direct_test区域%d第%d组(%s)无法装载JSON局部点，跳过当前组。原因：%s\n",
                    area_index,
                    group_index,
                    group_type.c_str(),
                    prepare_failure_reason.c_str()
                );
                continue;
            }

            tie_robot_msgs::ExecuteBindPoints bind_srv;
            bind_srv.request.points = local_points;
            const auto dispatched_point_jsons = collect_dispatched_precomputed_point_jsons(
                group_json,
                local_points
            );
            const std::unordered_set<int> active_dispatch_global_indices =
                collect_global_indices_from_group_json(dispatched_point_jsons);
            set_pseudo_slam_marker_execution_state(
                area_index,
                area_global_indices,
                active_dispatch_global_indices
            );
            if (!sg_precomputed_client.call(bind_srv) || !bind_srv.response.success) {
                skipped_group_count++;
                set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: bind_path_direct_test区域%d第%d组执行失败，跳过当前组。消息：%s\n",
                    area_index,
                    group_index,
                    bind_srv.response.message.c_str()
                );
                continue;
            }

            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            executed_group_count++;
        }

        publish_area_progress(
            area_index < total_area_count ? area_index + 1 : area_index,
            total_area_count,
            area_index,
            true,
            false
        );
    }

    publish_area_progress(total_area_count, total_area_count, total_area_count, false, true);
    if (executed_group_count <= 0) {
        std::ostringstream oss;
        oss << "直接执行账本测试未成功执行任何组，跳过" << skipped_group_count << "组";
        message = oss.str();
        return false;
    }

    std::ostringstream oss;
    oss << "索驱已按pseudo_slam_bind_path.json直接测试执行完成，成功执行"
        << executed_group_count << "组，跳过" << skipped_group_count << "组";
    message = oss.str();
    return true;
}

bool run_live_visual_global_work(std::string& message)
{
    clear_pseudo_slam_marker_execution_state();
    ScopedPseudoSlamMarkerExecutionStateClear scoped_marker_execution_state_clear;
    std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
    BindExecutionMemory bind_execution_memory;
    std::string current_path_signature;
    std::string bind_execution_memory_error;
    if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error)) {
        message = bind_execution_memory_error;
        return false;
    }
    if (!load_current_path_signature_for_execution(current_path_signature, message)) {
        return false;
    }
    nlohmann::json points_json;
    nlohmann::json bind_path_json;
    if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message)) {
        return false;
    }
    const std::unordered_set<int> blocked_global_indices =
        collect_blocked_execution_global_indices_from_points_json(points_json);
    LiveVisualCheckerboardGrid checkerboard_grid;
    std::string checkerboard_grid_error;
    if (!load_live_visual_checkerboard_grid(points_json, checkerboard_grid, checkerboard_grid_error)) {
        message = checkerboard_grid_error;
        return false;
    }

    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        message = "pseudo_slam_bind_path.json格式错误";
        return false;
    }
    const auto& areas_json = bind_path_json["areas"];
    const int total_area_count = static_cast<int>(areas_json.size());
    int executed_area_count = 0;
    int skipped_area_count = 0;

    float path_origin_x = 0.0f;
    float path_origin_y = 0.0f;
    float path_origin_z = bind_path_json.value("cabin_height", 500.0f);
    if (bind_path_json.contains("path_origin") && bind_path_json["path_origin"].is_object()) {
        path_origin_x = bind_path_json["path_origin"]["x"].get<float>();
        path_origin_y = bind_path_json["path_origin"]["y"].get<float>();
        path_origin_z = bind_path_json["path_origin"]["z"].get<float>();
    } else {
        const auto& first_cabin_pose = areas_json.front()["cabin_pose"];
        path_origin_x = first_cabin_pose.value("x", 0.0f);
        path_origin_y = first_cabin_pose.value("y", 0.0f);
        path_origin_z = first_cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
    }
    align_execution_path_origin_xy_to_first_area_if_needed(
        areas_json,
        path_origin_x,
        path_origin_y,
        "live_visual"
    );

    const float move_path_origin_z = clamp_bind_execution_cabin_z(path_origin_z);
    const float cabin_speed = get_global_cabin_move_speed_mm_per_sec();
    TCP_Move[0] = cabin_speed;
    TCP_Move[1] = path_origin_x;
    TCP_Move[2] = path_origin_y;
    TCP_Move[3] = move_path_origin_z;
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: live_visual先回到规划原点(%f,%f,%f)。\n",
        TCP_Move[1],
        TCP_Move[2],
        TCP_Move[3]
    );
    std::string live_visual_origin_driver_error_message;
    if (!move_cabin_pose_via_driver(
            cabin_speed,
            path_origin_x,
            path_origin_y,
            move_path_origin_z,
            &live_visual_origin_driver_error_message)) {
        message = compose_cabin_failure_message("live_visual回到规划原点时下发索驱移动指令失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_X, path_origin_x)) {
        message = compose_cabin_failure_message("live_visual回原点时索驱X轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Y, path_origin_y)) {
        message = compose_cabin_failure_message("live_visual回原点时索驱Y轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_path_origin_z)) {
        message = compose_cabin_failure_message("live_visual回原点时索驱Z轴到位失败");
        return false;
    }

    for (int area_order_index = 0; area_order_index < total_area_count; ++area_order_index) {
        const auto& planned_area_json = areas_json[static_cast<size_t>(area_order_index)];
        const int area_index = planned_area_json.value("area_index", area_order_index + 1);
        publish_area_progress(area_order_index + 1, total_area_count, 0, false, false);

        const std::unordered_map<int, tie_robot_msgs::PointCoords> planned_area_points_by_global_index =
            collect_planned_area_world_points_by_global_index(planned_area_json);
        const std::unordered_set<int> area_global_indices =
            collect_global_indices_from_area_json(planned_area_json);
        if (planned_area_points_by_global_index.empty() || area_global_indices.empty()) {
            skipped_area_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: live_visual区域%d扫描账本缺少有效参考点，跳过当前区域。\n",
                area_index
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        const auto& cabin_pose = planned_area_json["cabin_pose"];
        const float cabin_x = cabin_pose.value("x", 0.0f);
        const float cabin_y = cabin_pose.value("y", 0.0f);
        const float cabin_z = cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
        const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_z);
        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_x;
        TCP_Move[2] = cabin_y;
        TCP_Move[3] = move_cabin_z;
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: live_visual区域%d移动到(%f,%f,%f)。\n",
            area_index,
            TCP_Move[1],
            TCP_Move[2],
            TCP_Move[3]
        );
        std::string area_driver_error_message;
        if (!move_cabin_pose_via_driver(
                cabin_speed,
                cabin_x,
                cabin_y,
                move_cabin_z,
                &area_driver_error_message)) {
            skipped_area_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d下发索驱移动指令失败，跳过当前区域。\n",
                area_index
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_X, cabin_x) ||
            !wait_cabin_axis_stable_arrival(AXIS_Y, cabin_y) ||
            !wait_cabin_axis_stable_arrival(AXIS_Z, move_cabin_z)) {
            skipped_area_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d索驱到位失败，跳过当前区域。\n",
                area_index
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        tie_robot_msgs::ProcessImage scan_srv;
        scan_srv.request.request_mode = kProcessImageModeExecutionRefine;
        if (!AI_client.call(scan_srv)) {
            skipped_area_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d调用/pointAI/process_image失败，跳过当前区域。\n",
                area_index
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        if (!scan_srv.response.success) {
            skipped_area_count++;
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d视觉失败，跳过当前区域。消息：%s\n",
                area_index,
                scan_srv.response.message.c_str()
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        std::vector<tie_robot_msgs::PointCoords> area_world_points;
        int point_index = 1;
        for (const auto& point : scan_srv.response.PointCoordinatesArray) {
            tie_robot_msgs::PointCoords world_point;
            if (build_world_point_from_scan_response(point, point_index, world_point)) {
                area_world_points.push_back(world_point);
                point_index++;
            }
        }
        area_world_points = dedupe_world_points(area_world_points);

        nlohmann::json classified_group_json;
        classified_group_json["points"] = nlohmann::json::array();
        for (const auto& world_point : area_world_points) {
            nlohmann::json classified_point_json;
            if (!classify_live_visual_point_into_checkerboard(
                    world_point,
                    checkerboard_grid,
                    classified_point_json
                )) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_log: live_visual区域%d点idx=%d未能归入全局棋盘格，跳过该点。\n",
                    area_index,
                    world_point.idx
                );
                continue;
            }
            classified_group_json["points"].push_back(classified_point_json);
        }
        set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
        nlohmann::json execution_group_json;
        execution_group_json["points"] = build_live_visual_execution_points_from_planned_area(
            area_index,
            planned_area_json,
            classified_group_json["points"],
            planned_area_points_by_global_index
        );
        execution_group_json["points"] = filter_precomputed_group_points_for_execution(
            execution_group_json,
            bind_execution_memory,
            blocked_global_indices,
            checkerboard_jump_bind_enabled
        );

        std::vector<tie_robot_msgs::PointCoords> local_points;
        std::string prepare_failure_reason;
        if (!load_precomputed_local_points_from_group_json(
                execution_group_json,
                local_points,
                prepare_failure_reason
            )) {
            skipped_area_count++;
            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d无法装载JSON局部点，跳过当前区域。原因：%s\n",
                area_index,
                prepare_failure_reason.c_str()
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        tie_robot_msgs::ExecuteBindPoints bind_srv;
        bind_srv.request.points = local_points;
        const auto dispatched_point_jsons = collect_dispatched_precomputed_point_jsons(
            execution_group_json,
            local_points
        );
        const std::unordered_set<int> active_dispatch_global_indices =
            collect_global_indices_from_group_json(dispatched_point_jsons);
        set_pseudo_slam_marker_execution_state(
            area_index,
            area_global_indices,
            active_dispatch_global_indices
        );
        if (!sg_precomputed_fast_client.call(bind_srv) || !bind_srv.response.success) {
            skipped_area_count++;
            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d执行失败，跳过当前区域。消息：%s\n",
                area_index,
                bind_srv.response.message.c_str()
            );
            publish_area_progress(
                area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
                total_area_count,
                area_order_index + 1,
                true,
                false
            );
            continue;
        }

        set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
        for (const auto& dispatched_point_json : dispatched_point_jsons) {
            record_successful_execution_point(bind_execution_memory, dispatched_point_json, "live_visual");
        }
        std::string bind_execution_memory_error;
        if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_Warn: live_visual区域%d成功后写入bind_execution_memory.json失败：%s\n",
                area_index,
                bind_execution_memory_error.c_str()
            );
            std::string invalidate_scan_artifacts_error;
            const bool scan_artifacts_invalidated =
                invalidate_current_scan_artifacts_after_execution_memory_write_failure(
                    bind_execution_memory_error,
                    &invalidate_scan_artifacts_error
                );
            if (!scan_artifacts_invalidated) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: live_visual账本写盘失败后，失效化当前扫描产物session也失败：%s\n",
                    invalidate_scan_artifacts_error.c_str()
                );
                message =
                    "live_visual已实际执行成功，但bind_execution_memory.json写入失败，且当前扫描产物session失效化失败；"
                    "为避免重启后重复绑扎，请立即重新扫描/重新建图，并人工确认pseudo_slam_points.json/"
                    "pseudo_slam_bind_path.json不可继续使用";
            } else {
                message =
                    "live_visual已实际执行成功，但bind_execution_memory.json写入失败；"
                    "为避免重启后重复绑扎，已将当前扫描产物session失效化，需要重新扫描/重新建图后再执行";
            }
            return false;
        }

        executed_area_count++;
        publish_area_progress(
            area_order_index + 1 < total_area_count ? area_order_index + 2 : area_order_index + 1,
            total_area_count,
            area_order_index + 1,
            true,
            false
        );
    }

    publish_area_progress(total_area_count, total_area_count, total_area_count, false, true);

    std::ostringstream oss;
    if (executed_area_count <= 0) {
        oss << "live_visual模式下未成功执行任何区域，跳过" << skipped_area_count << "个区域";
        message = oss.str();
        return false;
    }

    oss << "live_visual全局执行完成，成功执行" << executed_area_count
        << "个区域，跳过" << skipped_area_count << "个区域";
    message = oss.str();
    return true;
}

bool run_bind_from_scan(std::string& message)
{
    clear_pseudo_slam_marker_execution_state();
    ScopedPseudoSlamMarkerExecutionStateClear scoped_marker_execution_state_clear;
    std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
    nlohmann::json points_json;
    nlohmann::json bind_path_json;
    BindExecutionMemory bind_execution_memory;
    std::string current_path_signature;
    std::string bind_execution_memory_error;
    if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error)) {
        message = bind_execution_memory_error;
        return false;
    }
    if (!load_current_path_signature_for_execution(current_path_signature, message)) {
        return false;
    }
    if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message)) {
        return false;
    }
    const std::unordered_set<int> blocked_global_indices =
        collect_blocked_execution_global_indices_from_points_json(points_json);
    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        message = "pseudo_slam_bind_path.json格式错误";
        return false;
    }

    const float cabin_speed = get_global_cabin_move_speed_mm_per_sec();
    const auto& areas_json = bind_path_json["areas"];
    if (areas_json.empty()) {
        message = "pseudo_slam_bind_path.json没有可执行区域，请先确认扫描分组成功";
        return false;
    }
    const int total_area_count = static_cast<int>(areas_json.size());

    float path_origin_x = 0.0f;
    float path_origin_y = 0.0f;
    float path_origin_z = bind_path_json.value("cabin_height", 500.0f);
    if (bind_path_json.contains("path_origin") && bind_path_json["path_origin"].is_object()) {
        path_origin_x = bind_path_json["path_origin"]["x"].get<float>();
        path_origin_y = bind_path_json["path_origin"]["y"].get<float>();
        path_origin_z = bind_path_json["path_origin"]["z"].get<float>();
    } else {
        const auto& first_cabin_pose = areas_json.front()["cabin_pose"];
        path_origin_x = first_cabin_pose.value("x", 0.0f);
        path_origin_y = first_cabin_pose.value("y", 0.0f);
        path_origin_z = first_cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
    }
    align_execution_path_origin_xy_to_first_area_if_needed(
        areas_json,
        path_origin_x,
        path_origin_y,
        "bind_from_scan"
    );

    const float move_path_origin_z = clamp_bind_execution_cabin_z(path_origin_z);
    TCP_Move[0] = cabin_speed;
    TCP_Move[1] = path_origin_x;
    TCP_Move[2] = path_origin_y;
    TCP_Move[3] = move_path_origin_z;
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: bind_from_scan先回到规划原点(%f,%f,%f)。\n",
        TCP_Move[1],
        TCP_Move[2],
        TCP_Move[3]
    );
    std::string bind_from_scan_origin_driver_error_message;
    if (!move_cabin_pose_via_driver(
            cabin_speed,
            path_origin_x,
            path_origin_y,
            move_path_origin_z,
            &bind_from_scan_origin_driver_error_message)) {
        message = compose_cabin_failure_message("bind_from_scan回到规划原点时下发索驱移动指令失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_X, path_origin_x)) {
        message = compose_cabin_failure_message("bind_from_scan回原点时索驱X轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Y, path_origin_y)) {
        message = compose_cabin_failure_message("bind_from_scan回原点时索驱Y轴到位失败");
        return false;
    }
    if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_path_origin_z)) {
        message = compose_cabin_failure_message("bind_from_scan回原点时索驱Z轴到位失败");
        return false;
    }

    int area_index = 0;
    for (const auto& area_json : areas_json) {
        area_index++;
        publish_area_progress(area_index, total_area_count, 0, false, false);
        const auto cabin_pose = area_json["cabin_pose"];
        const float cabin_x = cabin_pose.value("x", 0.0f);
        const float cabin_y = cabin_pose.value("y", 0.0f);
        const float cabin_z = cabin_pose.value("z", bind_path_json.value("cabin_height", 500.0f));
        const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_z);
        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_x;
        TCP_Move[2] = cabin_y;
        TCP_Move[3] = move_cabin_z;
        printCurrentTime();
        ros_log_printf("Cabin_log: bind_from_scan区域%d移动到(%f,%f,%f)。\n", area_index, TCP_Move[1], TCP_Move[2], TCP_Move[3]);
        std::string area_driver_error_message;
        if (!move_cabin_pose_via_driver(
                cabin_speed,
                cabin_x,
                cabin_y,
                move_cabin_z,
                &area_driver_error_message)) {
            message = compose_cabin_failure_message(
                "bind_from_scan区域" + std::to_string(area_index) + "下发索驱移动指令失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_X, cabin_x)) {
            message = compose_cabin_failure_message(
                "bind_from_scan区域" + std::to_string(area_index) + "索驱X轴到位失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_Y, cabin_y)) {
            message = compose_cabin_failure_message(
                "bind_from_scan区域" + std::to_string(area_index) + "索驱Y轴到位失败"
            );
            return false;
        }
        if (!wait_cabin_axis_stable_arrival(AXIS_Z, move_cabin_z)) {
            message = compose_cabin_failure_message(
                "bind_from_scan区域" + std::to_string(area_index) + "索驱Z轴到位失败"
            );
            return false;
        }

        if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
            printCurrentTime();
            ros_log_printf("Cabin_Warn: bind_from_scan区域%d缺少groups，跳过当前区域。\n", area_index);
            continue;
        }

        const std::unordered_set<int> area_global_indices = collect_global_indices_from_area_json(area_json);
        set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});

        int group_index = 0;
        for (const auto& group_json : area_json["groups"]) {
            group_index++;
            const std::string group_type = group_json.value("group_type", std::string("unknown_group"));
            nlohmann::json execution_group_json = group_json;
            execution_group_json["points"] = filter_precomputed_group_points_for_execution(
                group_json,
                bind_execution_memory,
                blocked_global_indices,
                checkerboard_jump_bind_enabled
            );
            std::vector<tie_robot_msgs::PointCoords> local_points;
            std::string prepare_failure_reason;
            if (!load_precomputed_local_points_from_group_json(
                    execution_group_json,
                    local_points,
                    prepare_failure_reason
                )) {
                set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组(%s)无法装载JSON局部点，跳过当前组。原因：%s\n",
                    area_index,
                    group_index,
                    group_type.c_str(),
                    prepare_failure_reason.c_str()
                );
                continue;
            }
            tie_robot_msgs::ExecuteBindPoints bind_srv;
            bind_srv.request.points = local_points;
            const auto dispatched_point_jsons = collect_dispatched_precomputed_point_jsons(
                execution_group_json,
                local_points
            );
            const std::unordered_set<int> active_dispatch_global_indices =
                collect_global_indices_from_group_json(dispatched_point_jsons);
            set_pseudo_slam_marker_execution_state(
                area_index,
                area_global_indices,
                active_dispatch_global_indices
            );
            if (!sg_precomputed_client.call(bind_srv) || !bind_srv.response.success) {
                set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组执行失败，跳过该点所在组，跳过该组继续下一组，消息：%s\n",
                    area_index,
                    group_index,
                    bind_srv.response.message.c_str()
                );
                continue;
            }

            set_pseudo_slam_marker_execution_state(area_index, area_global_indices, {});
            for (const auto& dispatched_point_json : dispatched_point_jsons) {
                record_successful_execution_point(bind_execution_memory, dispatched_point_json);
            }
            std::string bind_execution_memory_error;
            if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
                printCurrentTime();
                ros_log_printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组成功后写入bind_execution_memory.json失败：%s\n",
                    area_index,
                    group_index,
                    bind_execution_memory_error.c_str()
                );
                std::string invalidate_scan_artifacts_error;
                const bool scan_artifacts_invalidated =
                    invalidate_current_scan_artifacts_after_execution_memory_write_failure(
                        bind_execution_memory_error,
                        &invalidate_scan_artifacts_error
                    );
                if (!scan_artifacts_invalidated) {
                    printCurrentTime();
                    ros_log_printf(
                        "Cabin_Warn: bind_from_scan账本写盘失败后，失效化当前扫描产物session也失败：%s\n",
                        invalidate_scan_artifacts_error.c_str()
                    );
                    message =
                        "bind_from_scan已实际执行成功，但bind_execution_memory.json写入失败，且当前扫描产物session失效化失败；"
                        "为避免重启后重复绑扎，请立即重新扫描/重新建图，并人工确认pseudo_slam_points.json/"
                        "pseudo_slam_bind_path.json不可继续使用";
                } else {
                    message =
                        "bind_from_scan已实际执行成功，但bind_execution_memory.json写入失败；"
                        "为避免重启后重复绑扎，已将当前扫描产物session失效化，需要重新扫描/重新建图后再执行";
                }
                return false;
            }
        }
        publish_area_progress(area_index < total_area_count ? area_index + 1 : area_index, total_area_count, area_index, true, false);
    }

    publish_area_progress(total_area_count, total_area_count, total_area_count, false, true);
    message = "索驱已按pseudo_slam_bind_path.json执行完成";
    return true;
}

// service 编排已抽到 src/suoqu/service_orchestration.cpp。

/*
    读取索驱状态
*/
void read_cabin_state(Cabin_State *cab_state) {    
    float x_gesture,y_gesture;
    uint16_t check_sum=0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); //防止固定锁
        if (!cabin_driver_enabled.load()) {
            if (g_cabin_driver) {
                g_cabin_driver->stop();
            }
            sync_global_socket_fd_from_cabin_driver();
            cabin_driver_last_state_stamp_sec.store(0.0);
            continue;
        }
        {
            std::lock_guard<std::mutex> lock2(socket_mutex);
            check_sum=0;
            x_gesture=cab_state->cabin_x_gesture;
            FtoU(&x_gesture);
            TCP_Normal_Connection[4]=FtoU_register[3];
            TCP_Normal_Connection[5]=FtoU_register[2];
            TCP_Normal_Connection[6]=FtoU_register[1];
            TCP_Normal_Connection[7]=FtoU_register[0];
            y_gesture=cab_state->cabin_y_gesture;
            FtoU(&y_gesture);
            TCP_Normal_Connection[8]=FtoU_register[3];
            TCP_Normal_Connection[9]=FtoU_register[2];
            TCP_Normal_Connection[10]=FtoU_register[1];
            TCP_Normal_Connection[11]=FtoU_register[0];
            for(int i=0;i<12;i++)
            {
                check_sum+=TCP_Normal_Connection[i];
            }
            TCP_Normal_Connection[12]=check_sum;
            TCP_Normal_Connection[13]=check_sum>>8;
            Frame_Generate_With_Retry(TCP_Normal_Connection, 14, sizeof(cabin_state_buffer));
        }

        // 校验和计算和验证
        uint16_t sum = 0;
        for (int i = 0; i < 142; ++i) {
            sum += cabin_state_buffer[i];
        }
        //uint16_t calculatedChecksum = static_cast<uint16_t>(sum & 0xFFFF); // 保留最低16位
        uint16_t receivedChecksum = cabin_state_buffer[142]| (cabin_state_buffer[143]<<8);
        if (receivedChecksum != sum) {
            // 校验和不匹配，跳过本次循环
            printCurrentTime();
            ros_log_printf("Cabin_log: 校验和错误，当前TCP数据包被丢弃。\n");
            continue;
        }
 
        // 保护 cabin_state 读写操作，避免数据竞争
        {
            std::lock_guard<std::mutex> lock1(cabin_state_mutex);
            memcpy(&cab_state->X, &cabin_state_buffer[2], sizeof(float));
            memcpy(&cab_state->Y, &cabin_state_buffer[6], sizeof(float));
            memcpy(&cab_state->Z, &cabin_state_buffer[10], sizeof(float));
            memcpy(&cab_state->A, &cabin_state_buffer[14], sizeof(float));
            memcpy(&cab_state->B, &cabin_state_buffer[18], sizeof(float));
            memcpy(&cab_state->C, &cabin_state_buffer[22], sizeof(float));
            cab_state->motion_status = int((cabin_state_buffer[138] >> 3) & 0x01);
            // 获取设备报警和内部计算错误状态
            cab_state->device_alarm = int(cabin_state_buffer[138] & (1 << 4));         // bit4
            cab_state->internal_calc_error = int(cabin_state_buffer[138] & (1 << 5));  // bit5
            // cabin_state.X, cabin_state.Y,cabin_state.Z 读取到的0可能是一个很小的数值,float的存储精度问题
            if(std::abs(cab_state->X) < 1e-6)
                cab_state->X = 0;
            if(std::abs(cab_state->Y) < 1e-6)
                cab_state->Y = 0;
            if(std::abs(cab_state->Z) < 1e-6)
                cab_state->Z = 0;
        }
        // printCurrentTime();
        // ros_log_printf("Cabin_log: 现在索驱的位置坐标为(%f,%f,%f),运动状态为", cab_state->X,cab_state->Y,cab_state->Z);
        // ros_log_printf("%s\n", cab_state->motion_status ? " 运动。" : " 停止。");
        
        // 整合数据part1以上传
        cabin_data_upload.cabin_state_X = cab_state->X;
        cabin_data_upload.cabin_state_Y = cab_state->Y;
        cabin_data_upload.cabin_state_Z = cab_state->Z;
        cabin_data_upload.motion_status = cab_state->motion_status;
        cabin_data_upload.device_alarm = cab_state->device_alarm;
        cabin_data_upload.internal_calc_error = cab_state->internal_calc_error;
        cabin_data_upload.cabin_connect_flag = 1;
        pub_cabin_data_upload.publish(cabin_data_upload);
        cabin_driver_last_state_stamp_sec.store(ros::Time::now().toSec());
        maybe_refresh_pseudo_slam_marker_outlier_threshold();

        // if((abs(x_gesture)>=1.0||abs(y_gesture)>=1.0)&&cab_state->motion_status==0)
        // {
        //     float e_x[3]={0,cos(degreesToRadians(x_gesture)),sin(degreesToRadians(x_gesture))};
        //     float e_y[3]={cos(degreesToRadians(y_gesture)),0,sin(degreesToRadians(y_gesture))};
        //     float n_xy[3]={0};

        //     n_xy[0]=e_y[1]*e_x[2]-e_y[2]*e_x[1];
        //     n_xy[1]=e_y[2]*e_x[0]-e_y[0]*e_x[2];
        //     n_xy[2]=e_y[0]*e_x[1]-e_y[1]*e_x[1];
        
        //     float ang_y=acos(n_xy[2]);
        //     float ang_z1;
        
        //     if(n_xy[1]>0)
        //     {
        //         ang_z1=acos(n_xy[0]/(sqrt(n_xy[0]*n_xy[0]+n_xy[1]*n_xy[1])));
        //     }
        //     else 
        //     {
        //         ang_z1=-acos(n_xy[0]/(sqrt(n_xy[0]*n_xy[0]+n_xy[1]*n_xy[1])));
        //     }

        //     TCP_Move[5]=ang_z1*180/M_PI;
        //     //std::cout<<"A的数字为："<<TCP_Move[5]<<endl;
        //     TCP_Move[6]=-ang_y*180/M_PI;
        //     TCP_Move[7]=-ang_z1*180/M_PI;

        //     {
        //         std::lock_guard<std::mutex> lock2(socket_mutex);
        //         moveTCPPosition(0x01,TCP_Move);
        //         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
        //     }
        // }


        // 如果索驱状态报警或内部计算错误，直接退出程序
        if(cab_state->device_alarm == 1)
        {
               printCurrentTime();
               ros_log_printf("Cabin_error: 索驱设备报警，正在紧急关闭程序。\n");
               persist_last_cabin_fatal_error_detail("索驱设备报警，节点已执行紧急退出");
               
            try
            {
                std::string driver_error_message;
                stop_cabin_motion_via_driver(&driver_error_message);
                if (g_cabin_driver) {
                    g_cabin_driver->stop();
                }
                sync_global_socket_fd_from_cabin_driver();
                emergency_exit_with_flush(3); 
            }
            catch(const std::exception& e)
            {
                // 直接强制退出程序
                emergency_exit_with_flush(3);
            }
        }
        if(cab_state->internal_calc_error == 1)
        {
               printCurrentTime();
               ros_log_printf("Cabin_error: 索驱内部计算错误，正在紧急关闭程序。\n");
               persist_last_cabin_fatal_error_detail("索驱内部计算错误，节点已执行紧急退出");
               
            try
            {
                std::string driver_error_message;
                stop_cabin_motion_via_driver(&driver_error_message);
                if (g_cabin_driver) {
                    g_cabin_driver->stop();
                }
                sync_global_socket_fd_from_cabin_driver();
                emergency_exit_with_flush(3); 
            }
            catch(const std::exception& e)
            {
                // 直接强制退出程序
                emergency_exit_with_flush(3);
            }
        }
    }
}

bool suoquInit()
{
    g_cabin_driver = std::make_unique<tie_robot_hw::driver::CabinDriver>();
    //创建TCP连接
    if (!connectToServer()) {
        printCurrentTime();
        ros_log_printf("Cabin_Error: 索驱进程创建TCP连接失败。\n");
        return false;
    }
    // 索驱使能和逆解激活
    // Frame_Generate_With_Retry(motor_enable, 8, 6);
    // usleep(500000);
    // Frame_Generate_With_Retry(motor_inverse_enable, 8, 6);
    return true;
}

int RunSuoquNodeWithDefaultRole(int argc, char** argv, const std::string& default_role)
{
    (void)argc;
    (void)argv;
    setlocale(LC_ALL, "");
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    suoqu_node_role = default_role;
    private_nh.param<std::string>("node_role", suoqu_node_role, suoqu_node_role);
    use_remote_cabin_driver.store(!is_suoqu_driver_role(), std::memory_order_relaxed);
    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);
    printCurrentTime();
    ros_log_printf(
        "<--- %s Started, role=%s --->\n",
        ros::this_node::getName().c_str(),
        suoqu_node_role.c_str()
    );
    ros_log_printf(
        "Cabin_log: TF链路由robot_tf_broadcaster守护；本节点只发布/cabin/cabin_data_upload索驱原始状态。\n"
    );
    clear_last_cabin_fatal_error_detail();
    
    if (!is_suoqu_driver_role() &&
        !is_suoqu_cabin_motion_controller_role() &&
        !is_suoqu_bind_task_executor_role())
    {
        printCurrentTime();
        ros_log_printf("Cabin_Error: 未知索驱节点角色 node_role=%s，节点退出。\n", suoqu_node_role.c_str());
        return 1;
    }

    // 注册信号处理函数，捕获 SIGINT (Ctrl + C)
    signal(SIGINT, signalHandler_cc);

    ros::ServiceServer path_config_service;
    ros::ServiceServer set_execution_mode_service;
    ros::ServiceServer update_service;
    ros::ServiceServer update_service_with_options;
    ros::ServiceServer pseudo_slam_scan_service;
    ros::ServiceServer pseudo_slam_scan_with_options_service;
    ros::ServiceServer current_area_bind_from_scan_service;
    ros::ServiceServer bind_path_direct_test_service_server;
    ros::ServiceServer cabin_driver_start_service;
    ros::ServiceServer cabin_driver_stop_service;
    ros::ServiceServer cabin_driver_restart_service;
    ros::ServiceServer cabin_motion_stop_service;
    ros::ServiceServer cabin_driver_raw_move_service_server;
    ros::ServiceServer cabin_single_move_service;

    ros::Subscriber change_cabin_speed_sub;
    ros::Subscriber pseudo_slam_ir_image_sub;
    ros::Subscriber forced_stop_sub;
    ros::Subscriber interrupt0_sub;
    ros::Subscriber hand_solve_stop;
    ros::Subscriber moduan_work_sub;
    ros::Subscriber send_odd_points_sub;
    ros::Subscriber cabin_state_sub;

    ros::Timer cabin_diagnostic_timer;

    if (is_suoqu_driver_role()) {
        if (!suoquInit())
        {
            printCurrentTime();
            ros_log_printf("Cabin_Error: 索驱驱动节点初始化失败，节点退出。\n");
            return 1;
        }

        cabin_driver_start_service =
            nh.advertiseService("/cabin/driver/start", cabinDriverStartService);
        cabin_driver_stop_service =
            nh.advertiseService("/cabin/driver/stop", cabinDriverStopService);
        cabin_driver_restart_service =
            nh.advertiseService("/cabin/driver/restart", cabinDriverRestartService);
        cabin_motion_stop_service =
            nh.advertiseService("/cabin/motion/stop", cabinMotionStopService);
        cabin_driver_raw_move_service_server =
            nh.advertiseService("/cabin/driver/raw_move", cabin_driver_raw_move_service);
        pub_cabin_data_upload =
            nh.advertise<tie_robot_msgs::cabin_upload>("/cabin/cabin_data_upload", 5);

        g_cabin_diagnostic_updater = std::make_unique<diagnostic_updater::Updater>(nh);
        g_cabin_diagnostic_updater->setHardwareID(kCabinDiagnosticHardwareId);
        g_cabin_diagnostic_updater->add("索驱驱动", produce_cabin_driver_diagnostics);
        cabin_diagnostic_timer =
            nh.createTimer(ros::Duration(1.0), cabin_diagnostic_timer_callback);

        std::thread read_cabin_state_thread(read_cabin_state, &cabin_state);
        read_cabin_state_thread.detach();
        forced_stop_sub = nh.subscribe("/web/moduan/forced_stop", 5, &forced_stop_nodeCallback);
    }

    if (!is_suoqu_driver_role()) {
        cabin_state_sub = nh.subscribe("/cabin/cabin_data_upload", 20, &cabin_data_upload_callback);
    }

    if (is_suoqu_cabin_motion_controller_role()) {
        path_config_service = nh.advertiseService("/cabin/plan_path", planGlobalMovePath);
        cabin_single_move_service = nh.advertiseService("/cabin/single_move", cabin_single_move);
        change_cabin_speed_sub = nh.subscribe("/web/cabin/set_cabin_speed", 5, &change_cabin_speed_callback);
    }

    if (is_suoqu_bind_task_executor_role()) {
        set_execution_mode_service =
            nh.advertiseService("/cabin/set_execution_mode", setExecutionModeService);
        update_service = nh.advertiseService("/cabin/start_work", startGlobalWork);
        update_service_with_options =
            nh.advertiseService("/cabin/start_work_with_options", startGlobalWorkWithOptions);
        pseudo_slam_scan_service =
            nh.advertiseService("/cabin/start_pseudo_slam_scan", startPseudoSlamScan);
        pseudo_slam_scan_with_options_service =
            nh.advertiseService("/cabin/start_pseudo_slam_scan_with_options", startPseudoSlamScanWithOptions);
        current_area_bind_from_scan_service =
            nh.advertiseService("/cabin/bind_current_area_from_scan", bind_current_area_from_scan_service);
        bind_path_direct_test_service_server =
            nh.advertiseService("/cabin/run_bind_path_direct_test", bind_path_direct_test_service);
        pub_area_progress = nh.advertise<tie_robot_msgs::AreaProgress>("/cabin/area_progress", 5);
        pub_pseudo_slam_markers =
            nh.advertise<visualization_msgs::MarkerArray>("/cabin/pseudo_slam_markers", 1, true);
        restore_pseudo_slam_markers_from_json_on_startup();
        pseudo_slam_ir_image_sub =
            nh.subscribe(kPseudoSlamCaptureGateImageTopic, 1, &pseudo_slam_ir_image_callback);
        AI_client = nh.serviceClient<tie_robot_msgs::ProcessImage>("/pointAI/process_image");
        sg_live_visual_client = nh.serviceClient<std_srvs::Trigger>("/moduan/sg");
        sg_precomputed_client = nh.serviceClient<tie_robot_msgs::ExecuteBindPoints>("/moduan/sg_precomputed");
        sg_precomputed_fast_client = nh.serviceClient<tie_robot_msgs::ExecuteBindPoints>("/moduan/sg_precomputed_fast");
        change_cabin_speed_sub = nh.subscribe("/web/cabin/set_cabin_speed", 5, &change_cabin_speed_callback);
        interrupt0_sub = nh.subscribe("/web/moduan/interrupt_stop", 5, &pause_interrupt_Callback);
        hand_solve_stop = nh.subscribe("/web/moduan/hand_sovle_warn", 5, &solve_stop_Callback);
        moduan_work_sub = nh.subscribe("/moduan_work", 5, &moduan_work_Callback);
        send_odd_points_sub = nh.subscribe("/web/moduan/send_odd_points", 5, &checkerboard_jump_bind_callback);
    }
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}

int RunSuoquNode(int argc, char** argv)
{
    return RunSuoquNodeWithDefaultRole(argc, argv, "compat_all");
}
