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
#include <json.hpp>
#include <iostream>  
#include <memory>
#include <cctype>
#include <cstdio>
#include <tuple>
#include <limits>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <pthread.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <omp.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <chassis_ctrl/motion.h>
#include "chassis_ctrl/cabin_upload.h" //用于上传索驱模块反馈
#include "chassis_ctrl/MotionControl.h"
#include <std_msgs/Float32MultiArray.h>
#include "chassis_ctrl/cabin_move_all.h"
#include "chassis_ctrl/cabin_move_single.h"
#include "chassis_ctrl/cabin_calibration.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_upload.h"
#include "chassis_ctrl/AreaProgress.h"
#include <fast_image_solve/ProcessImage.h>
#include "fast_image_solve/PointCoords.h"
#include <fast_image_solve/ProcessImage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "chassis_ctrl/area_choose.h"
#include "chassis_ctrl/ExecuteBindPoints.h"
#include "std_srvs/Trigger.h" 
#include <chassis_ctrl/Pathguihua.h>
#include <chassis_ctrl/SingleMove.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "common.hpp"

using namespace std;

std::string path_points_json_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json";
std::string pseudo_slam_points_json_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json";
std::string pseudo_slam_bind_path_json_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json";
const std::string kBindExecutionMemoryJsonPath =
    "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json";

#define AXIS_X 0 //索驱轴体代表宏定义
#define AXIS_Y 3
#define AXIS_Z 4

// 定义互斥锁来保护 cabin_state 的读写操作
std::mutex cabin_state_mutex;
// 定义互斥锁避免同一时刻对TCP写入产生粘包现象
std::mutex socket_mutex;
std::mutex pseudo_slam_tf_points_mutex;
std::mutex pseudo_slam_ir_image_mutex;

[[noreturn]] void emergency_exit_with_flush(int exit_code)
{
    fflush(stdout);
    fflush(stderr);
    _exit(exit_code);
}

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
constexpr int DELAY_TIME_TIMEOUT_SEC = 30;
constexpr int DELAY_TIME_LOG_INTERVAL_SEC = 2;
constexpr uint8_t kProcessImageModeScanOnly = 3;
constexpr float kTravelMaxXMm = 320.0f;
constexpr float kTravelMaxYMm = 320.0f;
constexpr float kPseudoSlamDedupDistanceMm = 100.0f;
constexpr float kPseudoSlamPlanningZOutlierMm = 50.0f;
constexpr float kPseudoSlamCheckerboardAxisThresholdMm = 80.0f;
constexpr float kCurrentAreaBindTestCabinSpeedMultiplier = 1.5f;
constexpr float kCurrentAreaBindTestMinCabinSpeedMmPerSec = 450.0f;
constexpr int kPseudoSlamScanMinPointCount = 5;
constexpr double kPseudoSlamScanRetryIntervalSec = 0.2;
const std::string kPseudoSlamCaptureGateImageTopic = "/Scepter/ir/image_raw";
constexpr int kPseudoSlamCaptureGateStableSampleCount = 3;
constexpr double kPseudoSlamCaptureGatePollIntervalSec = 0.1;
constexpr double kPseudoSlamCaptureGateImageMeanDiffThreshold = 2.5;
constexpr double kPseudoSlamCaptureGateLogIntervalSec = 1.0;
constexpr float kPseudoSlamCaptureGateTargetToleranceMm = 25.0f;
constexpr float kPseudoSlamCaptureGatePoseDeltaToleranceMm = 1.0f;
constexpr int kPseudoSlamCaptureGateRoiMinX = 20;
constexpr int kPseudoSlamCaptureGateRoiMaxX = 620;
constexpr int kPseudoSlamCaptureGateRoiMinY = 114;
constexpr int kPseudoSlamCaptureGateRoiMaxY = 460;

enum class GlobalExecutionMode
{
    kSlamPrecomputed = 0,
    kLiveVisual = 1,
};

std::atomic<int> global_execution_mode{static_cast<int>(GlobalExecutionMode::kSlamPrecomputed)};

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

// 由计算模块得出的索驱路径点的X序列
struct Cabin_Point
{
    float x;
    float y;  
};

struct PseudoSlamAreaEntry
{
    int area_index;
    Cabin_Point cabin_point;
    std::vector<fast_image_solve::PointCoords> bind_points_world;
};

struct PseudoSlamBindGroup
{
    int group_index;
    std::string group_type;
    std::vector<fast_image_solve::PointCoords> bind_points_world;
};

struct PseudoSlamGroupedAreaEntry
{
    int area_index;
    Cabin_Point cabin_point;
    std::vector<PseudoSlamBindGroup> bind_groups;
};

struct PseudoSlamCheckerboardInfo
{
    int global_row = -1;
    int global_col = -1;
    int checkerboard_parity = -1;
    bool is_checkerboard_member = false;
};

struct BindExecutionPointRecord
{
    int global_row = -1;
    int global_col = -1;
    int checkerboard_parity = -1;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float world_z = 0.0f;
    std::string source_mode;
};

struct BindExecutionMemory
{
    std::string scan_session_id;
    Cabin_Point path_origin{};
    std::vector<BindExecutionPointRecord> executed_points;
};

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
chassis_ctrl::motion transform_msg;
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
chassis_ctrl::cabin_upload cabin_data_upload;
fast_image_solve::ProcessImage srv;
chassis_ctrl::MotionControl motion_srv;
ros::ServiceClient AI_client;
ros::ServiceClient sg_live_visual_client;
ros::ServiceClient sg_precomputed_client;
ros::ServiceClient sg_precomputed_fast_client;
ros::ServiceClient motion_client;
std::unique_ptr<tf2_ros::TransformBroadcaster> cabin_tf_broadcaster;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;
std::vector<fast_image_solve::PointCoords> pseudo_slam_tf_points;
std::vector<uint8_t> pseudo_slam_ir_roi_frame;
ros::Time pseudo_slam_ir_roi_stamp;

// 暂停中断标志位 0为未启用暂停中断或恢复，1为启用暂停中断
int handle_pause_interrupt = 0;
std::mutex error_msg;
 // 等待恢复信号
bool stop_flag = false;
bool moduan_work_flag = false;
bool checkerboard_jump_bind_enabled = false;

// 索驱状态查询的结构体
typedef struct {
    float X;
    float Y;
    float Z;
    float A;
    float B;
    float C;
    int motion_status;
    int device_alarm;
    int internal_calc_error;
    float cabin_x_gesture;
    float cabin_y_gesture;
} Cabin_State;
Cabin_State cabin_state;

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
    FILE *fp = fopen("/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/time_y_300.txt", "a");
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
    chassis_ctrl::AreaProgress progress_msg;
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
    printf("Cabin_log: pseudo_slam scan_only区域%d等待最终采集门通过（机械静止+画面静止）。\n", area_index);

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
            printf("Cabin_log: pseudo_slam scan_only区域%d最终采集门已通过，开始请求视觉。\n", area_index);
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        const double log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - last_log_time).count();
        if (log_elapsed_sec >= config.log_interval_sec) {
            printCurrentTime();
            printf(
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
    printf("Cabin_log:检测到ctrl+c，正在向索驱发送停止运动指令，关闭索驱系统，并广播急停消息。\n");
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
    std::ofstream outfile("/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/cabin_state.json");
    if(outfile.is_open()) {
        outfile << json_str;
        outfile.close();
        printf("Cabin_state JSON已保存到文件\n");
    } else {
        printf("无法打开文件保存JSON数据\n");
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
//     printf("增量式PID调试 - 设定值: %.2f, 反馈值: %.2f, 误差: %.2f\n", 
//            setpoint, feedback, error);
//     printf("积分累积: %.2f (限位: %.2f~%.2f)\n", 
//            pid->integral, pid->integral_min, pid->integral_max);
//     printf("增量输出: %.2f (P: %.2f, I: %.2f, D: %.2f)\n",
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
    for(int i = 0; i < 36; i++) {
        printf("%02X", TCP_Move_Frame[i]);
        if(i < 35) printf(" ");
    }
    printf("\n");
}

void printFrameBytes(const char* prefix, const uint8_t* data, size_t len)
{
    if (prefix == nullptr || data == nullptr) {
        return;
    }

    printf("%s", prefix);
    for (size_t i = 0; i < len; ++i) {
        printf("%02X", data[i]);
        if (i + 1 < len) {
            printf(" ");
        }
    }
    printf("\n");
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
                    // printf("Cabin_log: 路径点已成功保存为JSON文件\n");
                } else {
                    printf("Cabin_Error: 无法打开文件保存路径点\n");
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
                    // printf("Cabin_log: 路径点已成功保存为JSON文件\n");
                } else {
                    printf("Cabin_Error: 无法打开文件保存路径点\n");
                }
            }
        }

    }
    return Cabin_Coor;
}

void publish_cabin_depth_tf()
{
    if (!cabin_tf_broadcaster) {
        return;
    }

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "cabin_frame";
    transform.child_frame_id = "Scepter_depth_frame";
    {
        std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        transform.transform.translation.x = static_cast<double>(cabin_state.X) / 1000.0;
        transform.transform.translation.y = static_cast<double>(cabin_state.Y) / 1000.0;
        transform.transform.translation.z = static_cast<double>(cabin_state.Z) / 1000.0;
    }
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    cabin_tf_broadcaster->sendTransform(transform);
}

void set_pseudo_slam_tf_points(const std::vector<fast_image_solve::PointCoords>& world_points)
{
    std::lock_guard<std::mutex> lock(pseudo_slam_tf_points_mutex);
    pseudo_slam_tf_points = world_points;
}

void publish_pseudo_slam_point_transforms()
{
    if (!cabin_tf_broadcaster) {
        return;
    }

    std::vector<fast_image_solve::PointCoords> tf_points_snapshot;
    {
        std::lock_guard<std::mutex> lock(pseudo_slam_tf_points_mutex);
        tf_points_snapshot = pseudo_slam_tf_points;
    }

    if (tf_points_snapshot.empty()) {
        return;
    }

    std::vector<geometry_msgs::TransformStamped> point_transforms;
    point_transforms.reserve(tf_points_snapshot.size());
    const ros::Time stamp = ros::Time::now();
    for (const auto& world_point : tf_points_snapshot) {
        geometry_msgs::TransformStamped point_transform;
        point_transform.header.stamp = stamp;
        point_transform.header.frame_id = "cabin_frame";
        point_transform.child_frame_id = "pseudo_slam_point_" + std::to_string(world_point.idx);
        point_transform.transform.translation.x = static_cast<double>(world_point.World_coord[0]) / 1000.0;
        point_transform.transform.translation.y = static_cast<double>(world_point.World_coord[1]) / 1000.0;
        point_transform.transform.translation.z = static_cast<double>(world_point.World_coord[2]) / 1000.0;
        point_transform.transform.rotation.x = 0.0;
        point_transform.transform.rotation.y = 0.0;
        point_transform.transform.rotation.z = 0.0;
        point_transform.transform.rotation.w = 1.0;
        point_transforms.push_back(point_transform);
    }

    cabin_tf_broadcaster->sendTransform(point_transforms);
}

void clear_pseudo_slam_markers()
{
    if (!pub_pseudo_slam_markers) {
        return;
    }

    visualization_msgs::Marker clear_marker;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.header.frame_id = "cabin_frame";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(clear_marker);
    pub_pseudo_slam_markers.publish(marker_array);
}

void publish_pseudo_slam_markers(const std::vector<fast_image_solve::PointCoords>& world_points)
{
    if (!pub_pseudo_slam_markers) {
        return;
    }

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker points_marker;
    points_marker.header.stamp = ros::Time::now();
    points_marker.header.frame_id = "cabin_frame";
    points_marker.ns = "pseudo_slam_points";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.025;
    points_marker.scale.y = 0.025;
    points_marker.scale.z = 0.025;
    points_marker.color.r = 0.10f;
    points_marker.color.g = 0.95f;
    points_marker.color.b = 0.85f;
    points_marker.color.a = 1.0f;

    for (size_t point_pos = 0; point_pos < world_points.size(); ++point_pos) {
        const auto& world_point = world_points[point_pos];

        geometry_msgs::Point point_msg;
        point_msg.x = static_cast<double>(world_point.World_coord[0]) / 1000.0;
        point_msg.y = static_cast<double>(world_point.World_coord[1]) / 1000.0;
        point_msg.z = static_cast<double>(world_point.World_coord[2]) / 1000.0;
        points_marker.points.push_back(point_msg);

        visualization_msgs::Marker label_marker;
        label_marker.header.stamp = points_marker.header.stamp;
        label_marker.header.frame_id = "cabin_frame";
        label_marker.ns = "pseudo_slam_labels";
        label_marker.id = static_cast<int>(point_pos) + 1;
        label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label_marker.action = visualization_msgs::Marker::ADD;
        label_marker.pose.position = point_msg;
        label_marker.pose.position.z += 0.05;
        label_marker.pose.orientation.w = 1.0;
        label_marker.scale.z = 0.05;
        label_marker.color.r = 1.0f;
        label_marker.color.g = 1.0f;
        label_marker.color.b = 1.0f;
        label_marker.color.a = 1.0f;
        const int global_idx = world_point.idx > 0 ? world_point.idx : static_cast<int>(point_pos) + 1;
        label_marker.text = std::to_string(global_idx);
        marker_array.markers.push_back(label_marker);
    }

    marker_array.markers.insert(marker_array.markers.begin(), points_marker);
    pub_pseudo_slam_markers.publish(marker_array);
}

double point_distance_mm(const fast_image_solve::PointCoords& lhs, const fast_image_solve::PointCoords& rhs)
{
    const double dx = static_cast<double>(lhs.World_coord[0]) - static_cast<double>(rhs.World_coord[0]);
    const double dy = static_cast<double>(lhs.World_coord[1]) - static_cast<double>(rhs.World_coord[1]);
    const double dz = static_cast<double>(lhs.World_coord[2]) - static_cast<double>(rhs.World_coord[2]);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool transform_cabin_world_point_to_gripper_point(
    const fast_image_solve::PointCoords& world_point,
    fast_image_solve::PointCoords& gripper_point
)
{
    if (!tf_buffer_ptr) {
        return false;
    }

    try {
        geometry_msgs::PointStamped input_point;
        geometry_msgs::PointStamped output_point;
        input_point.header.stamp = ros::Time(0);
        input_point.header.frame_id = "cabin_frame";
        input_point.point.x = static_cast<double>(world_point.World_coord[0]) / 1000.0;
        input_point.point.y = static_cast<double>(world_point.World_coord[1]) / 1000.0;
        input_point.point.z = static_cast<double>(world_point.World_coord[2]) / 1000.0;
        tf_buffer_ptr->transform(input_point, output_point, "gripper_frame", ros::Duration(0.2));

        gripper_point = world_point;
        gripper_point.World_coord[0] = static_cast<float>(output_point.point.x * 1000.0);
        gripper_point.World_coord[1] = static_cast<float>(output_point.point.y * 1000.0);
        gripper_point.World_coord[2] = static_cast<float>(output_point.point.z * 1000.0);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: world->gripper变换失败: %s", ex.what());
        return false;
    }
}

bool is_local_bind_point_in_range(const fast_image_solve::PointCoords& point)
{
    return point.World_coord[0] >= 0.0f &&
           point.World_coord[0] <= kTravelMaxXMm &&
           point.World_coord[1] >= 0.0f &&
           point.World_coord[1] <= kTravelMaxYMm;
}

std::vector<fast_image_solve::PointCoords> dedupe_world_points(
    const std::vector<fast_image_solve::PointCoords>& input_points
)
{
    std::vector<fast_image_solve::PointCoords> deduped_points;
    for (const auto& point : input_points) {
        bool is_duplicate = false;
        for (const auto& existing_point : deduped_points) {
            if (point_distance_mm(point, existing_point) < kPseudoSlamDedupDistanceMm) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            deduped_points.push_back(point);
        }
    }
    return deduped_points;
}

float median_world_z_mm(const std::vector<fast_image_solve::PointCoords>& world_points)
{
    if (world_points.empty()) {
        return 0.0f;
    }

    std::vector<float> z_values;
    z_values.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        z_values.push_back(world_point.World_coord[2]);
    }

    std::sort(z_values.begin(), z_values.end());
    const size_t middle_index = z_values.size() / 2;
    if (z_values.size() % 2 == 1) {
        return z_values[middle_index];
    }
    return (z_values[middle_index - 1] + z_values[middle_index]) * 0.5f;
}

std::vector<fast_image_solve::PointCoords> filter_pseudo_slam_planning_outliers(
    const std::vector<fast_image_solve::PointCoords>& world_points
)
{
    if (world_points.empty()) {
        return {};
    }

    const float median_z = median_world_z_mm(world_points);
    std::vector<fast_image_solve::PointCoords> planning_points;
    planning_points.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        if (std::fabs(world_point.World_coord[2] - median_z) <= kPseudoSlamPlanningZOutlierMm) {
            planning_points.push_back(world_point);
        }
    }

    if (planning_points.size() != world_points.size()) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam规划离群过滤：可视化保留%d个点，规划使用%d个点，剔除%d个点，z中位数=%.1fmm，阈值=±%.1fmm。\n",
            static_cast<int>(world_points.size()),
            static_cast<int>(planning_points.size()),
            static_cast<int>(world_points.size() - planning_points.size()),
            median_z,
            kPseudoSlamPlanningZOutlierMm
        );
    }

    return planning_points;
}

std::vector<float> cluster_checkerboard_axis_centers(
    const std::vector<fast_image_solve::PointCoords>& world_points,
    int axis_index,
    float threshold_mm
)
{
    if (world_points.empty()) {
        return {};
    }

    std::vector<float> axis_values;
    axis_values.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        axis_values.push_back(world_point.World_coord[axis_index]);
    }
    std::sort(axis_values.begin(), axis_values.end());

    std::vector<float> centers;
    std::vector<float> current_group = {axis_values.front()};
    double current_mean = static_cast<double>(axis_values.front());
    for (size_t value_index = 1; value_index < axis_values.size(); ++value_index) {
        const double axis_value = static_cast<double>(axis_values[value_index]);
        if (std::fabs(axis_value - current_mean) <= threshold_mm) {
            current_group.push_back(axis_values[value_index]);
            double axis_sum = 0.0;
            for (const float grouped_value : current_group) {
                axis_sum += grouped_value;
            }
            current_mean = axis_sum / static_cast<double>(current_group.size());
        } else {
            centers.push_back(static_cast<float>(current_mean));
            current_group = {axis_values[value_index]};
            current_mean = axis_value;
        }
    }
    if (!current_group.empty()) {
        centers.push_back(static_cast<float>(current_mean));
    }
    return centers;
}

int find_nearest_checkerboard_center_index(float axis_value, const std::vector<float>& centers)
{
    if (centers.empty()) {
        return -1;
    }

    int best_index = 0;
    float best_gap = std::fabs(axis_value - centers.front());
    for (size_t center_index = 1; center_index < centers.size(); ++center_index) {
        const float gap = std::fabs(axis_value - centers[center_index]);
        if (gap < best_gap) {
            best_gap = gap;
            best_index = static_cast<int>(center_index);
        }
    }
    return best_index;
}

long long encode_checkerboard_cell_key(int global_row, int global_col)
{
    return (static_cast<long long>(global_row) << 32) ^
           static_cast<unsigned int>(global_col);
}

std::unordered_map<int, PseudoSlamCheckerboardInfo> build_checkerboard_info_by_global_index(
    const std::vector<fast_image_solve::PointCoords>& world_points,
    const Cabin_Point& path_origin
)
{
    std::unordered_map<int, PseudoSlamCheckerboardInfo> checkerboard_info_by_idx;
    if (world_points.empty()) {
        return checkerboard_info_by_idx;
    }

    const std::vector<float> x_centers = cluster_checkerboard_axis_centers(
        world_points,
        0,
        kPseudoSlamCheckerboardAxisThresholdMm
    );
    const std::vector<float> y_centers = cluster_checkerboard_axis_centers(
        world_points,
        1,
        kPseudoSlamCheckerboardAxisThresholdMm
    );
    if (x_centers.empty() || y_centers.empty()) {
        return checkerboard_info_by_idx;
    }

    std::unordered_set<long long> occupied_cells;
    int phase_reference = 0;
    double nearest_origin_distance_sq = std::numeric_limits<double>::max();
    for (const auto& world_point : world_points) {
        if (world_point.idx <= 0) {
            continue;
        }

        PseudoSlamCheckerboardInfo info;
        info.global_col = find_nearest_checkerboard_center_index(world_point.World_coord[0], x_centers);
        info.global_row = find_nearest_checkerboard_center_index(world_point.World_coord[1], y_centers);
        if (info.global_row < 0 || info.global_col < 0) {
            continue;
        }

        occupied_cells.insert(encode_checkerboard_cell_key(info.global_row, info.global_col));
        checkerboard_info_by_idx[world_point.idx] = info;
        const double distance_sq =
            static_cast<double>(world_point.World_coord[0] - path_origin.x) * static_cast<double>(world_point.World_coord[0] - path_origin.x) +
            static_cast<double>(world_point.World_coord[1] - path_origin.y) * static_cast<double>(world_point.World_coord[1] - path_origin.y);
        if (distance_sq < nearest_origin_distance_sq) {
            nearest_origin_distance_sq = distance_sq;
            phase_reference = (info.global_row + info.global_col) % 2;
        }
    }

    for (auto& entry : checkerboard_info_by_idx) {
        auto& info = entry.second;
        info.checkerboard_parity = (info.global_row + info.global_col + phase_reference) % 2;

        const bool has_horizontal_neighbor =
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row, info.global_col - 1)) > 0 ||
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row, info.global_col + 1)) > 0;
        const bool has_vertical_neighbor =
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row - 1, info.global_col)) > 0 ||
            occupied_cells.count(encode_checkerboard_cell_key(info.global_row + 1, info.global_col)) > 0;
        const bool can_form_matrix =
            has_horizontal_neighbor &&
            has_vertical_neighbor;
        const bool can_form_edge_pair = has_horizontal_neighbor || has_vertical_neighbor;
        info.is_checkerboard_member = can_form_matrix || can_form_edge_pair;
    }
    return checkerboard_info_by_idx;
}

std::vector<fast_image_solve::PointCoords> filter_pseudo_slam_non_checkerboard_points(
    const std::vector<fast_image_solve::PointCoords>& planning_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx
)
{
    if (planning_points.empty() || checkerboard_info_by_idx.empty()) {
        return planning_points;
    }

    std::vector<fast_image_solve::PointCoords> filtered_points;
    filtered_points.reserve(planning_points.size());
    int removed_count = 0;
    for (const auto& planning_point : planning_points) {
        const auto checkerboard_it = checkerboard_info_by_idx.find(planning_point.idx);
        if (checkerboard_it == checkerboard_info_by_idx.end() ||
            !checkerboard_it->second.is_checkerboard_member) {
            removed_count++;
            continue;
        }
        filtered_points.push_back(planning_point);
    }

    if (removed_count > 0) {
        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam棋盘格成员过滤：无法融入棋盘格行列邻接的点视为离群点，移除%d个点，剩余%d个规划点。\n",
            removed_count,
            static_cast<int>(filtered_points.size())
        );
    }

    return filtered_points;
}

bool lookup_gripper_from_scepter_transform(tf2::Transform& gripper_from_scepter)
{
    if (!tf_buffer_ptr) {
        return false;
    }

    try {
        const geometry_msgs::TransformStamped transform_msg = tf_buffer_ptr->lookupTransform(
            "gripper_frame",
            "Scepter_depth_frame",
            ros::Time(0),
            ros::Duration(0.2)
        );
        tf2::fromMsg(transform_msg.transform, gripper_from_scepter);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "Cabin_Warn: Scepter_depth_frame->gripper_frame静态变换失败: %s", ex.what());
        return false;
    }
}

bool transform_cabin_world_point_to_planned_gripper_point(
    const fast_image_solve::PointCoords& world_point,
    const Cabin_Point& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    fast_image_solve::PointCoords& gripper_point
)
{
    const tf2::Vector3 point_in_scepter_frame(
        static_cast<double>(world_point.World_coord[0] - cabin_point.x) / 1000.0,
        static_cast<double>(world_point.World_coord[1] - cabin_point.y) / 1000.0,
        static_cast<double>(world_point.World_coord[2] - cabin_height) / 1000.0
    );
    const tf2::Vector3 point_in_gripper_frame = gripper_from_scepter * point_in_scepter_frame;

    gripper_point = world_point;
    gripper_point.World_coord[0] = static_cast<float>(point_in_gripper_frame.x() * 1000.0);
    gripper_point.World_coord[1] = static_cast<float>(point_in_gripper_frame.y() * 1000.0);
    gripper_point.World_coord[2] = static_cast<float>(point_in_gripper_frame.z() * 1000.0);
    return true;
}

struct PseudoSlamCandidatePoint
{
    fast_image_solve::PointCoords world_point;
    fast_image_solve::PointCoords local_point;
    size_t remaining_index;
};

double local_origin_distance_sq(const fast_image_solve::PointCoords& point)
{
    const double x = static_cast<double>(point.World_coord[0]);
    const double y = static_cast<double>(point.World_coord[1]);
    return x * x + y * y;
}

std::vector<std::vector<int>> group_candidate_indices_by_axis(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    int axis_index,
    float threshold
)
{
    if (candidates.empty()) {
        return {};
    }

    std::vector<int> sorted_indices(candidates.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(
        sorted_indices.begin(),
        sorted_indices.end(),
        [&](int lhs_index, int rhs_index) {
            const auto& lhs = candidates[lhs_index].local_point;
            const auto& rhs = candidates[rhs_index].local_point;
            if (lhs.World_coord[axis_index] != rhs.World_coord[axis_index]) {
                return lhs.World_coord[axis_index] < rhs.World_coord[axis_index];
            }
            if (lhs.World_coord[1 - axis_index] != rhs.World_coord[1 - axis_index]) {
                return lhs.World_coord[1 - axis_index] < rhs.World_coord[1 - axis_index];
            }
            return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
        }
    );

    std::vector<std::vector<int>> grouped_indices;
    std::vector<int> current_group = {sorted_indices.front()};
    double current_mean = static_cast<double>(
        candidates[sorted_indices.front()].local_point.World_coord[axis_index]
    );
    for (size_t sorted_pos = 1; sorted_pos < sorted_indices.size(); ++sorted_pos) {
        const int candidate_index = sorted_indices[sorted_pos];
        const double axis_value = static_cast<double>(
            candidates[candidate_index].local_point.World_coord[axis_index]
        );
        if (std::fabs(axis_value - current_mean) <= threshold) {
            current_group.push_back(candidate_index);
            double axis_sum = 0.0;
            for (const int grouped_index : current_group) {
                axis_sum += candidates[grouped_index].local_point.World_coord[axis_index];
            }
            current_mean = axis_sum / static_cast<double>(current_group.size());
        } else {
            grouped_indices.push_back(current_group);
            current_group = {candidate_index};
            current_mean = axis_value;
        }
    }

    if (!current_group.empty()) {
        grouped_indices.push_back(current_group);
    }
    return grouped_indices;
}

std::vector<std::tuple<int, int, float>> match_candidate_indices_between_rows(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& upper_row,
    const std::vector<int>& lower_row,
    float column_threshold
)
{
    std::vector<int> upper_sorted = upper_row;
    std::vector<int> lower_sorted = lower_row;
    auto by_local_y = [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        if (lhs.World_coord[1] != rhs.World_coord[1]) {
            return lhs.World_coord[1] < rhs.World_coord[1];
        }
        if (lhs.World_coord[0] != rhs.World_coord[0]) {
            return lhs.World_coord[0] < rhs.World_coord[0];
        }
        return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
    };
    std::sort(upper_sorted.begin(), upper_sorted.end(), by_local_y);
    std::sort(lower_sorted.begin(), lower_sorted.end(), by_local_y);

    std::vector<std::tuple<int, int, float>> matched_pairs;
    std::vector<bool> lower_used(lower_sorted.size(), false);
    for (const int upper_index : upper_sorted) {
        const float upper_y = candidates[upper_index].local_point.World_coord[1];
        int best_lower_position = -1;
        float best_gap = 0.0f;
        for (size_t lower_pos = 0; lower_pos < lower_sorted.size(); ++lower_pos) {
            if (lower_used[lower_pos]) {
                continue;
            }
            const float lower_y = candidates[lower_sorted[lower_pos]].local_point.World_coord[1];
            const float gap = std::fabs(upper_y - lower_y);
            if (gap > column_threshold) {
                continue;
            }
            if (best_lower_position < 0 || gap < best_gap) {
                best_lower_position = static_cast<int>(lower_pos);
                best_gap = gap;
            }
        }
        if (best_lower_position < 0) {
            continue;
        }

        lower_used[best_lower_position] = true;
        matched_pairs.emplace_back(
            upper_index,
            lower_sorted[best_lower_position],
            best_gap
        );
    }

    std::sort(
        matched_pairs.begin(),
        matched_pairs.end(),
        [&](const auto& lhs, const auto& rhs) {
            const auto& lhs_upper = candidates[std::get<0>(lhs)].local_point;
            const auto& lhs_lower = candidates[std::get<1>(lhs)].local_point;
            const auto& rhs_upper = candidates[std::get<0>(rhs)].local_point;
            const auto& rhs_lower = candidates[std::get<1>(rhs)].local_point;
            const float lhs_min_y = std::min(lhs_upper.World_coord[1], lhs_lower.World_coord[1]);
            const float rhs_min_y = std::min(rhs_upper.World_coord[1], rhs_lower.World_coord[1]);
            if (lhs_min_y != rhs_min_y) {
                return lhs_min_y < rhs_min_y;
            }
            if (std::get<2>(lhs) != std::get<2>(rhs)) {
                return std::get<2>(lhs) < std::get<2>(rhs);
            }
            if (lhs_upper.World_coord[0] != rhs_upper.World_coord[0]) {
                return lhs_upper.World_coord[0] < rhs_upper.World_coord[0];
            }
            return lhs_lower.World_coord[0] < rhs_lower.World_coord[0];
        }
    );
    return matched_pairs;
}

std::vector<int> sort_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& matrix_indices
)
{
    if (matrix_indices.size() != 4) {
        std::vector<int> sorted_indices = matrix_indices;
        std::sort(
            sorted_indices.begin(),
            sorted_indices.end(),
            [&](int lhs_index, int rhs_index) {
                const auto& lhs = candidates[lhs_index].local_point;
                const auto& rhs = candidates[rhs_index].local_point;
                if (local_origin_distance_sq(lhs) != local_origin_distance_sq(rhs)) {
                    return local_origin_distance_sq(lhs) < local_origin_distance_sq(rhs);
                }
                if (lhs.World_coord[0] != rhs.World_coord[0]) {
                    return lhs.World_coord[0] < rhs.World_coord[0];
                }
                if (lhs.World_coord[1] != rhs.World_coord[1]) {
                    return lhs.World_coord[1] < rhs.World_coord[1];
                }
                return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
            }
        );
        return sorted_indices;
    }

    auto point1_it = std::min_element(
        matrix_indices.begin(),
        matrix_indices.end(),
        [&](int lhs_index, int rhs_index) {
            const auto& lhs = candidates[lhs_index].local_point;
            const auto& rhs = candidates[rhs_index].local_point;
            if (local_origin_distance_sq(lhs) != local_origin_distance_sq(rhs)) {
                return local_origin_distance_sq(lhs) < local_origin_distance_sq(rhs);
            }
            if (lhs.World_coord[0] != rhs.World_coord[0]) {
                return lhs.World_coord[0] < rhs.World_coord[0];
            }
            if (lhs.World_coord[1] != rhs.World_coord[1]) {
                return lhs.World_coord[1] < rhs.World_coord[1];
            }
            return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
        }
    );
    const int point1_index = *point1_it;
    const float point1_x = candidates[point1_index].local_point.World_coord[0];
    const float point1_y = candidates[point1_index].local_point.World_coord[1];

    std::vector<int> remaining_indices;
    for (const int candidate_index : matrix_indices) {
        if (candidate_index != point1_index) {
            remaining_indices.push_back(candidate_index);
        }
    }

    auto point2_it = std::min_element(
        remaining_indices.begin(),
        remaining_indices.end(),
        [&](int lhs_index, int rhs_index) {
            const auto& lhs = candidates[lhs_index].local_point;
            const auto& rhs = candidates[rhs_index].local_point;
            const auto lhs_key = std::make_tuple(
                std::fabs(lhs.World_coord[1] - point1_y),
                -std::fabs(lhs.World_coord[0] - point1_x),
                lhs.World_coord[0],
                candidates[lhs_index].remaining_index
            );
            const auto rhs_key = std::make_tuple(
                std::fabs(rhs.World_coord[1] - point1_y),
                -std::fabs(rhs.World_coord[0] - point1_x),
                rhs.World_coord[0],
                candidates[rhs_index].remaining_index
            );
            return lhs_key < rhs_key;
        }
    );
    const int point2_index = *point2_it;
    remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), point2_index), remaining_indices.end());

    auto point3_it = std::min_element(
        remaining_indices.begin(),
        remaining_indices.end(),
        [&](int lhs_index, int rhs_index) {
            const auto& lhs = candidates[lhs_index].local_point;
            const auto& rhs = candidates[rhs_index].local_point;
            const auto lhs_key = std::make_tuple(
                std::fabs(lhs.World_coord[0] - point1_x),
                -std::fabs(lhs.World_coord[1] - point1_y),
                lhs.World_coord[1],
                candidates[lhs_index].remaining_index
            );
            const auto rhs_key = std::make_tuple(
                std::fabs(rhs.World_coord[0] - point1_x),
                -std::fabs(rhs.World_coord[1] - point1_y),
                rhs.World_coord[1],
                candidates[rhs_index].remaining_index
            );
            return lhs_key < rhs_key;
        }
    );
    const int point3_index = *point3_it;
    remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), point3_index), remaining_indices.end());
    const int point4_index = remaining_indices.front();
    return {point1_index, point2_index, point3_index, point4_index};
}

std::vector<double> build_matrix_candidate_score(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& sorted_matrix_indices,
    const std::vector<float>& column_gaps
)
{
    std::vector<double> score_key;
    std::vector<float> x_values;
    std::vector<float> y_values;
    std::vector<float> sorted_gaps = column_gaps;
    std::sort(sorted_gaps.begin(), sorted_gaps.end());

    for (const int candidate_index : sorted_matrix_indices) {
        const auto& local_point = candidates[candidate_index].local_point;
        score_key.push_back(local_origin_distance_sq(local_point));
        score_key.push_back(local_point.World_coord[0]);
        score_key.push_back(local_point.World_coord[1]);
        score_key.push_back(static_cast<double>(candidates[candidate_index].remaining_index));
        x_values.push_back(local_point.World_coord[0]);
        y_values.push_back(local_point.World_coord[1]);
    }

    std::sort(x_values.begin(), x_values.end());
    std::sort(y_values.begin(), y_values.end());
    for (const float x_value : x_values) {
        score_key.push_back(x_value);
    }
    for (const float y_value : y_values) {
        score_key.push_back(y_value);
    }
    for (const float gap : sorted_gaps) {
        score_key.push_back(gap);
    }
    return score_key;
}

std::vector<int> select_nearest_origin_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    float row_threshold = 40.0f,
    float column_threshold = 45.0f
)
{
    if (candidates.size() < 4) {
        return {};
    }

    const std::vector<std::vector<int>> rows = group_candidate_indices_by_axis(
        candidates,
        0,
        row_threshold
    );
    if (rows.size() < 2) {
        return {};
    }

    std::vector<int> best_indices;
    std::vector<double> best_score;
    for (size_t upper_index = 0; upper_index + 1 < rows.size(); ++upper_index) {
        const auto& upper_row = rows[upper_index];
        if (upper_row.size() < 2) {
            continue;
        }

        for (size_t lower_index = upper_index + 1; lower_index < rows.size(); ++lower_index) {
            const auto& lower_row = rows[lower_index];
            if (lower_row.size() < 2) {
                continue;
            }

            const auto matched_pairs = match_candidate_indices_between_rows(
                candidates,
                upper_row,
                lower_row,
                column_threshold
            );
            if (matched_pairs.size() < 2) {
                continue;
            }

            for (size_t first_pair_index = 0; first_pair_index + 1 < matched_pairs.size(); ++first_pair_index) {
                for (size_t second_pair_index = first_pair_index + 1; second_pair_index < matched_pairs.size(); ++second_pair_index) {
                    const auto& first_pair = matched_pairs[first_pair_index];
                    const auto& second_pair = matched_pairs[second_pair_index];
                    std::vector<int> selected_indices = {
                        std::get<0>(first_pair),
                        std::get<1>(first_pair),
                        std::get<0>(second_pair),
                        std::get<1>(second_pair),
                    };
                    const std::vector<int> sorted_indices = sort_matrix_candidate_indices(candidates, selected_indices);
                    const std::vector<double> score = build_matrix_candidate_score(
                        candidates,
                        sorted_indices,
                        {std::get<2>(first_pair), std::get<2>(second_pair)}
                    );
                    if (best_indices.empty() || score < best_score) {
                        best_indices = sorted_indices;
                        best_score = score;
                    }
                }
            }
        }
    }

    return best_indices;
}

std::vector<int> select_nearest_origin_edge_pair_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    float row_threshold = 40.0f,
    float column_threshold = 45.0f
)
{
    if (candidates.size() < 2) {
        return {};
    }

    std::vector<int> best_indices;
    std::vector<double> best_score;
    const std::vector<std::vector<int>> grouped_by_x = group_candidate_indices_by_axis(
        candidates,
        0,
        row_threshold
    );
    const std::vector<std::vector<int>> grouped_by_y = group_candidate_indices_by_axis(
        candidates,
        1,
        column_threshold
    );

    auto consider_grouped_pairs = [&](const std::vector<std::vector<int>>& grouped_indices, int grouped_axis_index) {
        for (const auto& grouped_pair_candidates : grouped_indices) {
            if (grouped_pair_candidates.size() < 2) {
                continue;
            }

            for (size_t lhs_pos = 0; lhs_pos + 1 < grouped_pair_candidates.size(); ++lhs_pos) {
                for (size_t rhs_pos = lhs_pos + 1; rhs_pos < grouped_pair_candidates.size(); ++rhs_pos) {
                    std::vector<int> pair_indices = {
                        grouped_pair_candidates[lhs_pos],
                        grouped_pair_candidates[rhs_pos],
                    };
                    const std::vector<int> sorted_pair_indices = sort_matrix_candidate_indices(candidates, pair_indices);
                    const auto& first_point = candidates[sorted_pair_indices[0]].local_point;
                    const auto& second_point = candidates[sorted_pair_indices[1]].local_point;
                    const int paired_axis_index = 1 - grouped_axis_index;
                    const std::vector<double> score = {
                        std::max(
                            local_origin_distance_sq(first_point),
                            local_origin_distance_sq(second_point)
                        ),
                        local_origin_distance_sq(first_point) + local_origin_distance_sq(second_point),
                        std::fabs(
                            first_point.World_coord[grouped_axis_index] -
                            second_point.World_coord[grouped_axis_index]
                        ),
                        -std::fabs(
                            first_point.World_coord[paired_axis_index] -
                            second_point.World_coord[paired_axis_index]
                        ),
                        first_point.World_coord[0],
                        first_point.World_coord[1],
                        second_point.World_coord[0],
                        second_point.World_coord[1],
                        static_cast<double>(candidates[sorted_pair_indices[0]].remaining_index),
                        static_cast<double>(candidates[sorted_pair_indices[1]].remaining_index),
                    };
                    if (best_indices.empty() || score < best_score) {
                        best_indices = sorted_pair_indices;
                        best_score = score;
                    }
                }
            }
        }
    };

    consider_grouped_pairs(grouped_by_x, 0);
    consider_grouped_pairs(grouped_by_y, 1);
    return best_indices;
}

std::vector<PseudoSlamCandidatePoint> build_area_bind_candidates(
    const std::vector<fast_image_solve::PointCoords>& remaining_world_points,
    const Cabin_Point& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter
)
{
    std::vector<PseudoSlamCandidatePoint> candidates;
    for (size_t world_index = 0; world_index < remaining_world_points.size(); ++world_index) {
        fast_image_solve::PointCoords local_point;
        if (!transform_cabin_world_point_to_planned_gripper_point(
                remaining_world_points[world_index],
                cabin_point,
                cabin_height,
                gripper_from_scepter,
                local_point
            )) {
            continue;
        }
        if (!is_local_bind_point_in_range(local_point)) {
            continue;
        }
        candidates.push_back({remaining_world_points[world_index], local_point, world_index});
    }
    return candidates;
}

std::vector<PseudoSlamBindGroup> build_bind_groups_from_scan_world(
    std::vector<fast_image_solve::PointCoords>& remaining_world_points,
    const Cabin_Point& cabin_point,
    float cabin_height
)
{
    tf2::Transform gripper_from_scepter;
    if (!lookup_gripper_from_scepter_transform(gripper_from_scepter)) {
        return {};
    }

    std::vector<PseudoSlamBindGroup> bind_groups;
    int group_index = 1;
    while (true) {
        const std::vector<PseudoSlamCandidatePoint> candidates = build_area_bind_candidates(
            remaining_world_points,
            cabin_point,
            cabin_height,
            gripper_from_scepter
        );
        std::vector<int> selected_indices = select_nearest_origin_matrix_candidate_indices(candidates);
        if (selected_indices.empty()) {
            selected_indices = select_nearest_origin_edge_pair_candidate_indices(candidates);
        }

        PseudoSlamBindGroup bind_group;
        bind_group.group_index = group_index++;
        if (selected_indices.size() == 4) {
            bind_group.group_type = "matrix_2x2";
        } else if (selected_indices.size() == 2) {
            bind_group.group_type = "edge_pair";
        } else {
            break;
        }

        std::vector<size_t> consumed_remaining_indices;
        for (const int candidate_index : selected_indices) {
            auto world_point = candidates[candidate_index].world_point;
            bind_group.bind_points_world.push_back(world_point);
            consumed_remaining_indices.push_back(candidates[candidate_index].remaining_index);
        }

        std::sort(consumed_remaining_indices.begin(), consumed_remaining_indices.end());
        consumed_remaining_indices.erase(
            std::unique(consumed_remaining_indices.begin(), consumed_remaining_indices.end()),
            consumed_remaining_indices.end()
        );
        std::sort(consumed_remaining_indices.rbegin(), consumed_remaining_indices.rend());
        for (const size_t consumed_index : consumed_remaining_indices) {
            if (consumed_index < remaining_world_points.size()) {
                remaining_world_points.erase(remaining_world_points.begin() + consumed_index);
            }
        }

        bind_groups.push_back(bind_group);
    }

    return bind_groups;
}

std::vector<fast_image_solve::PointCoords> flatten_bind_groups(
    const std::vector<PseudoSlamBindGroup>& bind_groups
)
{
    std::vector<fast_image_solve::PointCoords> flattened_points;
    for (const auto& bind_group : bind_groups) {
        flattened_points.insert(
            flattened_points.end(),
            bind_group.bind_points_world.begin(),
            bind_group.bind_points_world.end()
        );
    }
    return flattened_points;
}

BindExecutionMemory load_bind_execution_memory_json()
{
    BindExecutionMemory memory;
    std::ifstream file_obj(kBindExecutionMemoryJsonPath);
    if (!file_obj.is_open()) {
        if (access(kBindExecutionMemoryJsonPath.c_str(), F_OK) == 0) {
            printCurrentTime();
            printf("Cabin_log: bind_execution_memory.json读取或解析失败，无法打开现有记忆文件。\n");
        }
        return memory;
    }

    try {
        nlohmann::json memory_json;
        file_obj >> memory_json;
        if (file_obj.fail() && !file_obj.eof()) {
            throw std::runtime_error("bind execution memory read failure");
        }
        memory.scan_session_id = memory_json.value("scan_session_id", "");
        if (memory_json.contains("path_origin") && memory_json["path_origin"].is_object()) {
            const auto& path_origin_json = memory_json["path_origin"];
            memory.path_origin.x = path_origin_json.value("x", 0.0f);
            memory.path_origin.y = path_origin_json.value("y", 0.0f);
        }
        if (memory_json.contains("executed_points") && memory_json["executed_points"].is_array()) {
            for (const auto& point_json : memory_json["executed_points"]) {
                BindExecutionPointRecord point_record;
                point_record.global_row = point_json.value("global_row", -1);
                point_record.global_col = point_json.value("global_col", -1);
                point_record.checkerboard_parity = point_json.value("checkerboard_parity", -1);
                point_record.world_x = point_json.value("world_x", 0.0f);
                point_record.world_y = point_json.value("world_y", 0.0f);
                point_record.world_z = point_json.value("world_z", 0.0f);
                point_record.source_mode = point_json.value("source_mode", "");
                memory.executed_points.push_back(point_record);
            }
        }
    } catch (const std::exception&) {
        printCurrentTime();
        printf("Cabin_log: bind_execution_memory.json读取或解析失败，保留告警并返回空记忆。\n");
        return BindExecutionMemory{};
    }

    return memory;
}

bool write_bind_execution_memory_json(const BindExecutionMemory& memory, std::string* error_message)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    nlohmann::json memory_json = {
        {"scan_session_id", memory.scan_session_id},
        {"path_origin", {{"x", memory.path_origin.x}, {"y", memory.path_origin.y}, {"z", 0.0f}}},
        {"executed_points", nlohmann::json::array()},
    };
    for (const auto& point_record : memory.executed_points) {
        memory_json["executed_points"].push_back(
            {
                {"global_row", point_record.global_row},
                {"global_col", point_record.global_col},
                {"checkerboard_parity", point_record.checkerboard_parity},
                {"world_x", point_record.world_x},
                {"world_y", point_record.world_y},
                {"world_z", point_record.world_z},
                {"source_mode", point_record.source_mode},
            }
        );
    }

    const std::string temp_json_path = kBindExecutionMemoryJsonPath + ".tmp";
    std::ofstream file_obj(temp_json_path);
    if (!file_obj.is_open()) {
        if (error_message != nullptr) {
            *error_message = "无法打开bind_execution_memory.json临时文件进行写入";
        }
        return false;
    }

    file_obj << memory_json.dump(4);
    file_obj.flush();
    file_obj.close();
    if (!file_obj.good()) {
        std::remove(temp_json_path.c_str());
        if (error_message != nullptr) {
            *error_message = "写入bind_execution_memory.json临时文件失败";
        }
        return false;
    }
    if (std::rename(temp_json_path.c_str(), kBindExecutionMemoryJsonPath.c_str()) != 0) {
        std::remove(temp_json_path.c_str());
        if (error_message != nullptr) {
            *error_message = "原子替换bind_execution_memory.json失败";
        }
        return false;
    }
    return true;
}

BindExecutionMemory reset_bind_execution_memory_for_scan_session(
    const std::string& scan_session_id,
    const Cabin_Point& path_origin
)
{
    BindExecutionMemory memory;
    memory.scan_session_id = scan_session_id;
    memory.path_origin = path_origin;
    memory.executed_points.clear();
    return memory;
}

bool is_point_already_executed(
    const BindExecutionMemory& memory,
    int global_row,
    int global_col
)
{
    if (global_row < 0 || global_col < 0) {
        return false;
    }

    return std::any_of(
        memory.executed_points.begin(),
        memory.executed_points.end(),
        [global_row, global_col](const BindExecutionPointRecord& point_record) {
            return point_record.global_row == global_row && point_record.global_col == global_col;
        }
    );
}

nlohmann::json filter_precomputed_group_points_for_execution(
    const nlohmann::json& group_json,
    const BindExecutionMemory& memory,
    bool only_checkerboard_parity_zero
)
{
    nlohmann::json filtered_points = nlohmann::json::array();
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        return filtered_points;
    }

    for (const auto& point_json : group_json["points"]) {
        if (only_checkerboard_parity_zero && point_json.value("checkerboard_parity", 0) != 0) {
            continue;
        }

        const int global_row = point_json.value("global_row", -1);
        const int global_col = point_json.value("global_col", -1);
        if (is_point_already_executed(memory, global_row, global_col)) {
            continue;
        }

        filtered_points.push_back(point_json);
    }

    return filtered_points;
}

void record_successful_execution_point(
    BindExecutionMemory& memory,
    const nlohmann::json& point_json
)
{
    const int global_row = point_json.value("global_row", -1);
    const int global_col = point_json.value("global_col", -1);
    if (global_row < 0 || global_col < 0) {
        return;
    }
    if (is_point_already_executed(memory, global_row, global_col)) {
        return;
    }

    BindExecutionPointRecord point_record;
    point_record.global_row = global_row;
    point_record.global_col = global_col;
    point_record.checkerboard_parity = point_json.value("checkerboard_parity", -1);
    point_record.world_x = point_json.value("world_x", point_json.value("x", 0.0f));
    point_record.world_y = point_json.value("world_y", point_json.value("y", 0.0f));
    point_record.world_z = point_json.value("world_z", point_json.value("z", 0.0f));
    point_record.source_mode = "slam_precomputed";
    memory.executed_points.push_back(point_record);
}

void write_pseudo_slam_points_json(
    const std::vector<fast_image_solve::PointCoords>& merged_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx
)
{
    nlohmann::json points_json;
    points_json["pseudo_slam_points"] = nlohmann::json::array();
    for (const auto& point : merged_points) {
        auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
        const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
        const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
        const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
        const bool is_checkerboard_member =
            checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
        points_json["pseudo_slam_points"].push_back(
            {
                {"idx", point.idx},
                {"global_idx", point.idx},
                {"global_row", global_row},
                {"global_col", global_col},
                {"checkerboard_parity", checkerboard_parity},
                {"is_checkerboard_member", is_checkerboard_member},
                {"x", point.World_coord[0]},
                {"y", point.World_coord[1]},
                {"z", point.World_coord[2]},
                {"angle", point.Angle},
            }
        );
    }
    std::ofstream file_obj(pseudo_slam_points_json_file);
    if (file_obj.is_open()) {
        file_obj << points_json.dump(4);
    }
}

void write_pseudo_slam_bind_path_json(
    const std::vector<PseudoSlamGroupedAreaEntry>& area_entries,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const Cabin_Point& path_origin,
    float cabin_height,
    float cabin_speed
)
{
    nlohmann::json bind_path_json;
    bind_path_json["scan_mode"] = "scan_only";
    bind_path_json["cabin_height"] = cabin_height;
    bind_path_json["cabin_speed"] = cabin_speed;
    bind_path_json["path_origin"] = {
        {"x", path_origin.x},
        {"y", path_origin.y},
        {"z", cabin_height},
    };
    bind_path_json["areas"] = nlohmann::json::array();

    for (const auto& area_entry : area_entries) {
        nlohmann::json area_json;
        area_json["area_index"] = area_entry.area_index;
        area_json["cabin_pose"] = {
            {"x", area_entry.cabin_point.x},
            {"y", area_entry.cabin_point.y},
            {"z", cabin_height},
        };
        area_json["groups"] = nlohmann::json::array();
        for (const auto& bind_group : area_entry.bind_groups) {
            nlohmann::json group_json;
            group_json["group_index"] = bind_group.group_index;
            group_json["group_type"] = bind_group.group_type;
            group_json["points"] = nlohmann::json::array();
            int local_idx = 1;
            for (const auto& point : bind_group.bind_points_world) {
                auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
                const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
                const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
                const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
                const bool is_checkerboard_member =
                    checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
                group_json["points"].push_back(
                    {
                        {"idx", local_idx},
                        {"local_idx", local_idx},
                        {"global_idx", point.idx},
                        {"global_row", global_row},
                        {"global_col", global_col},
                        {"checkerboard_parity", checkerboard_parity},
                        {"is_checkerboard_member", is_checkerboard_member},
                        {"x", point.World_coord[0]},
                        {"y", point.World_coord[1]},
                        {"z", point.World_coord[2]},
                        {"world_x", point.World_coord[0]},
                        {"world_y", point.World_coord[1]},
                        {"world_z", point.World_coord[2]},
                        {"angle", point.Angle},
                    }
                );
                local_idx++;
            }
            area_json["groups"].push_back(group_json);
        }
        bind_path_json["areas"].push_back(area_json);
    }

    std::ofstream file_obj(pseudo_slam_bind_path_json_file);
    if (file_obj.is_open()) {
        file_obj << bind_path_json.dump(4);
    }
}


/*************************************************************************************************************************************************************
功能：驱动层函数，索驱移动位置转换为索驱接受命令字符数组，更新TCP_Move_Frame数组
输入：命令字 TCP套接字
输出：
**************************************************************************************************************************************************************/
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
// bool connectToServer() {
// struct sockaddr_in server_addr;
// socklen_t addrlen = sizeof(server_addr);

// sockfd = socket(AF_INET, SOCK_STREAM, 0);
// if (sockfd < 0) {
//     printCurrentTime();
//     printf("Cabin_Error: 创建套接字失败。\n");
//     return false;
// }

// //防止意外未关闭套接字，空出端口
// int opt = 1;
// if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) 
//     perror("设置 SO_REUSEADDR 失败");

// // 设置非阻塞模式
// fcntl(sockfd, F_SETFL, O_NONBLOCK);

// // 设置NODELAY模式关闭nagle算法
// int flag = 1;
// if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int)) < 0) {
//     printCurrentTime();
//     printf("Cabin_Error: 设置TCP_NODELAY失败。\n");
//     close(sockfd);
//     return false;
// }

// memset(&server_addr, 0, sizeof(server_addr));
// server_addr.sin_family = AF_INET;
// server_addr.sin_port = htons(2001);
// server_addr.sin_addr.s_addr = inet_addr("192.168.6.62");

// // 尝试连接
// int ret = connect(sockfd, (struct sockaddr *)&server_addr, addrlen);
// if (ret < 0) {
//     if (errno == EINPROGRESS) {
//         // 连接正在进行中，使用select等待连接
//         fd_set writefds;
//         FD_ZERO(&writefds);
//         FD_SET(sockfd, &writefds);

//         struct timeval timeout;
//         timeout.tv_sec = 5;  // 设置超时时间为5秒
//         timeout.tv_usec = 0;

//         // 使用 select 等待连接完成
//         ret = select(sockfd + 1, NULL, &writefds, NULL, &timeout);
//         if (ret <= 0) {
//             // 如果 select 返回 0 或负数，则认为连接超时
//             printCurrentTime();
//             printf("Cabin_Error: TCP连接超时。\n");
//             close(sockfd);
//             return false;
//         } else if (FD_ISSET(sockfd, &writefds)) {
//             // 连接成功，检查是否发生了错误
//             int error;
//             socklen_t len = sizeof(error);
//             if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
//                 printCurrentTime();
//                 perror("Cabin_Error: 获取连接错误状态失败。\n");
//                 close(sockfd);
//                 return false;
//             }
//             if (error != 0) {
//                 printCurrentTime();
//                 printf("Cabin_Error: 连接错误 %d\n", error);
//                 close(sockfd);
//                 return false;
//             }
//         }
//     } else {
//         // 其他错误
//         printCurrentTime();
//         printf("Cabin_Error: TCP连接失败。\n");
//         close(sockfd);
//         return false;
//     }
// }
// // 连接成功
// printCurrentTime();
// printf("Cabin_log: TCP连接成功。\n");
// // 在连接成功后，将套接字设回阻塞模式
// fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) & ~O_NONBLOCK);
// return true;
// }

bool connectToServer()
{
    const uint16_t SERVER_PORT = 2001;
    const char* SERVER_IP = "192.168.6.62";

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printCurrentTime();
        printf("Cabin_Error: 创建套接字失败。\n");
        return false;
    }

    // === 1. 设置 socket 选项 ===
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    timeval rw_timeout{TCP_TIMEOUT_SEC, 0};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &rw_timeout, sizeof(rw_timeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rw_timeout, sizeof(rw_timeout));

    // === 2. 设置为非阻塞 ===
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // === 3. 发起非阻塞 connect ===
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    int ret = connect(sock, (sockaddr*)&server_addr, sizeof(server_addr));
    if (ret < 0 && errno != EINPROGRESS) {
        printCurrentTime();
        printf("Cabin_Error: TCP连接失败。\n");
        close(sock);
        return false;
    }

    // === 4. select 等待连接完成 ===
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sock, &writefds);

    timeval timeout{TCP_TIMEOUT_SEC, 0};
    ret = select(sock + 1, nullptr, &writefds, nullptr, &timeout);

    if (ret <= 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP连接超时。\n");
        close(sock);
        return false;
    }

    // === 5. 检查 socket 错误状态 ===
    int error = 0;
    socklen_t len = sizeof(error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);

    if (error != 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP连接错误 %d\n", error);
        close(sock);
        return false;
    }

    // === 6. 连接成功，切回阻塞模式 ===
    fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);

    sockfd = sock;  // ✅ 全局 socket 赋值

    printCurrentTime();
    printf("Cabin_log: TCP连接成功。\n");
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
//         printf("Cabin_Error: 无法向write中使用select方法。\n");
//         return -1; // 错误
//     } else if (ret == 0) {
//         printCurrentTime();
//         printf("Cabin_Error: TCP写入超时。\n");
//         return -1; // 超时
//     } else {
//         if (FD_ISSET(socket, &writefds)) {
//             ssize_t send_len = send(socket, Control_Word, Tlen,0);
//             if (send_len < 0) {
//                 printCurrentTime();
//                 printf("Cabin_Error: TCP指令写入失败。\n");
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
//         printf("Cabin_Error: 无法向read中使用select方法。\n");
//         return -1; // 错误
//     } else if (ret == 0) {
//         printCurrentTime();
//         printf("Cabin_Error: TCP读取超时。\n");
//         return -1; // 超时
//     } else {
//         if (FD_ISSET(socket, &readfds)) {
//             ssize_t recv_len = recv(socket, buffer, Rlen,0);
//             if (recv_len < 0) {
//                 printCurrentTime();
//                 printf("Cabin_Error: TCP反馈读取失败。\n");
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
        printf("Cabin_Error: Invalid input parameters.\n");
        return -1;
    }

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
        printCurrentTime();
        printf("Cabin_Error: TCP write select failed: %s\n", strerror(errno));
        return -1;
    }
    if (ret == 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP write select timeout after %d seconds.\n", TCP_TIMEOUT_SEC);
        return -1;
    }

    // 3. 循环发送，确保所有数据发出
    ssize_t total_sent = 0;
    while (total_sent < Tlen) {
        ssize_t sent = send(socket, Control_Word + total_sent, Tlen - total_sent, 0);
        if (sent == 0) {
            printCurrentTime();
            printf("Cabin_Error: TCP connection closed while sending command.\n");
            return -1;
        }
        if (sent < 0) {
            printCurrentTime();
            printf("Cabin_Error: TCP send failed: %s\n", strerror(errno));
            return -1;
        }
        total_sent += sent;
    }
    if (!is_heartbeat_frame) {
        printCurrentTime();
        printFrameBytes("Cabin_log: TCP sent frame: ", Control_Word, static_cast<size_t>(Tlen));
    }

    // 4. 写后延时（根据设备协议决定是否需要）
    std::this_thread::sleep_for(std::chrono::milliseconds(WRITE_DELAY_MS));

    // 5. 读操作带超时控制
    uint8_t buffer[RECV_BUFFER_SIZE];
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket, &readfds);

    timeout.tv_sec = TCP_TIMEOUT_SEC;
    timeout.tv_usec = 0;
    ret = select(socket + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP read select failed: %s\n", strerror(errno));
        return -1;
    }
    if (ret == 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP read select timeout after %d seconds.\n", TCP_TIMEOUT_SEC);
        return -1;
    }

    // 6. 循环接收或限制最大接收长度
    ssize_t recv_len = recv(socket, buffer, sizeof(buffer) - 1, 0); // 预留1字节给'\0'
    if (recv_len == 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP connection closed by peer.\n");
        return -1;
    }
    if (recv_len < 0) {
        printCurrentTime();
        printf("Cabin_Error: TCP recv failed: %s\n", strerror(errno));
        return -1;
    }
    
    // 安全添加字符串结束符
    buffer[recv_len] = '\0';
    if (!is_heartbeat_frame) {
        printCurrentTime();
        printFrameBytes("Cabin_log: TCP recv frame: ", buffer, static_cast<size_t>(recv_len));
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
    for(int i = 0; i < 5; i++){
        if (Frame_Generate(Control_Word, Tlen, Rlen,socket) < 0) 
        {
            printCurrentTime();
            printf("Cabin_Error:第%d次发送指令失败。\n",i+1);
            if(reconnect_flag == false)
            {
                reconnect_flag = true;
                printCurrentTime();
                printf("Cabin_Error:正在尝试与索驱上位机重新创建TCP连接。\n");
                for(; j < 5; j++)
                {
                    if (!connectToServer()) 
                    {
                        printCurrentTime();
                        printf("Cabin_Error:重新连接失败，正在尝试第%d次重新连接。\n",j+1);
                    }
                    else
                    {
                        printCurrentTime();
                        printf("Cabin_Error:重新连接成功，正在尝试重新发送指令。\n");
                        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 延时0.2秒重试
                        j = 0;
                        break;
                    }
                }
                if(j == 5)
                {
                    printCurrentTime();
                    printf("Cabin_Error:重新连接失败超过5次，可能存在网络问题，无法控制索驱系统，正在紧急退出程序。\n");
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
            return 0; // 成功发送命令后退出函数
        }
    }     
    printCurrentTime();
    printf("Cabin_Error:重新发送命令失败超过5次，可能无法控制索驱系统，正在紧急退出程序。\n");
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
            printf("Cabin_log:索驱暂停中断恢复,重新发送目标位置。\n");
            
            // 重新生成运动指令
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                moveTCPPosition(0x01, TCP_Move);  // 假设这个函数用于更新TCP_Move_Frame
                Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8); //恢复索驱继续运动
            }
        }
    }
    return;
}
/*************************************************************************************************************************************************************
功能: 驱动层函数，延时，等待索驱某个轴运动到指定位置
输入: 轴 位置
输出:
**************************************************************************************************************************************************************/
void delay_time(int Axis, double Target_position)
{
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;

    while (1)
    {
        
        // printf("cabin_state - X: %.2f, Y: %.2f, Z: %.2f, motion_status: %.2d, Target traget_coordinate: %.2f\n, Axis: %d\n",
        // cabin_state.X, cabin_state.Y, cabin_state.Z, cabin_state.motion_status,Target_position,Axis);
        {
            // 如果当前轴体的坐标位置等于目标位置,且索驱完全停下来时运动结束。
            std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        // float cur_pos_x = cabin_state.X;
        // float cur_pos_y = cabin_state.Y;
        // getSuoquTestTimeTxt(cur_pos_y);
            if(Axis == AXIS_X && abs(cabin_state.X - Target_position)<25 && cabin_state.motion_status == 0)
                break;
            if(Axis == AXIS_Y && abs(cabin_state.Y - Target_position)<25 && cabin_state.motion_status == 0)
                break;
            if(Axis == AXIS_Z && abs(cabin_state.Z - Target_position)<25 && cabin_state.motion_status == 0)
                break;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_sec >= DELAY_TIME_TIMEOUT_SEC) {
            printCurrentTime();
            printf("Cabin_Error: 等待轴%d到位超时，目标位置 %.2f。\n", Axis, Target_position);
            return;
        }

        const auto log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (log_elapsed_sec >= DELAY_TIME_LOG_INTERVAL_SEC) {
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
            printf(
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

    return;
}

// 规划路径前下发速度
void change_cabin_speed_callback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    printf("Cabin_log: 修改索驱速度指令，速度为: %.2f\n", debug_mes.data);
    global_cabin_speed = debug_mes.data;
    return;
}

void checkerboard_jump_bind_callback(const std_msgs::Bool &debug_mes)
{
    checkerboard_jump_bind_enabled = debug_mes.data;
    printCurrentTime();
    if (checkerboard_jump_bind_enabled) {
        printf("Cabin_log: 全局棋盘格跳绑已开启，仅执行checkerboard_parity==0的点。\n");
    } else {
        printf("Cabin_log: 全局棋盘格跳绑已关闭，恢复全绑。\n");
    }
}

void global_execution_mode_callback(const std_msgs::Float32 &debug_mes)
{
    const GlobalExecutionMode mode =
        debug_mes.data >= 1.0f ? GlobalExecutionMode::kLiveVisual : GlobalExecutionMode::kSlamPrecomputed;
    set_global_execution_mode(mode);
    printCurrentTime();
    printf(
        "Cabin_log: 全局执行模式已切换为%s。\n",
        global_execution_mode_name(mode)
    );
}

// 路径规划
bool planGlobalMovePath(chassis_ctrl::Pathguihua::Request &req, chassis_ctrl::Pathguihua::Response &res)
{
    float mx = req.marking_x;
    float my = req.marking_y;
    float zxl = req.zone_x;
    float zyl = req.zone_y;
    float rxs = req.robot_x_step;
    float rys = req.robot_y_step;
    float cabin_height = req.height;
    float cabin_speed = global_cabin_speed;
    printCurrentTime();
    printf("Cabin_log: 当前机器人起点位置为：(%f,%f,%f)\n",mx,my,cabin_height);
    // 生成路径点
    new_path = path_point_generate(mx, my, zxl, zyl, rxs, rys, cabin_height,cabin_speed);
    // 可视化路径 （待开发）
    res.success = true;
    res.message = "路径成功生成";
    printCurrentTime();
    printf("Cabin_log: 新路径配置生成，包含 %zu 个路径点\n", new_path.size());
    return res.success;
}


void moduan_work_Callback(const std_msgs::Bool &debug_mes)
{
    moduan_work_flag = debug_mes.data;
}
/*
    函数功能：暂停中断的回调函数,收到暂停中断信号后，设置暂停中断标志位为1，等待人工恢复。
*/
void pause_interrupt_Callback(const std_msgs::Float32 &debug_mes)
{
    if (debug_mes.data == 1.0 && !moduan_work_flag)
    {
        printCurrentTime();
        printf("Cabin_log: 索驱人工暂停，正在等待人工恢复。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 1;
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                Frame_Generate_With_Retry(TCP_stop, 8, 8); //先暂停索驱运动
            }
        }
    }
    else
    {
        printCurrentTime();
        printf("Cabin_log: 绑扎正在进行，不可暂停索驱。\n");
    }

    return;
}

void solve_stop_Callback(const std_msgs::Float32 &debug_mes)
{
    if (debug_mes.data == 1.0 && !moduan_work_flag)
    {
        printCurrentTime();
        printf("Cabin_log: 恢复索驱运动，重置暂停标志。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 0;
        }
    }
    else
    {
        printCurrentTime();
        printf("Cabin_log: 绑扎正在进行，不可恢复索驱中断。\n");
    }
    return;
}

/*
    函数功能：急停的回调函数,收到急停信号后，强制停止索驱运动，关闭节点。
*/
void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    printf("Cabin_log:急停信号已被触发，正在强制停止索驱运动，关闭节点。\n");
    
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        Frame_Generate_With_Retry(TCP_stop, 8, 8);
        close(sockfd);
    }
    ros::shutdown();
}
/*
    函数功能：索驱单点运动的回调函数
*/
bool cabin_single_move(chassis_ctrl::SingleMove::Request& req, chassis_ctrl::SingleMove::Response& res)
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
        printf("Cabin_Error: 索驱Z轴距离设置超限。\n");
        res.message = "Cabin_log: 索驱单点运动失败（Z轴距离设置超限），位置为(" + std::to_string(cabin_x) + "," + std::to_string(cabin_y) + "," + std::to_string(cabin_height) + ")。";
        res.success = false;
        return res.success;
    }
// getSuoquTestTimeTxt(cabin_y);
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01,TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
// getSuoquTestTimeTxt(cabin_y);
    delay_time(AXIS_X,cabin_x);
    delay_time(AXIS_Y,cabin_y);
    delay_time(AXIS_Z,cabin_height);

    printCurrentTime();
    printf("Cabin_log: 索驱单点运动完成，位置为(%lf,%lf,%lf)。\n",cabin_x,cabin_y,cabin_height);
    res.message = "Cabin_log: 索驱单点运动完成，位置为(" + std::to_string(cabin_x) + "," + std::to_string(cabin_y) + "," + std::to_string(cabin_height) + ")。";
    res.success = true;
    return res.success;
}

/*
    函数功能：索驱单轴运动
// */
// void atom_cabin_move_single(const chassis_ctrl::cabin_move_single::ConstPtr& msg)
// {
//     char C;
//     if(msg->Axis ==AXIS_X && (msg->target_distance < -1500 || msg->target_distance > 1500))
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱X轴距离设3置超限。\n");
//         return ;
//     }
//     if(msg->Axis ==AXIS_Y && (msg->target_distance < -3000 || msg->target_distance > 3000))
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱Y轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->Axis ==AXIS_Z && (msg->target_distance < -60 || msg->target_distance > 1200))
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱Z轴距离设置超限。\n");
//         return ;
//     }
    
//     {
//         std::lock_guard<std::mutex> lock1(cabin_state_mutex);
//         TCP_Move[1] = cabin_state.X;
//         TCP_Move[2] = cabin_state.Y;
//         TCP_Move[3] = cabin_state.Z;
//     }

//     printCurrentTime();
//     printf("Cabin_log:索驱单轴运动请求已被触发，");
//     if(msg->Axis == AXIS_X)
//     {
//         printf("X轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[1] = msg->target_distance;
//         C= 'X';
//     }
//     if(msg->Axis == AXIS_Y)
//     {
//         printf("Y轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[2] = msg->target_distance;
//         C= 'Y';
//     }
//     if(msg->Axis == AXIS_Z)
//     {
//         printf("Z轴正在移动至%lfmm处。\n",msg->target_distance);
//         TCP_Move[3] = msg->target_distance;
//         C= 'Z';
//     }
//     else
//     {
//         printf("轴体参数不正确。\n");
//         return ;
//     }
//     // 移动索驱单轴
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     delay_time(msg->Axis,msg->target_distance);
//     printCurrentTime();
//     printf("Cabin_log:索驱%C轴已移动至目标位置%lfmm处。\n",C,msg->target_distance);
//     return ;
// }

/*
    函数功能：索驱三轴运动调试
*/
// void atom_cabin_move_all(const chassis_ctrl::cabin_move_all::ConstPtr& msg)
// {
//     if(msg->cabin_X_distance < -1500 || msg->cabin_X_distance > 1500)
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱X轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->cabin_Y_distance < -3000 || msg->cabin_Y_distance > 3000)
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱Y轴距离设置超限。\n");
//         return ;
//     }
//     if(msg->cabin_Z_distance < -60 || msg->cabin_Z_distance > 1200)
//     {
//         printCurrentTime();
//         printf("Cabin_Error:索驱Z轴距离设置超限。\n");
//         return ;
//     }
    
//     printCurrentTime();
//     printf("Cabin_log:索驱正在移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
    
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
//     delay_time(AXIS_Z,0);
//     // 当Z轴回到原点后，再移动X轴和Y轴
//     TCP_Move[1] = msg->cabin_X_distance;
//     TCP_Move[2] = msg->cabin_Y_distance;
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     delay_time(AXIS_X,msg->cabin_X_distance);
//     delay_time(AXIS_Y,msg->cabin_Y_distance);
//     // 当X,Y轴抵达位置后，再移动Z轴运动
//     TCP_Move[3] = msg->cabin_Z_distance;
//     {
//         std::lock_guard<std::mutex> lock2(socket_mutex);
//         moveTCPPosition(0x01,TCP_Move);
//         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
//     }
//     delay_time(AXIS_Z,msg->cabin_Z_distance);
//     printCurrentTime();
//     printf("Cabin_log:索驱已移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
//     return;
// }

// 订阅外部姿态传感器计算的索驱姿态
void cabin_gesture_get(const chassis_ctrl::linear_module_upload& msg)
{
    cabin_state.cabin_x_gesture = msg.x_gesture;
    cabin_state.cabin_y_gesture = msg.y_gesture;
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
        printf("Cabin_log: 使用path_points.json中的历史路径。\n");
        for (const auto& json_point : path_json["path_points"]) {
            Cabin_Point point;
            point.x = json_point["x"].get<float>();
            point.y = json_point["y"].get<float>();
            con_path.push_back(point);
        }
    }
    return !con_path.empty();
}

fast_image_solve::PointCoords build_world_point_from_scan_response(
    const fast_image_solve::PointCoords& point,
    int point_index
)
{
    fast_image_solve::PointCoords world_point = point;
    world_point.idx = point_index;
    return world_point;
}

void assign_global_indices(std::vector<fast_image_solve::PointCoords>& world_points)
{
    int global_index = 1;
    for (auto& world_point : world_points) {
        world_point.idx = global_index++;
    }
}

bool run_pseudo_slam_scan(std::vector<Cabin_Point> con_path, float cabin_height, float cabin_speed, std::string& message)
{
    std::vector<fast_image_solve::PointCoords> merged_world_points;
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
    const Cabin_Point path_origin = con_path.front();

    int area_index = 0;
    for (const auto& cabin_point : con_path) {
        area_index++;
        publish_area_progress(area_index, total_area_count, 0, false, false);
        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_point.x;
        TCP_Move[2] = cabin_point.y;
        TCP_Move[3] = cabin_height;
        printCurrentTime();
        printf("Cabin_log: pseudo_slam scan_only区域%d移动到(%f,%f,%f)。\n", area_index, TCP_Move[1], TCP_Move[2], TCP_Move[3]);
        {
            std::lock_guard<std::mutex> lock2(socket_mutex);
            moveTCPPosition(0x01, TCP_Move);
            Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
        }
        delay_time(AXIS_X, cabin_point.x);
        delay_time(AXIS_Y, cabin_point.y);
        delay_time(AXIS_Z, cabin_height);
        if (!wait_for_pseudo_slam_capture_gate(area_index, cabin_point, cabin_height)) {
            printCurrentTime();
            printf("Cabin_Warn: pseudo_slam scan_only区域%d等待最终采集门时ROS关闭，结束扫描。\n", area_index);
            message = "扫描建图在等待最终采集门时被中断";
            return false;
        }

        while (ros::ok()) {
            const PseudoSlamCaptureGateConfig config = load_pseudo_slam_capture_gate_config();
            srv.request.request_mode = kProcessImageModeScanOnly;
            if (!AI_client.call(srv) || !srv.response.success) {
                printCurrentTime();
                printf(
                    "Cabin_Warn: pseudo_slam scan_only区域%d视觉失败，跳过当前区域，消息：%s\n",
                    area_index,
                    srv.response.message.c_str()
                );
                break;
            }

            if (static_cast<int>(srv.response.PointCoordinatesArray.size()) < config.scan_min_point_count) {
                printCurrentTime();
                printf(
                    "Cabin_Warn: pseudo_slam scan_only区域%d白色矩形内点数%d<%d，继续轮询视觉。\n",
                    area_index,
                    static_cast<int>(srv.response.PointCoordinatesArray.size()),
                    config.scan_min_point_count
                );
                ros::Duration(config.scan_retry_interval_sec).sleep();
                continue;
            }
            break;
        }
        if (!ros::ok()) {
            message = "扫描建图在视觉轮询阶段被中断";
            return false;
        }
        if (!srv.response.success) {
            continue;
        }

        std::vector<fast_image_solve::PointCoords> area_world_points;
        int point_index = 1;
        for (const auto& point : srv.response.PointCoordinatesArray) {
            area_world_points.push_back(build_world_point_from_scan_response(point, point_index++));
        }
        area_world_points = dedupe_world_points(area_world_points);
        merged_world_points.insert(merged_world_points.end(), area_world_points.begin(), area_world_points.end());
        merged_world_points = dedupe_world_points(merged_world_points);
        assign_global_indices(merged_world_points);
        set_pseudo_slam_tf_points(merged_world_points);
        publish_pseudo_slam_markers(merged_world_points);

        printCurrentTime();
        printf(
            "Cabin_log: pseudo_slam区域%d识别完成，世界点%d个，当前累计世界点%d个。\n",
            area_index,
            static_cast<int>(area_world_points.size()),
            static_cast<int>(merged_world_points.size())
        );
    }

    assign_global_indices(merged_world_points);
    set_pseudo_slam_tf_points(merged_world_points);
    publish_pseudo_slam_markers(merged_world_points);
    const std::unordered_map<int, PseudoSlamCheckerboardInfo> merged_checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(merged_world_points, path_origin);
    std::vector<fast_image_solve::PointCoords> planning_world_points = filter_pseudo_slam_planning_outliers(merged_world_points);
    std::unordered_map<int, PseudoSlamCheckerboardInfo> checkerboard_info_by_idx =
        build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    planning_world_points = filter_pseudo_slam_non_checkerboard_points(
        planning_world_points,
        checkerboard_info_by_idx
    );
    checkerboard_info_by_idx = build_checkerboard_info_by_global_index(planning_world_points, path_origin);
    std::vector<fast_image_solve::PointCoords> remaining_world_points = planning_world_points;
    std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries;
    int bind_group_count = 0;
    int bind_point_count = 0;
    int grouped_area_count = 0;
    int grouped_area_index = 0;
    for (const auto& cabin_point : con_path) {
        grouped_area_index++;
        std::vector<PseudoSlamBindGroup> bind_groups = build_bind_groups_from_scan_world(
            remaining_world_points,
            cabin_point,
            cabin_height
        );
        if (bind_groups.empty()) {
            continue;
        }

        grouped_area_count++;
        for (const auto& bind_group : bind_groups) {
            bind_group_count++;
            bind_point_count += static_cast<int>(bind_group.bind_points_world.size());
        }
        bind_area_entries.push_back(
            {
                grouped_area_index,
                cabin_point,
                bind_groups
            }
        );
    }

    write_pseudo_slam_points_json(merged_world_points, merged_checkerboard_info_by_idx);
    write_pseudo_slam_bind_path_json(bind_area_entries, checkerboard_info_by_idx, path_origin, cabin_height, cabin_speed);
    const std::string scan_session_id = std::to_string(ros::Time::now().toNSec());
    BindExecutionMemory bind_execution_memory =
        reset_bind_execution_memory_for_scan_session(scan_session_id, path_origin);
    std::string bind_execution_memory_error;
    if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
        printCurrentTime();
        printf(
            "Cabin_log: 扫描完成后重置bind_execution_memory.json失败：%s\n",
            bind_execution_memory_error.c_str()
        );
    }
    publish_area_progress(total_area_count, total_area_count, total_area_count, false, true);
    std::ostringstream oss;
    oss << "扫描建图完成，pseudo_slam_points.json=" << merged_world_points.size()
        << "个点，pseudo_slam_bind_path.json=" << grouped_area_count
        << "个区域/" << bind_group_count << "个分组/" << bind_point_count << "个绑扎点";
    message = oss.str();
    return true;
}

std::vector<fast_image_solve::PointCoords> load_bind_points_from_group_json(
    const nlohmann::json& group_json
)
{
    std::vector<fast_image_solve::PointCoords> points;
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        return points;
    }

    int point_index = 1;
    for (const auto& point_json : group_json["points"]) {
        fast_image_solve::PointCoords point;
        point.idx = point_json.value("local_idx", point_json.value("idx", point_index));
        point.Pix_coord[0] = 0;
        point.Pix_coord[1] = 0;
        point.World_coord[0] = point_json.value("x", 0.0f);
        point.World_coord[1] = point_json.value("y", 0.0f);
        point.World_coord[2] = point_json.value("z", 0.0f);
        point.Angle = point_json.value("angle", -45.0f);
        point.is_shuiguan = false;
        points.push_back(point);
        point_index++;
    }
    return points;
}

std::vector<fast_image_solve::PointCoords> convert_bind_world_points_to_gripper_points(
    const std::vector<fast_image_solve::PointCoords>& world_points
)
{
    std::vector<fast_image_solve::PointCoords> local_points;
    for (const auto& world_point : world_points) {
        fast_image_solve::PointCoords local_point;
        if (!transform_cabin_world_point_to_gripper_point(world_point, local_point)) {
            continue;
        }
        if (!is_local_bind_point_in_range(local_point)) {
            continue;
        }
        local_point.idx = world_point.idx;
        local_points.push_back(local_point);
    }
    return local_points;
}

std::vector<fast_image_solve::PointCoords> convert_bind_world_points_to_xy_in_range_gripper_points(
    const std::vector<fast_image_solve::PointCoords>& world_points
)
{
    std::vector<fast_image_solve::PointCoords> local_points;
    for (const auto& world_point : world_points) {
        fast_image_solve::PointCoords local_point;
        if (!transform_cabin_world_point_to_gripper_point(world_point, local_point)) {
            continue;
        }
        if (!is_local_bind_point_in_range(local_point)) {
            continue;
        }
        local_point.idx = world_point.idx;
        local_points.push_back(local_point);
    }
    return local_points;
}

bool prepare_precomputed_bind_group_for_execution(
    const std::vector<fast_image_solve::PointCoords>& world_points,
    std::vector<fast_image_solve::PointCoords>& local_points,
    std::string& failure_reason
)
{
    local_points.clear();
    failure_reason.clear();
    if (world_points.empty()) {
        failure_reason = "当前组无预计算世界点";
        return false;
    }

    local_points = convert_bind_world_points_to_xy_in_range_gripper_points(world_points);
    if (local_points.empty()) {
        failure_reason = "当前组转换到虎口后无x/y工作区内点";
        return false;
    }
    return true;
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

float get_current_area_bind_test_cabin_speed(float configured_cabin_speed)
{
    const float base_speed = configured_cabin_speed > 0.0f ? configured_cabin_speed : global_cabin_speed;
    return std::max(
        base_speed * kCurrentAreaBindTestCabinSpeedMultiplier,
        kCurrentAreaBindTestMinCabinSpeedMmPerSec
    );
}

bool run_current_area_bind_from_scan_test(std::string& message)
{
    std::ifstream infile(pseudo_slam_bind_path_json_file);
    if (!infile.is_open()) {
        message = "未找到pseudo_slam_bind_path.json，请先执行扫描建图";
        return false;
    }

    nlohmann::json bind_path_json;
    infile >> bind_path_json;
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
    const float fast_cabin_speed = get_current_area_bind_test_cabin_speed(
        bind_path_json.value("cabin_speed", global_cabin_speed)
    );

    publish_area_progress(area_index, total_area_count, 0, false, false);
    printCurrentTime();
    printf(
        "Cabin_log: 当前区域预计算直执行选择区域%d，当前位置(%f,%f)，目标区域(%f,%f,%f)，距离%.1fmm，索驱快速度%.1f。\n",
        area_index,
        current_cabin_x,
        current_cabin_y,
        cabin_x,
        cabin_y,
        cabin_height,
        nearest_distance_mm,
        fast_cabin_speed
    );

    TCP_Move[0] = fast_cabin_speed;
    TCP_Move[1] = cabin_x;
    TCP_Move[2] = cabin_y;
    TCP_Move[3] = cabin_height;
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01, TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(AXIS_X, cabin_x);
    delay_time(AXIS_Y, cabin_y);
    delay_time(AXIS_Z, cabin_height);

    if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
        message = "当前区域缺少groups，无法执行预计算绑扎测试";
        return false;
    }

    BindExecutionMemory bind_execution_memory = load_bind_execution_memory_json();
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
            checkerboard_jump_bind_enabled
        );
        const std::vector<fast_image_solve::PointCoords> world_points =
            load_bind_points_from_group_json(execution_group_json);
        if (world_points.empty()) {
            skipped_group_count++;
            printCurrentTime();
            printf(
                "Cabin_log: 当前区域预计算直执行第%d组(%s)经棋盘格/执行记忆过滤后无可执行点，跳过当前组。\n",
                group_index,
                group_type.c_str()
            );
            continue;
        }

        std::vector<fast_image_solve::PointCoords> local_points;
        std::string prepare_failure_reason;
        if (!prepare_precomputed_bind_group_for_execution(
                world_points,
                local_points,
                prepare_failure_reason
            )) {
            skipped_group_count++;
            printCurrentTime();
            printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组无法满足虎口执行条件，跳过当前组。原因：%s\n",
                group_index,
                prepare_failure_reason.c_str()
            );
            continue;
        }

        chassis_ctrl::ExecuteBindPoints bind_srv;
        bind_srv.request.points = local_points;
        if (!sg_precomputed_fast_client.call(bind_srv) || !bind_srv.response.success) {
            skipped_group_count++;
            printCurrentTime();
            printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组失败，跳过当前组，消息：%s\n",
                group_index,
                bind_srv.response.message.c_str()
            );
            continue;
        }

        for (const auto& point_json : execution_group_json["points"]) {
            record_successful_execution_point(bind_execution_memory, point_json);
        }
        std::string bind_execution_memory_error;
        if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
            printCurrentTime();
            printf(
                "Cabin_Warn: 当前区域预计算直执行第%d组成功后写入bind_execution_memory.json失败：%s\n",
                group_index,
                bind_execution_memory_error.c_str()
            );
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

bool run_live_visual_global_work(std::string& message)
{
    std::vector<Cabin_Point> con_path;
    float cabin_height = 0.0f;
    float cabin_speed = 0.0f;
    if (!load_configured_path(con_path, cabin_height, cabin_speed)) {
        message = "无法加载全局执行路径";
        return false;
    }

    const int total_area_count = static_cast<int>(con_path.size());
    int executed_area_count = 0;
    int skipped_area_count = 0;

    for (int area_index = 0; area_index < total_area_count; ++area_index) {
        const Cabin_Point& cabin_point = con_path[area_index];
        publish_area_progress(area_index + 1, total_area_count, 0, false, false);

        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_point.x;
        TCP_Move[2] = cabin_point.y;
        TCP_Move[3] = cabin_height;
        printCurrentTime();
        printf(
            "Cabin_log: live_visual区域%d移动到(%f,%f,%f)。\n",
            area_index + 1,
            TCP_Move[1],
            TCP_Move[2],
            TCP_Move[3]
        );
        {
            std::lock_guard<std::mutex> lock2(socket_mutex);
            moveTCPPosition(0x01, TCP_Move);
            Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
        }
        delay_time(AXIS_X, cabin_point.x);
        delay_time(AXIS_Y, cabin_point.y);
        delay_time(AXIS_Z, cabin_height);

        std_srvs::Trigger bind_srv;
        if (!sg_live_visual_client.call(bind_srv)) {
            skipped_area_count++;
            printCurrentTime();
            printf(
                "Cabin_Warn: live_visual区域%d调用/moduan/sg失败，跳过当前区域。\n",
                area_index + 1
            );
            publish_area_progress(
                area_index + 1 < total_area_count ? area_index + 2 : area_index + 1,
                total_area_count,
                area_index + 1,
                true,
                false
            );
            continue;
        }

        if (!bind_srv.response.success) {
            skipped_area_count++;
            printCurrentTime();
            printf(
                "Cabin_Warn: live_visual区域%d执行失败，跳过当前区域。消息：%s\n",
                area_index + 1,
                bind_srv.response.message.c_str()
            );
            publish_area_progress(
                area_index + 1 < total_area_count ? area_index + 2 : area_index + 1,
                total_area_count,
                area_index + 1,
                true,
                false
            );
            continue;
        }

        executed_area_count++;
        publish_area_progress(
            area_index + 1 < total_area_count ? area_index + 2 : area_index + 1,
            total_area_count,
            area_index + 1,
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
    std::ifstream infile(pseudo_slam_bind_path_json_file);
    if (!infile.is_open()) {
        message = "未找到pseudo_slam_bind_path.json，请先执行扫描建图";
        return false;
    }

    nlohmann::json bind_path_json;
    infile >> bind_path_json;
    if (!bind_path_json.contains("areas") || !bind_path_json["areas"].is_array()) {
        message = "pseudo_slam_bind_path.json格式错误";
        return false;
    }

    const float cabin_height = bind_path_json.value("cabin_height", 500.0f);
    const float cabin_speed = bind_path_json.value("cabin_speed", global_cabin_speed);
    const auto& areas_json = bind_path_json["areas"];
    if (areas_json.empty()) {
        message = "pseudo_slam_bind_path.json没有可执行区域，请先确认扫描分组成功";
        return false;
    }
    const int total_area_count = static_cast<int>(areas_json.size());

    float path_origin_x = 0.0f;
    float path_origin_y = 0.0f;
    float path_origin_z = cabin_height;
    if (bind_path_json.contains("path_origin") && bind_path_json["path_origin"].is_object()) {
        path_origin_x = bind_path_json["path_origin"]["x"].get<float>();
        path_origin_y = bind_path_json["path_origin"]["y"].get<float>();
        path_origin_z = bind_path_json["path_origin"]["z"].get<float>();
    } else {
        const auto& first_cabin_pose = areas_json.front()["cabin_pose"];
        path_origin_x = first_cabin_pose.value("x", 0.0f);
        path_origin_y = first_cabin_pose.value("y", 0.0f);
        path_origin_z = first_cabin_pose.value("z", cabin_height);
    }

    TCP_Move[0] = cabin_speed;
    TCP_Move[1] = path_origin_x;
    TCP_Move[2] = path_origin_y;
    TCP_Move[3] = path_origin_z;
    printCurrentTime();
    printf(
        "Cabin_log: bind_from_scan先回到规划原点(%f,%f,%f)。\n",
        TCP_Move[1],
        TCP_Move[2],
        TCP_Move[3]
    );
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01, TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(AXIS_X, path_origin_x);
    delay_time(AXIS_Y, path_origin_y);
    delay_time(AXIS_Z, path_origin_z);

    int area_index = 0;
    BindExecutionMemory bind_execution_memory = load_bind_execution_memory_json();
    for (const auto& area_json : areas_json) {
        area_index++;
        publish_area_progress(area_index, total_area_count, 0, false, false);
        const auto cabin_pose = area_json["cabin_pose"];
        const float cabin_x = cabin_pose.value("x", 0.0f);
        const float cabin_y = cabin_pose.value("y", 0.0f);
        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = cabin_x;
        TCP_Move[2] = cabin_y;
        TCP_Move[3] = cabin_height;
        printCurrentTime();
        printf("Cabin_log: bind_from_scan区域%d移动到(%f,%f,%f)。\n", area_index, TCP_Move[1], TCP_Move[2], TCP_Move[3]);
        {
            std::lock_guard<std::mutex> lock2(socket_mutex);
            moveTCPPosition(0x01, TCP_Move);
            Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
        }
        delay_time(AXIS_X, cabin_x);
        delay_time(AXIS_Y, cabin_y);
        delay_time(AXIS_Z, cabin_height);

        if (!area_json.contains("groups") || !area_json["groups"].is_array()) {
            printCurrentTime();
            printf("Cabin_Warn: bind_from_scan区域%d缺少groups，跳过当前区域。\n", area_index);
            continue;
        }

        int group_index = 0;
        for (const auto& group_json : area_json["groups"]) {
            group_index++;
            const std::string group_type = group_json.value("group_type", std::string("unknown_group"));
            nlohmann::json execution_group_json = group_json;
            execution_group_json["points"] = filter_precomputed_group_points_for_execution(
                group_json,
                bind_execution_memory,
                checkerboard_jump_bind_enabled
            );
            const std::vector<fast_image_solve::PointCoords> world_points =
                load_bind_points_from_group_json(execution_group_json);
            if (world_points.empty()) {
                printCurrentTime();
                printf(
                    "Cabin_log: bind_from_scan区域%d第%d组(%s)经棋盘格/执行记忆过滤后无可执行点，跳过当前组。\n",
                    area_index,
                    group_index,
                    group_type.c_str()
                );
                continue;
            }
            std::vector<fast_image_solve::PointCoords> local_points;
            std::string prepare_failure_reason;
            if (!prepare_precomputed_bind_group_for_execution(
                    world_points,
                    local_points,
                    prepare_failure_reason
                )) {
                printCurrentTime();
                printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组无法满足虎口执行条件，跳过当前组。原因：%s\n",
                    area_index,
                    group_index,
                    prepare_failure_reason.c_str()
                );
                continue;
            }
            chassis_ctrl::ExecuteBindPoints bind_srv;
            bind_srv.request.points = local_points;
            if (!sg_precomputed_client.call(bind_srv) || !bind_srv.response.success) {
                printCurrentTime();
                printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组执行失败，跳过该点所在组，跳过该组继续下一组，消息：%s\n",
                    area_index,
                    group_index,
                    bind_srv.response.message.c_str()
                );
                continue;
            }

            for (const auto& point_json : execution_group_json["points"]) {
                record_successful_execution_point(bind_execution_memory, point_json);
            }
            std::string bind_execution_memory_error;
            if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
                printCurrentTime();
                printf(
                    "Cabin_Warn: bind_from_scan区域%d第%d组成功后写入bind_execution_memory.json失败：%s\n",
                    area_index,
                    group_index,
                    bind_execution_memory_error.c_str()
                );
            }
        }
        publish_area_progress(area_index < total_area_count ? area_index + 1 : area_index, total_area_count, area_index, true, false);
    }

    publish_area_progress(total_area_count, total_area_count, total_area_count, false, true);
    message = "索驱已按pseudo_slam_bind_path.json执行完成";
    return true;
}

bool startPseudoSlamScan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::vector<Cabin_Point> con_path;
    float cabin_height = 0.0f;
    float cabin_speed = 0.0f;
    try {
        load_configured_path(con_path, cabin_height, cabin_speed);
        res.success = run_pseudo_slam_scan(con_path, cabin_height, cabin_speed, res.message);
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool bind_current_area_from_scan_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    try {
        res.success = run_current_area_bind_from_scan_test(res.message);
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}


bool startGlobalWork(chassis_ctrl::MotionControl::Request &req,
                            chassis_ctrl::MotionControl::Response &res) {
    printCurrentTime();
    printf("Cabin_log: 收到%s\n", req.command.c_str());
    try {
        const GlobalExecutionMode execution_mode = get_global_execution_mode();
        printCurrentTime();
        printf(
            "Cabin_log: 当前全局执行模式为%s。\n",
            global_execution_mode_name(execution_mode)
        );

        if (execution_mode == GlobalExecutionMode::kLiveVisual) {
            res.success = run_live_visual_global_work(res.message);
            return true;
        }

        std::ifstream scan_file(pseudo_slam_bind_path_json_file);
        if (scan_file.good()) {
            res.success = run_bind_from_scan(res.message);
            return true;
        }
        res.success = false;
        res.message =
            "当前全局执行模式为slam_precomputed，未找到pseudo_slam_bind_path.json，请先完成扫描建图或切换到live_visual模式";
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

/*
    读取索驱状态
*/
void read_cabin_state(Cabin_State *cab_state) {    
    float x_gesture,y_gesture;
    uint16_t check_sum=0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); //防止固定锁
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
            printf("Cabin_log: 校验和错误，当前TCP数据包被丢弃。\n");
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
        // printf("Cabin_log: 现在索驱的位置坐标为(%f,%f,%f),运动状态为", cab_state->X,cab_state->Y,cab_state->Z);
        // printf("%s\n", cab_state->motion_status ? " 运动。" : " 停止。");
        
        // 整合数据part1以上传
        cabin_data_upload.cabin_state_X = cab_state->X;
        cabin_data_upload.cabin_state_Y = cab_state->Y;
        cabin_data_upload.cabin_state_Z = cab_state->Z;
        cabin_data_upload.motion_status = cab_state->motion_status;
        cabin_data_upload.device_alarm = cab_state->device_alarm;
        cabin_data_upload.internal_calc_error = cab_state->internal_calc_error;
        publish_cabin_depth_tf();
        publish_pseudo_slam_point_transforms();

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
               printf("Cabin_error: 索驱设备报警，正在紧急关闭程序。\n");
               
            try
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                Frame_Generate_With_Retry(TCP_stop, 8, 8);
                close(sockfd);
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
               printf("Cabin_error: 索驱内部计算错误，正在紧急关闭程序。\n");
               
            try
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                Frame_Generate_With_Retry(TCP_stop, 8, 8);
                close(sockfd);
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
    //创建TCP连接
    if (!connectToServer()) {
        printCurrentTime();
        printf("Cabin_Error: 索驱进程创建TCP连接失败。\n");
        return false;
    }
    // 索驱使能和逆解激活
    Frame_Generate_With_Retry(motor_enable, 8, 6);
    usleep(500000);
    Frame_Generate_With_Retry(motor_inverse_enable, 8, 6);
    return true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    ros::init(argc, argv, "suoquNode");
    ros::NodeHandle nh;
    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);
    cabin_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
    printCurrentTime();
    printf("<--- suoquNode Started --->\n");
    
    if (!suoquInit())
    {
        printCurrentTime();
        printf("Cabin_Error: suoquNode初始化失败，节点退出。\n");
        return 1;
    }

    // 注册信号处理函数，捕获 SIGINT (Ctrl + C)
    signal(SIGINT, signalHandler_cc);
    // ros::ServiceServer service = nh.advertiseService("motion_control", handleMotionRequest);
    // motion_client = nh.serviceClient<chassis_ctrl::MotionControl>("motion_control");
    // 全局路径规划
    ros::ServiceServer path_config_service = nh.advertiseService("/cabin/plan_path", planGlobalMovePath);
    // 设置索驱速度
    ros::Subscriber change_cabin_speed_sub = nh.subscribe("/web/cabin/set_cabin_speed", 5, &change_cabin_speed_callback);
    ros::Subscriber global_execution_mode_sub = nh.subscribe("/web/cabin/set_execution_mode", 5, &global_execution_mode_callback);
    // 开始全局作业
    ros::ServiceServer update_service = nh.advertiseService("/cabin/start_work", startGlobalWork);
    ros::ServiceServer pseudo_slam_scan_service = nh.advertiseService("/cabin/start_pseudo_slam_scan", startPseudoSlamScan);
    ros::ServiceServer current_area_bind_from_scan_service = nh.advertiseService("/cabin/bind_current_area_from_scan", bind_current_area_from_scan_service);
    // 单点运动调试
    ros::ServiceServer cabin_single_move_service = nh.advertiseService("/cabin/single_move", cabin_single_move);
    pub_area_progress = nh.advertise<chassis_ctrl::AreaProgress>("/cabin/area_progress", 5);
    pub_pseudo_slam_markers = nh.advertise<visualization_msgs::MarkerArray>("/cabin/pseudo_slam_markers", 1, true);
    ros::Subscriber pseudo_slam_ir_image_sub = nh.subscribe(kPseudoSlamCaptureGateImageTopic, 1, &pseudo_slam_ir_image_callback);
    // 订阅视觉模块的消息
    AI_client = nh.serviceClient<fast_image_solve::ProcessImage>("/pointAI/process_image");
    sg_live_visual_client = nh.serviceClient<std_srvs::Trigger>("/moduan/sg");
    sg_precomputed_client = nh.serviceClient<chassis_ctrl::ExecuteBindPoints>("/moduan/sg_precomputed");
    sg_precomputed_fast_client = nh.serviceClient<chassis_ctrl::ExecuteBindPoints>("/moduan/sg_precomputed_fast");
    // 急停信息发布者对象(当前逻辑不考虑在索驱内部发送急停信号)
    // pub_forced_stop = nh.advertise<std_msgs::Float32>("/forced_stop", 5);
    // 索驱数据反馈的发布话题声明
    // pub_cabin_data_upload = nh.advertise<chassis_ctrl::cabin_upload>("/cabin/cabin_data_upload", 5);
    // pub_test=nh.advertise<std_msgs::Float32>("/test", 5);
    // 线程1索驱状态获取
    std::thread read_cabin_state_thread(read_cabin_state,&cabin_state); 
    read_cabin_state_thread.detach();
    // 订阅机器人当前姿态角
    // ros::Subscriber robot_angle = nh.subscribe("/camera/publisher_robot_angle", 5, from_camera_get_robot_attitude);
    // 连续办法的测试
    // ros::Subscriber cable_setup_con = nh.subscribe("/cabin/cabin_setup_continuouos", 5, cabin_global_discontinuous_move);
    // 订阅急停信息(通过末端发送)
    ros::Subscriber forced_stop_sub = nh.subscribe("/web/moduan/forced_stop", 5, &forced_stop_nodeCallback);
    // 订阅暂停中断信息
    ros::Subscriber interrupt0_sub = nh.subscribe("/web/moduan/interrupt_stop", 5, &pause_interrupt_Callback);
    ros::Subscriber hand_solve_stop = nh.subscribe("/web/moduan/hand_sovle_warn", 5, &solve_stop_Callback);
    ros::Subscriber moduan_work_sub = nh.subscribe("/moduan_work", 5, &moduan_work_Callback);
    ros::Subscriber send_odd_points_sub = nh.subscribe("/web/moduan/send_odd_points", 5, &checkerboard_jump_bind_callback);
    // 订阅末端安装的姿态传感器数据 （通过末端发送）
    // ros::Subscriber cabin_gesture = nh.subscribe("/moduan/moduan_gesture_data", 5, &cabin_gesture_get);
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
