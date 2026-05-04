#include "tie_robot_control/moduan/linear_module_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

#include <ros/ros.h>
#include <tie_robot_msgs/ExecuteBindPoints.h>

#include <tie_robot_control/common.hpp>
#include "tie_robot_control/moduan/numeric_codec.hpp"
#include "tie_robot_control/moduan/register_map.hpp"
#include "tie_robot_control/moduan/runtime_state.hpp"

using namespace tie_robot_control::moduan_registers;

void pub_moduan_work_state(bool moduan_work_flag);

void apply_module_speed_mm_per_sec(double new_speed)
{
    module_speed = new_speed;
    std::lock_guard<std::mutex> lock2(plc_mutex);
    Set_Module_Speed(WX_SPEED, &module_speed, plc);
    Set_Module_Speed(WY_SPEED, &module_speed, plc);
    Set_Module_Speed(WZ_SPEED, &module_speed, plc);
}

ScopedModuleSpeedOverride::ScopedModuleSpeedOverride(double override_speed)
    : original_speed_(module_speed), active_(override_speed > module_speed)
{
    if (active_) {
        printCurrentTime();
        ros_log_printf(
            "Moduan_log: 预计算当前区域直执行启用快速度，线性模组速度从%.1lf提升到%.1lf。\n",
            original_speed_,
            override_speed
        );
        apply_module_speed_mm_per_sec(override_speed);
    }
}

ScopedModuleSpeedOverride::~ScopedModuleSpeedOverride()
{
    if (active_) {
        apply_module_speed_mm_per_sec(original_speed_);
        printCurrentTime();
        ros_log_printf(
            "Moduan_log: 预计算当前区域直执行结束，线性模组速度恢复到%.1lf。\n",
            original_speed_
        );
    }
}

std::string compose_linear_module_driver_error_message(
    const std::string& prefix,
    const tie_robot_hw::driver::DriverError& driver_error)
{
    std::string detail = driver_error.message;
    if (!driver_error.detail.empty()) {
        detail += "，detail=" + driver_error.detail;
    }
    if (detail.empty() && g_linear_module_driver) {
        detail = g_linear_module_driver->lastErrorText();
    }
    if (detail.empty()) {
        return prefix;
    }
    return prefix + "，" + detail;
}

bool ensure_linear_module_driver_started(tie_robot_hw::driver::DriverError* error)
{
    if (!g_moduan_driver_enabled.load()) {
        if (error != nullptr) {
            error->code = "driver_disabled";
            error->message = "末端驱动已关闭";
            error->detail.clear();
            error->retryable = false;
        }
        return false;
    }
    if (!g_linear_module_driver) {
        g_linear_module_driver = std::make_unique<tie_robot_hw::driver::LinearModuleDriver>();
    }
    return g_linear_module_driver->start(error);
}

static std::vector<tie_robot_hw::driver::LinearModulePoint> build_linear_module_points_from_bind_points(
    const std::vector<tie_robot_msgs::PointCoords>& points)
{
    std::vector<tie_robot_hw::driver::LinearModulePoint> driver_points;
    driver_points.reserve(points.size());
    for (const auto& point : points) {
        tie_robot_hw::driver::LinearModulePoint driver_point;
        driver_point.x_mm = point.World_coord[0];
        driver_point.y_mm = point.World_coord[1];
        driver_point.z_mm = point.World_coord[2];
        driver_point.angle_deg = point.Angle;
        driver_points.push_back(driver_point);
    }
    return driver_points;
}

bool request_linear_module_zero_via_driver(std::string* error_message)
{
    tie_robot_hw::driver::DriverError driver_error;
    if (!ensure_linear_module_driver_started(&driver_error)) {
        const std::string detail = compose_linear_module_driver_error_message(
            "线性模组驱动连接失败",
            driver_error
        );
        if (error_message != nullptr) {
            *error_message = detail;
        }
        ROS_ERROR_STREAM("Moduan_Error: " << detail);
        return false;
    }

    if (!g_linear_module_driver->pulseExecutionEnable(&driver_error)) {
        const std::string detail = compose_linear_module_driver_error_message(
            "线性模组回零执行触发失败",
            driver_error
        );
        if (error_message != nullptr) {
            *error_message = detail;
        }
        ROS_ERROR_STREAM("Moduan_Error: " << detail);
        return false;
    }

    if (!g_linear_module_driver->setZeroRequest(true, &driver_error)) {
        const std::string detail = compose_linear_module_driver_error_message(
            "线性模组回零请求位写入失败",
            driver_error
        );
        if (error_message != nullptr) {
            *error_message = detail;
        }
        ROS_ERROR_STREAM("Moduan_Error: " << detail);
        return false;
    }

    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

namespace {

constexpr double kLinearModuleAxisArrivalToleranceMm = 5.0;
constexpr double kLinearModuleAxisArrivalStableDeltaMm = 1.0;
constexpr int kLinearModuleAxisArrivalStableSampleCount = 3;
constexpr int kLinearModuleAxisArrivalTimeoutSec = 20;
constexpr int kLinearModuleAxisArrivalLogIntervalSec = 2;
constexpr double kLinearModuleAxisArrivalPollSec = 0.02;
constexpr auto kFinishAllPollInterval = std::chrono::milliseconds(150);
constexpr auto kFinishAllTimeout = std::chrono::seconds(kFinishAllTimeoutSec);

class ScopedPlcExecutionState
{
public:
    ScopedPlcExecutionState()
    {
        moduan_plc_execution_state.store(true, std::memory_order_release);
        pub_moduan_work_state(true);
    }

    ~ScopedPlcExecutionState()
    {
        moduan_plc_execution_state.store(false, std::memory_order_release);
        pub_moduan_work_state(false);
    }
};

struct LinearModuleAxisSnapshot
{
    double position = 0.0;
    int error_flag = 0;
    const char* name = "unknown";
};

LinearModuleAxisSnapshot read_linear_module_axis_snapshot(int Axis)
{
    LinearModuleAxisSnapshot snapshot;
    std::lock_guard<std::mutex> lock(module_state_mutex);
    if (Axis == AXIS_X) {
        snapshot.position = module_state.X;
        snapshot.error_flag = static_cast<int>(module_state.ERROR_FLAG_X);
        snapshot.name = "X";
    } else if (Axis == AXIS_Y) {
        snapshot.position = module_state.Y;
        snapshot.error_flag = static_cast<int>(module_state.ERROR_FLAG_Y);
        snapshot.name = "Y";
    } else if (Axis == AXIS_Z) {
        snapshot.position = module_state.Z;
        snapshot.error_flag = static_cast<int>(module_state.ERROR_FLAG_Z);
        snapshot.name = "Z";
    } else if (Axis == AXIS_MOTOR) {
        snapshot.position = motor_state.MOTOR_ANGLE;
        snapshot.error_flag = static_cast<int>(motor_state.ERROR_FLAG_MOTOR);
        snapshot.name = "MOTOR";
    }
    return snapshot;
}

}  // namespace

bool wait_linear_module_axis_arrival(int Axis, double target_coordinate)
{
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    int stable_sample_count = 0;
    double previous_position = std::numeric_limits<double>::quiet_NaN();

    while (ros::ok()) {
        const LinearModuleAxisSnapshot snapshot = read_linear_module_axis_snapshot(Axis);
        if (snapshot.error_flag != 0) {
            printCurrentTime();
            ros_log_printf(
                "Moduan_Error: 等待线性模组%s轴到位时检测到错误标志=%d，目标位置 %.2f，当前位置 %.2f。\n",
                snapshot.name,
                snapshot.error_flag,
                target_coordinate,
                snapshot.position
            );
            return false;
        }

        const double axis_error_mm = std::fabs(snapshot.position - target_coordinate);
        const double pose_delta_mm = std::isfinite(previous_position)
            ? std::fabs(snapshot.position - previous_position)
            : std::numeric_limits<double>::infinity();
        previous_position = snapshot.position;

        if (axis_error_mm <= kLinearModuleAxisArrivalToleranceMm &&
            (!std::isfinite(pose_delta_mm) ||
             pose_delta_mm <= kLinearModuleAxisArrivalStableDeltaMm)) {
            stable_sample_count++;
            if (stable_sample_count >= kLinearModuleAxisArrivalStableSampleCount) {
                return true;
            }
        } else {
            stable_sample_count = 0;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_sec >= kLinearModuleAxisArrivalTimeoutSec) {
            printCurrentTime();
            ros_log_printf(
                "Moduan_Error: 等待线性模组%s轴到位超时，目标位置 %.2f，当前位置 %.2f，误差 %.2fmm，连续稳定样本=%d/%d。\n",
                snapshot.name,
                target_coordinate,
                snapshot.position,
                axis_error_mm,
                stable_sample_count,
                kLinearModuleAxisArrivalStableSampleCount
            );
            return false;
        }

        const auto log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (log_elapsed_sec >= kLinearModuleAxisArrivalLogIntervalSec) {
            printCurrentTime();
            ros_log_printf(
                "Moduan_log: 等待线性模组%s轴到位中，目标位置 %.2f，当前位置 %.2f，误差 %.2fmm，位置变化 %.2fmm，连续稳定样本=%d/%d。\n",
                snapshot.name,
                target_coordinate,
                snapshot.position,
                axis_error_mm,
                std::isfinite(pose_delta_mm) ? pose_delta_mm : -1.0,
                stable_sample_count,
                kLinearModuleAxisArrivalStableSampleCount
            );
            last_log_time = now;
        }

        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            if (error_detected) {
                printCurrentTime();
                ros_log_printf(
                    "Moduan_Error: 等待线性模组%s轴到位时收到全局错误标志，目标位置 %.2f，当前位置 %.2f。\n",
                    snapshot.name,
                    target_coordinate,
                    snapshot.position
                );
                return false;
            }
        }

        ros::Duration(kLinearModuleAxisArrivalPollSec).sleep();
    }
    return false;
}

bool arrive_z(int axis_z, double& z, bool& is_current_z)
{
    (void)axis_z;
    float t = z / module_speed;
    ros::Duration(t).sleep();
    while (true)
    {
        double ceju_current_z_ = 0.;
        bool current_z_flag_ = false;
        int arrivez_flag = 0;
        {
            ceju_current_z_ = current_z;
            current_z_flag_ = current_z_flag;
            arrivez_flag = static_cast<int>(module_state.ARRIVEZ_FLAG);
        }

        if (current_z_flag_) {
            z = ceju_current_z_;
            current_z_flag = false;
            is_current_z = true;
        }

        if (arrivez_flag == 1)
        {
            printCurrentTime();
            ros_log_printf("当前z轴状态: [%d, %d, %lf mm]\n", arrivez_flag, (int)current_z_flag_, z);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
}

void clear_finishall_flag_if_needed()
{
    int finishall_flag = 0;
    {
        std::lock_guard<std::mutex> lock2(module_state_mutex);
        finishall_flag = static_cast<int>(module_state.FINISH_ALL_FLAG);
    }
    if (!finishall_flag) {
        return;
    }

    printCurrentTime();
    ros_log_printf("Moduan_log: 检测到上一轮FINISHALL标志仍为1，先清零后再启动本轮末端执行。\n");
    tie_robot_hw::driver::DriverError driver_error;
    if (ensure_linear_module_driver_started(&driver_error) &&
        g_linear_module_driver->clearFinishAll(&driver_error)) {
        return;
    }

    const std::string driver_error_message = compose_linear_module_driver_error_message(
        "线性模组清理FINISHALL失败，回退旧PLC写入链",
        driver_error
    );
    ROS_WARN_STREAM("Moduan_Warn: " << driver_error_message);
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(FINISHALL, 0, plc);
    }
}

bool wait_for_plc_finish_all(std::chrono::milliseconds poll_interval, std::chrono::seconds timeout)
{
    const auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    while (true)
    {
        int finishall_flag = 0;
        double cur_x = 0.0;
        double cur_y = 0.0;
        double cur_z = 0.0;
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            finishall_flag = static_cast<int>(module_state.FINISH_ALL_FLAG);
            cur_x = module_state.X;
            cur_y = module_state.Y;
            cur_z = module_state.Z;
        }

        if (finishall_flag) break;

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_sec >= timeout.count()) {
            printCurrentTime();
            ros_log_printf(
                "Moduan_Error: 等待FINISHALL标志超时，超时=%ds，当前FINISH_ALL_FLAG=%d，当前位置(X,Y,Z)=(%.2f,%.2f,%.2f)。\n",
                static_cast<int>(timeout.count()),
                finishall_flag,
                cur_x,
                cur_y,
                cur_z
            );
            return false;
        }

        const auto log_elapsed_sec =
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
        if (log_elapsed_sec >= kFinishAllLogIntervalSec) {
            printCurrentTime();
            ros_log_printf(
                "Moduan_log: 等待FINISHALL标志中，当前FINISH_ALL_FLAG=%d，已等待%lds，当前位置(X,Y,Z)=(%.2f,%.2f,%.2f)。\n",
                finishall_flag,
                static_cast<long>(elapsed_sec),
                cur_x,
                cur_y,
                cur_z
            );
            last_log_time = now;
        }

        std::this_thread::sleep_for(poll_interval);
    }

    printCurrentTime();
    ros_log_printf("当前子区域绑扎完成，准备启动置0\n");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(FINISHALL, 0, plc);
    }
    return true;
}

void moveLinearModule(double x, double y, double z, double angle) {
    printCurrentTime();
    ros_log_printf("Moduan_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n", x, y, z);
    bool is_error = false;
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        is_error = error_detected;
    }
    while (is_error)
    {
        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            is_error = error_detected;
        }
        printCurrentTime();
        ros_log_printf("Moduan_Error: waiting on current target pt \n");
    }

    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        ROS_WARN("Cur Angle: %f\n", angle);
        Set_Motor_Angle(&angle, plc);
    }

    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE, &x, plc);
        Set_Module_Coordinate(WY_COORDINATE, &y, plc);
    }

    if (!wait_linear_module_axis_arrival(AXIS_X, x) ||
        !wait_linear_module_axis_arrival(AXIS_Y, y)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE, &z, plc);
    }
    wait_linear_module_axis_arrival(AXIS_Z, z);
}

int linear_module_move_origin_single(int Axis)
{
    std::lock_guard<std::mutex> lock2(plc_mutex);
    double zero_target = 0;
    if (Axis == AXIS_X) Set_Module_Coordinate(WX_COORDINATE, &zero_target, plc);
    if (Axis == AXIS_Y) Set_Module_Coordinate(WY_COORDINATE, &zero_target, plc);
    if (Axis == AXIS_Z) Set_Module_Coordinate(WZ_COORDINATE, &zero_target, plc);
    return 0;
}

void move_linear_module_to_origin()
{
    double zero_target = 0;
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE, &zero_target, plc);
    }
    if (!wait_linear_module_axis_arrival(AXIS_Z, zero_target)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE, &zero_target, plc);
        Set_Module_Coordinate(WY_COORDINATE, &zero_target, plc);
    }
    wait_linear_module_axis_arrival(AXIS_X, zero_target);
    wait_linear_module_axis_arrival(AXIS_Y, zero_target);
}

double max_bind_height_excess_mm(const std::vector<float>& out_of_height_z_values)
{
    double max_excess = 0.0;
    for (float world_z : out_of_height_z_values) {
        max_excess = std::max(max_excess, static_cast<double>(world_z) - kBindMaxHeightMm);
    }
    return max_excess;
}

std::string append_bind_height_excess_message(const std::string& message, double height_excess_mm)
{
    if (height_excess_mm <= 0.0) {
        return message;
    }

    std::ostringstream oss;
    oss << message << "; " << kBindHeightExcessMessageKey
        << std::fixed << std::setprecision(2) << height_excess_mm;
    return oss.str();
}

bool should_keep_jump_bind_point(const tie_robot_msgs::PointCoords& point)
{
    if (send_odd_points != 1) {
        return true;
    }
    return point.idx == 1 || point.idx == 4;
}

bool is_valid_precomputed_tcp_travel_z(double local_z_mm)
{
    return std::isfinite(local_z_mm) &&
           local_z_mm >= kTcpTravelMinZMm &&
           local_z_mm <= kTcpTravelMaxZMm;
}

bool is_valid_precomputed_tcp_travel_point(double local_x_mm, double local_y_mm, double local_z_mm)
{
    return std::isfinite(local_x_mm) &&
           std::isfinite(local_y_mm) &&
           local_x_mm >= 0.0 &&
           local_x_mm <= kTravelMaxXMm &&
           local_y_mm >= 0.0 &&
           local_y_mm <= kTravelMaxYMm &&
           is_valid_precomputed_tcp_travel_z(local_z_mm);
}

void inputAllPoints(int i, double x, double y, double z, double rz)
{
    if (i < 0 || i >= kPointSlotCount) {
        return;
    }
    std::lock_guard<std::mutex> lock2(plc_mutex);
    Set_Module_Coordinate(kXCoordinateSlots[i], &x, plc);
    Set_Module_Coordinate(kYCoordinateSlots[i], &y, plc);
    Set_Module_Coordinate(kZCoordinateSlots[i], &z, plc);
    Set_Motor_Angle(kRzCoordinateSlots[i], &rz, plc);
}

bool execute_bind_points(
    const std::vector<tie_robot_msgs::PointCoords>& filteredPoints,
    std::string& response_message,
    bool apply_jump_bind_filter)
{
    if (g_use_remote_moduan_driver.load(std::memory_order_relaxed)) {
        tie_robot_msgs::ExecuteBindPoints raw_execute_srv;
        raw_execute_srv.request.points = filteredPoints;
        ScopedPlcExecutionState plc_execution_state;
        if (!ros::service::call("/moduan/driver/raw_execute_points", raw_execute_srv)) {
            response_message = "无法调用线性模组驱动层 raw execute 服务 /moduan/driver/raw_execute_points";
            return false;
        }
        response_message = raw_execute_srv.response.message;
        return raw_execute_srv.response.success;
    }

    ROS_WARN("Cur pt_vec size: %zu\n", filteredPoints.size());
    if (filteredPoints.empty()) {
        printCurrentTime();
        ros_log_printf("Moduan_Warn: 预生成绑扎点为空，跳过当前区域。\n");
        response_message = "预生成绑扎点为空，跳过当前区域";
        return false;
    }

    int selected_bind_point_count = 0;
    int rejected_invalid_tcp_travel_count = 0;
    std::vector<tie_robot_msgs::PointCoords> selected_bind_points;
    bind_data.first.push_back(0);

    for (int i = 0; i < static_cast<int>(filteredPoints.size()); i++) {
        const auto& point = filteredPoints[i];
        if (apply_jump_bind_filter && !should_keep_jump_bind_point(point)) {
            continue;
        }

        float_t world_x = point.World_coord[0];
        float_t world_y = point.World_coord[1];
        float_t world_z = point.World_coord[2];
        float_t angle = point.Angle;

        if (!is_valid_precomputed_tcp_travel_point(
                static_cast<double>(world_x),
                static_cast<double>(world_y),
                static_cast<double>(world_z))) {
            rejected_invalid_tcp_travel_count++;
            printCurrentTime();
            ros_log_printf(
                "Moduan_Warn: 预生成点 idx=%d 的TCP局部坐标(%.2f,%.2f,%.2f)mm，不是合法TCP行程X[0.00, %.2f] Y[0.00, %.2f] Z[%.2f, %.2f]mm，已拒绝下发。\n",
                point.idx,
                world_x,
                world_y,
                world_z,
                kTravelMaxXMm,
                kTravelMaxYMm,
                kTcpTravelMinZMm,
                kTcpTravelMaxZMm
            );
            continue;
        }

        printCurrentTime();
        ROS_INFO(
            "Current %d, 线性模组移动x:%f, y:%f, z:%f, 旋转角度:%f\n",
            i,
            world_x,
            world_y,
            world_z,
            angle
        );

        bind_data.second.push_back(float(world_x));
        bind_data.second.push_back(float(world_y));
        bind_data.second.push_back(float(world_z));
        bind_data.second.push_back(float(angle));

        selected_bind_points.push_back(point);
        selected_bind_point_count++;
    }

    if (selected_bind_point_count == 0 && rejected_invalid_tcp_travel_count > 0) {
        printCurrentTime();
        ros_log_printf(
            "Moduan_Warn: 当前组全部点的TCP局部坐标都超出行程X[0.00, %.2f] Y[0.00, %.2f] Z[%.2f, %.2f]mm，跳过当前组。\n",
            kTravelMaxXMm,
            kTravelMaxYMm,
            kTcpTravelMinZMm,
            kTcpTravelMaxZMm
        );
        response_message = "预生成点TCP局部坐标超出行程，当前组无可执行点";
        bind_data.first.back() = 0;
        bind_all_data.push_back(bind_data);
        return false;
    }

    bind_data.first.back() = selected_bind_point_count;
    tie_robot_hw::driver::DriverError driver_error;
    if (!ensure_linear_module_driver_started(&driver_error)) {
        response_message = compose_linear_module_driver_error_message(
            "线性模组驱动连接失败",
            driver_error
        );
        return false;
    }
    if (selected_bind_point_count != 0) {
        if (!g_linear_module_driver->clearFinishAll(&driver_error)) {
            response_message = compose_linear_module_driver_error_message(
                "线性模组清理FINISHALL失败",
                driver_error
            );
            return false;
        }
        const std::vector<tie_robot_hw::driver::LinearModulePoint> driver_points =
            build_linear_module_points_from_bind_points(selected_bind_points);
        if (!g_linear_module_driver->writeQueuedPoints(driver_points, &driver_error)) {
            response_message = compose_linear_module_driver_error_message(
                "线性模组预计算点位写入失败",
                driver_error
            );
            return false;
        }
        ScopedPlcExecutionState plc_execution_state;
        if (!g_linear_module_driver->pulseExecutionEnable(&driver_error)) {
            response_message = compose_linear_module_driver_error_message(
                "线性模组预计算点执行触发失败",
                driver_error
            );
            return false;
        }
        if (!wait_for_plc_finish_all(kFinishAllPollInterval, kFinishAllTimeout)) {
            response_message = "等待FINISHALL标志超时，当前子区域绑扎未确认完成";
            return false;
        }
    }
    bind_all_data.push_back(bind_data);
    response_message = "区域绑扎作业完成";
    return selected_bind_point_count > 0;
}
