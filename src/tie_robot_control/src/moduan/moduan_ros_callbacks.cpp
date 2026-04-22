#include "tie_robot_control/moduan/moduan_ros_callbacks.hpp"

#include <chrono>
#include <clocale>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#include <ros/ros.h>

#include "common.hpp"
#include "tie_robot_control/moduan/error_handling.hpp"
#include "tie_robot_control/moduan/linear_module_executor.hpp"
#include "tie_robot_control/moduan/numeric_codec.hpp"
#include "tie_robot_control/moduan/register_map.hpp"
#include "tie_robot_control/moduan/runtime_state.hpp"

using namespace tie_robot_control::moduan_registers;

void pub_moduan_state(
    ros::Publisher& pub_linear_module_data_upload,
    Module_State* state,
    Motor_State* mot_state,
    float robot_battery_voltage,
    float robot_temperature)
{
    tie_robot_msgs::linear_module_upload linear_module_data_upload_msg;
    linear_module_data_upload_msg.linear_module_position_X = state->X;
    linear_module_data_upload_msg.linear_module_position_Y = state->Y;
    linear_module_data_upload_msg.linear_module_position_Z = state->Z;
    linear_module_data_upload_msg.linear_module_speed_X = state->X_SPEED;
    linear_module_data_upload_msg.linear_module_speed_Y = state->Y_SPEED;
    linear_module_data_upload_msg.linear_module_speed_Z = state->Z_SPEED;
    linear_module_data_upload_msg.linear_module_error_flag_X = state->ERROR_FLAG_X;
    linear_module_data_upload_msg.linear_module_error_flag_Y = state->ERROR_FLAG_Y;
    linear_module_data_upload_msg.linear_module_error_flag_Z = state->ERROR_FLAG_Z;
    linear_module_data_upload_msg.motor_angle = mot_state->MOTOR_ANGLE;
    linear_module_data_upload_msg.motor_speed = mot_state->MOTOR_SPEED;
    linear_module_data_upload_msg.motor_error_flag = mot_state->ERROR_FLAG_MOTOR;
    linear_module_data_upload_msg.robot_battery_voltage =
        (std::isfinite(robot_battery_voltage) && robot_battery_voltage > 0.0f && robot_battery_voltage <= 100.0f)
            ? robot_battery_voltage
            : 0.0f;
    linear_module_data_upload_msg.robot_temperature = robot_temperature;
    linear_module_data_upload_msg.x_gesture = state->y_gesture;
    linear_module_data_upload_msg.y_gesture = state->x_gesture;
    pub_linear_module_data_upload.publish(linear_module_data_upload_msg);
}

void pub_moduan_work_state(bool moduan_work_flag)
{
    std_msgs::Bool msg_;
    msg_.data = moduan_work_flag;
    pub_moduan_work.publish(msg_);
}

void enable_lashing_callback(const std_msgs::Float32::ConstPtr& msg)
{
    enable_lashing = bool(msg->data);
    printCurrentTime();
    printf("Moduan_log: 绑扎使能状态已更新为：%s\n", enable_lashing ? "允许绑扎" : "禁止绑扎");
    std::lock_guard<std::mutex> lock(plc_mutex);
    PLC_Order_Write(IS_LASHING, enable_lashing ? 1 : 0, plc);
}

void pause_interrupt_Callback(const std_msgs::Float32& debug_mes)
{
    if (debug_mes.data == 1.0)
    {
        std::lock_guard<std::mutex> lock(plc_mutex);
        PLC_Order_Write(IS_STOP, 1, plc);
        handle_pause_interrupt = true;
        printCurrentTime();
        printf("Moduan_log:手动暂停，正在暂停末端运动。\n");
    }
}

bool moduan_bind_service(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    (void)req;
    {
        int finishall_flag = 0;
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            finishall_flag = static_cast<int>(module_state.FINISH_ALL_FLAG);
        }
        if (finishall_flag)
        {
            std::lock_guard<std::mutex> lock2(plc_mutex);
            PLC_Order_Write(FINISHALL, 0, plc);
        }
    }
    pub_moduan_work_state(true);
    srv.request.request_mode = kProcessImageModeBindCheck;
    if (!AI_client.call(srv)) {
        res.success = false;
        res.message = "调用视觉服务失败";
        pub_moduan_work_state(false);
        return true;
    }

    if (!srv.response.success) {
        printCurrentTime();
        printf("Moduan_Warn: 绑扎视觉校验失败：%s\n", srv.response.message.c_str());
        if (srv.response.out_of_height_count > 0) {
            for (size_t i = 0; i < srv.response.out_of_height_point_indices.size() &&
                               i < srv.response.out_of_height_z_values.size(); ++i) {
                printf(
                    "Moduan_Warn: 超高点 idx=%d, 实际z=%.2fmm\n",
                    srv.response.out_of_height_point_indices[i],
                    srv.response.out_of_height_z_values[i]
                );
            }
        }
        res.success = false;
        res.message = srv.response.message;
        pub_moduan_work_state(false);
        return true;
    }

    if (srv.response.out_of_height_count > 0) {
        printCurrentTime();
        printf("Moduan_Warn: 绑扎视觉检测到超高点，本次仅记录日志，不拦截下游执行。\n");
        for (size_t i = 0; i < srv.response.out_of_height_point_indices.size() &&
                           i < srv.response.out_of_height_z_values.size(); ++i) {
            printf(
                "Moduan_Warn: 超高点 idx=%d, 实际z=%.2fmm\n",
                srv.response.out_of_height_point_indices[i],
                srv.response.out_of_height_z_values[i]
            );
        }
    }

    printf("Moduan_log: 子区域内钢筋绑扎点数量:%zu.\n", srv.response.PointCoordinatesArray.size());
    auto sortedArray = srv.response.PointCoordinatesArray;
    std::vector<tie_robot_msgs::PointCoords> filteredPoints(sortedArray.begin(), sortedArray.end());
    if (filteredPoints.empty()) {
        printCurrentTime();
        printf(
            "Moduan_Warn: 视觉无可用绑扎点，跳过当前区域。原始点数量:%zu，过滤后点数量:%zu。\n",
            sortedArray.size(),
            filteredPoints.size()
        );
        res.success = false;
        res.message = "视觉无可用绑扎点，跳过当前区域";
        pub_moduan_work_state(false);
        return true;
    }
    const bool executed = execute_bind_points(filteredPoints, res.message);
    pub_moduan_work_state(false);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "视觉无可用绑扎点，跳过当前区域";
    }
    return true;
}

bool moduan_bind_points_service(
    tie_robot_msgs::ExecuteBindPoints::Request &req,
    tie_robot_msgs::ExecuteBindPoints::Response &res)
{
    clear_finishall_flag_if_needed();
    pub_moduan_work_state(true);
    std::vector<tie_robot_msgs::PointCoords> points(req.points.begin(), req.points.end());
    const bool executed = execute_bind_points(points, res.message, false);
    pub_moduan_work_state(false);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "预生成绑扎点为空，跳过当前区域";
    }
    return true;
}

bool moduan_bind_points_fast_service(
    tie_robot_msgs::ExecuteBindPoints::Request &req,
    tie_robot_msgs::ExecuteBindPoints::Response &res)
{
    ScopedModuleSpeedOverride speed_override(kPrecomputedFastModuleSpeedMmPerSec);
    clear_finishall_flag_if_needed();
    pub_moduan_work_state(true);
    std::vector<tie_robot_msgs::PointCoords> points(req.points.begin(), req.points.end());
    const bool executed = execute_bind_points(points, res.message, false);
    pub_moduan_work_state(false);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "预生成绑扎点为空，跳过当前区域";
    }
    return true;
}

void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    if(debug_mes.data == 3.0)
    {
        printCurrentTime();
        printf("Moduan_log:急停信号已被触发，强制关闭末端节点。\n");
        ros::shutdown();
    }
}

void request_moduan_zero(const char* reason)
{
    printCurrentTime();
    printf("Moduan_log:%s，末端返回零点，旋转电机回零至 %.1f 度。\n", reason, reset_angle);
    std::string driver_error_message;
    if (request_linear_module_zero_via_driver(&driver_error_message)) {
        return;
    }
    ROS_WARN_STREAM("Moduan_Warn: " << driver_error_message << "，回退旧回零链。");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        Set_Motor_Speed(&motor_speed, plc);
        Set_Motor_Angle(&reset_angle, plc);
    }
    move_linear_module_to_origin();
}

void moduan_move_zero_forthread(double x, double y, double z, double angle)
{
    (void)x;
    (void)y;
    (void)z;
    (void)angle;
    printCurrentTime();
    printf("Moduan_log:末端返回零点。\n");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        PLC_Order_Write(IS_ZERO, 1, plc);
    }
}

void moduan_move_zero_callback(const std_msgs::Float32::ConstPtr& msg)
{
    (void)msg;
    printCurrentTime();
    printf("Moduan_log:末端返回零点。\n");
    std::string driver_error_message;
    if (request_linear_module_zero_via_driver(&driver_error_message)) {
        return;
    }
    ROS_WARN_STREAM("Moduan_Warn: " << driver_error_message << "，回退旧回零链。");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        PLC_Order_Write(IS_ZERO, 1, plc);
    }
}

bool moduan_move_service(
    tie_robot_msgs::linear_module_move::Request &req,
    tie_robot_msgs::linear_module_move::Response &res)
{
    double x = req.pos_x;
    double y = req.pos_y;
    double z = req.pos_z;
    double angle = req.angle;
    pub_moduan_work_state(true);

    printCurrentTime();
    printf("Moduan_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n", x, y, z);
    if(x < 0 || x > kTravelMaxXMm || y < 0 || y > kTravelMaxYMm ||
       z < kTcpTravelMinZMm || z > kTcpTravelMaxZMm)
    {
        printCurrentTime();
        printf("Moduan_log:目标点超出范围。\n");
        res.success = false;
        res.message = "目标点超出范围，TCP z轴行程仅支持0~140mm";
        return res.success;
    }

    tie_robot_msgs::PointCoords single_point;
    single_point.idx = 1;
    single_point.World_coord[0] = static_cast<float>(x);
    single_point.World_coord[1] = static_cast<float>(y);
    single_point.World_coord[2] = static_cast<float>(z);
    single_point.Angle = static_cast<float>(angle);

    std::vector<tie_robot_msgs::PointCoords> single_points{single_point};
    const bool executed = execute_bind_points(single_points, res.message, false);
    pub_moduan_work_state(false);
    if (!executed) {
        res.success = false;
        if (res.message.empty()) {
            res.message = "等待FINISHALL标志超时，线性模组未确认完成";
        }
        return true;
    }

    printf("Moduan_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n", x, y, z);
    res.success = true;
    res.message = "运动完成";
    return true;
}

void light_switch(const std_msgs::Bool &debug_mes)
{
    if (!debug_mes.data) {
        printCurrentTime();
        printf("Moduan_log:灯已关闭。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x00, plc);
    } else {
        printCurrentTime();
        printf("Moduan_log:灯已打开。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x01, plc);
    }
}

void send_odd_points_callback(const std_msgs::Bool &debug_mes)
{
    if(debug_mes.data)
    {
        printCurrentTime();
        printf("Moduan_log:跳绑2/4已开启；视觉直绑仅绑第1和第4个点，全局预计算路径由上游棋盘格过滤。\n");
        send_odd_points  = 1;
    }
    else
    {
        printCurrentTime();
        printf("Moduan_log:跳绑已关闭，恢复全绑。\n");
        send_odd_points  = 3;
    }
}

void change_speed_callback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    printf("Moduan_log:设置速度为%lf。\n",debug_mes.data);
    apply_module_speed_mm_per_sec(static_cast<double>(debug_mes.data));
}

void handSolveWarnCallback(const std_msgs::Float32 &warn_msg)
{
    if (warn_msg.data == 1.0 && !handle_pause_interrupt)
    {
        printCurrentTime();
        ROS_WARN("手动恢复报警复位\n");
        {
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 1, plc);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 0, plc);
            }
        }

        for(int n = 0; n < 6; n++)
        {
            std_msgs::Float32 error_flag;
            error_flag.data = 0.0;
            pub_lashing_warning.publish(error_flag);
            pub_moduan_warning.publish(error_flag);
        }
    }
    else {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(IS_STOP, 0, plc);
        handle_pause_interrupt = false;
        printCurrentTime();
        printf("Moduan_log:手动恢复，正在恢复末端运动。\n");
    }
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(FINISHALL, 0, plc);
    }
}

void read_module_motor_state(Module_State *state, Motor_State *mot_state)
{
    while (true)
    {
        float robot_battery_voltage = 0.;
        float robot_temperature = 0.;
        {
            std::lock_guard<std::mutex> lock1(module_state_mutex);
            std::lock_guard<std::mutex> lock2(plc_mutex);
            state->X = Read_Module_Coordinate(RX_COORDINATE, plc);
            state->Y = Read_Module_Coordinate(RY_COORDINATE, plc);
            state->Z = Read_Module_Coordinate(RZ_COORDINATE, plc);
            state->X_SPEED = Read_Module_Speed(RX_SPEED, plc);
            state->Y_SPEED = Read_Module_Speed(RY_SPEED, plc);
            state->Z_SPEED = Read_Module_Speed(RZ_SPEED, plc);
            uint16_t status_moudle_error_all = Read_Module_Status(ERROR_INQUIRE, plc);
            uint16_t status_moudle_error_ceju = Read_Module_Status(CEJU, plc);
            uint16_t status_moudle_arrive_z = Read_Module_Status(ARRIVEZ, plc);
            uint16_t status_moudle_finish = Read_Module_Status(FINISHALL, plc);
            state->JULI = (status_moudle_error_ceju & 0x02) != 0;
            state->ARRIVEZ_FLAG = (status_moudle_arrive_z & 0x02) != 0;
            state->FINISH_ALL_FLAG = (status_moudle_finish & 0x01) != 0;

            state->ERROR_FLAG_X = (status_moudle_error_all & 0x01) != 0;
            state->ERROR_FLAG_Y = (status_moudle_error_all & 0x02) != 0;
            state->ERROR_FLAG_Z = (status_moudle_error_all & 0x04) != 0;
            mot_state->ERROR_FLAG_MOTOR = (status_moudle_error_all & 0x08) != 0;
            state->ERROR_FLAG_LASHING = (status_moudle_error_all & 0x10) != 0;
            mot_state->MOTOR_SPEED = Read_Motor_Speed(plc);
            mot_state->MOTOR_ANGLE = Read_Motor_Angle(plc);
            robot_battery_voltage = Read_Module_Float_RangedScaled(BATTERY_VOLTAGE, plc, 0.0f, 100.0f, 0.01f);
            robot_temperature = Read_Module_Float_RangedScaled(INNER_TEM, plc, -40.0f, 120.0f, 0.01f);
        }
        {
            if(state->ERROR_FLAG_LASHING == 1)
            {
                printf("state->ERROR_FLAG_LASHING:%d\n",state->ERROR_FLAG_LASHING);
                std_msgs::Float32 error_flag;
                error_flag.data = 1.0;
                pub_lashing_warning.publish(error_flag);
                handle_system_error("检测绑扎枪报警");
            }
            if(state->ERROR_FLAG_X == 1 || state->ERROR_FLAG_Y == 1 ||
               state->ERROR_FLAG_Z == 1 || mot_state->ERROR_FLAG_MOTOR == 1 )
            {
                printCurrentTime();
                printf("Moduan_error:");
                std_msgs::Float32 error_flag;
                error_flag.data = 1.0;

                if(state->ERROR_FLAG_X == 1)
                {
                    printf("X轴异常！！！\n");
                    handle_system_error("X轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(state->ERROR_FLAG_Y == 1)
                {
                    printf("Y轴异常！！！\n");
                    handle_system_error("Y轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(state->ERROR_FLAG_Z == 1)
                {
                    printf("Z轴异常！！！\n");
                    handle_system_error("Z轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(mot_state->ERROR_FLAG_MOTOR == 1)
                {
                    printf("旋转电机异常！！！\n");
                    handle_system_error("旋转电机异常");
                    pub_moduan_warning.publish(error_flag);
                }
                printf("正在急停末端运动和旋转电机转动。\n");
                ros::Duration(2.0).sleep();
            }
            if (!current_z_flag && state->JULI == 1 && last_JULI == 0) {
                printCurrentTime();
                printf("Moduan_log: 检测到JULI上升沿信号！当前Z轴位置: %f mm\n", state->Z);
                current_z = state->Z;
                current_z_flag = true;
            } else if (state->JULI == 0) {
                current_z = state->Z;
                current_z_flag = false;
            }

            last_JULI = state->JULI;
        }
        pub_moduan_state(pub_linear_module_data_upload, state, mot_state, robot_battery_voltage, robot_temperature);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::string getBeijingTimeString()
{
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    std::time_t now_time = clock::to_time_t(now);
    std::tm local_time{};
#ifdef __linux__
    localtime_r(&now_time, &local_time);
#else
    localtime_s(&local_time, &now_time);
#endif
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

void robotSaveBindingDataCallback(const std_msgs::Float32::ConstPtr& msg) {
    (void)msg;
    std::string filename = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/data/bindData.txt";
    int floats_per_int = 4;
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs.is_open())
    {
        std::cerr << "[ERROR] Cannot open file: " << filename << std::endl;
        return;
    }
    ofs << "===== World Time: " << getBeijingTimeString() << " =====\n";

    int count = 0;
    for (const auto& block : bind_all_data)
    {
        count++;
        const auto& ints = block.first;
        const auto& floats = block.second;

        if (count == static_cast<int>(bind_all_data.size()))
        {
            ofs << "Ints:";
            for (int v : ints)
            {
                ofs << " " << v;
            }
            ofs << "\n";
            ofs << "Floats:\n";
            for (size_t i = 0; i < floats.size(); ++i)
            {
                ofs << floats[i];
                if ((i + 1) % floats_per_int == 0)
                    ofs << "\n";
                else
                    ofs << " ";
            }
            ofs << "\n";
        }
    }
}

void initPLC()
{
    g_linear_module_driver = std::make_unique<tie_robot_hw::driver::LinearModuleDriver>();
    plc = PLC_Connection();
    PLC_Order_Write(WARNING_RESET, 1, plc);
    PLC_Order_Write(WARNING_RESET, 0, plc);
    PLC_Order_Write(EN_DISABLE, 1, plc);
    PLC_Order_Write(FINISHALL, 0, plc);
    PLC_Order_Write(IS_STOP, 0, plc);
    Set_Module_Speed(WX_SPEED, &module_speed, plc);
    Set_Module_Speed(WY_SPEED, &module_speed, plc);
    Set_Module_Speed(WZ_SPEED, &module_speed, plc);
    Set_Motor_Speed(&motor_speed, plc);
    Set_Motor_Angle(&reset_angle, plc);
}

void auto_zero_on_startup(ros::NodeHandle& private_nh)
{
    bool auto_zero_on_start = true;
    private_nh.param("auto_zero_on_start", auto_zero_on_start, true);
    if (!auto_zero_on_start) {
        printCurrentTime();
        printf("Moduan_log:节点启动自动回零已关闭。\n");
        return;
    }

    ros::Duration(0.2).sleep();
    request_moduan_zero("节点启动自动回零");
}

int RunModuanNode(int argc, char** argv) {
    (void)argc;
    (void)argv;
    setlocale(LC_ALL, "");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    printCurrentTime();
    printf("<---Linear_module_node Started.--->\n");
    signal(SIGINT, signalHandler);
    initPLC();

    pub_moduan_warning = nh_.advertise<std_msgs::Float32>("/robot/moduan_status", 5);
    pub_lashing_warning = nh_.advertise<std_msgs::Float32>("/robot/binding_gun_status", 5);
    pub_forced_stop = nh_.advertise<std_msgs::Float32>("/cabin/forced_stop", 5);
    pub_pause = nh_.advertise<std_msgs::Bool>("/cabin/pause_interrupt", 5);
    pub_moduan_work = nh_.advertise<std_msgs::Bool>("/moduan_work", 5);
    pub_linear_module_data_upload = nh_.advertise<tie_robot_msgs::linear_module_upload>("/moduan/moduan_gesture_data", 5);

    AI_client = nh_.serviceClient<tie_robot_msgs::ProcessImage>("/pointAI/process_image");
    ros::ServiceServer linear_service = nh_.advertiseService("/moduan/single_move", moduan_move_service);
    ros::ServiceServer lashing_service = nh_.advertiseService("/moduan/sg", moduan_bind_service);
    ros::ServiceServer pseudo_slam_lashing_service = nh_.advertiseService("/moduan/sg_precomputed", moduan_bind_points_service);
    ros::ServiceServer pseudo_slam_lashing_fast_service = nh_.advertiseService("/moduan/sg_precomputed_fast", moduan_bind_points_fast_service);

    std::thread get_module_state_thread(read_module_motor_state, &module_state, &motor_state);
    get_module_state_thread.detach();

    ros::Subscriber moduan_zero_sub = nh_.subscribe("/web/moduan/moduan_move_zero", 5, &moduan_move_zero_callback);
    ros::Subscriber enb_las_sub_local = nh_.subscribe("/web/moduan/enb_las", 5, &enable_lashing_callback);
    ros::Subscriber forced_stop = nh_.subscribe("/web/moduan/forced_stop", 5, &forced_stop_nodeCallback);
    ros::Subscriber hand_solve_warn = nh_.subscribe("/web/moduan/hand_sovle_warn", 5, &handSolveWarnCallback);
    ros::Subscriber interrupt0 = nh_.subscribe("/web/moduan/interrupt_stop", 5, &pause_interrupt_Callback);
    ros::Subscriber send_odd = nh_.subscribe("/web/moduan/send_odd_points", 5, &send_odd_points_callback);
    ros::Subscriber change_speed = nh_.subscribe("/web/moduan/set_moduan_speed", 5, &change_speed_callback);
    ros::Subscriber light_order = nh_.subscribe("/web/moduan/light", 5, &light_switch);
    ros::Subscriber save_bind_data_sub = nh_.subscribe("/web/moduan/save_binding_data", 5, robotSaveBindingDataCallback);

    (void)private_nh;
    (void)linear_service;
    (void)lashing_service;
    (void)pseudo_slam_lashing_service;
    (void)pseudo_slam_lashing_fast_service;
    (void)moduan_zero_sub;
    (void)enb_las_sub_local;
    (void)forced_stop;
    (void)hand_solve_warn;
    (void)interrupt0;
    (void)send_odd;
    (void)change_speed;
    (void)light_order;
    (void)save_bind_data_sub;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
