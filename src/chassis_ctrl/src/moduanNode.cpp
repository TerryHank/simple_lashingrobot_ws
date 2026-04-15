#include <mutex>
#include <thread>
#include <chrono>
#include <thread>
#include <ctype.h>
#include <errno.h>
#include <cstdlib>
#include <csignal> 
#include <modbus.h>  
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/io.h>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <termios.h>

#include <ros/ros.h>
#include <pthread.h>
#include <sys/time.h>  
#include <sys/socket.h>  
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "simulated_annealing.h"
#include "chassis_ctrl/motion.h"
#include "fast_image_solve/PointCoords.h"
#include <geometry_msgs/Twist.h>  
#include "modbus_connect.h"
#include "sbus_decoder.h"
#include "chassis_ctrl/linear_module_upload.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_move_single.h"
#include "chassis_ctrl/linear_module_move.h"
#include <fast_image_solve/ProcessImage.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "common.hpp"

// Linux-specific code
// xy轴移动范围为0到380mm
// 旋转电机角度 0~180°
#define LIGHT 5084
bool light_state =0;
std::atomic<bool> error_detected(false);
std::mutex error_msg_mutex;
// 新增错误信息存储变量
std::string last_error_msg;

chassis_ctrl::motion transform_msg;
chassis_ctrl::linear_module_upload linear_module_data_upload;
fast_image_solve::ProcessImage srv;
ros::ServiceClient AI_client;
ros::ServiceClient linear_client;
std_srvs::Trigger Trigger_srv;
ros::ServiceClient trigger_client;

constexpr uint8_t kProcessImageModeAdaptiveHeight = 1;
constexpr uint8_t kProcessImageModeBindCheck = 2;
constexpr double kBindMaxHeightMm = 94.0;
constexpr const char* kBindHeightExcessMessageKey = "BIND_HEIGHT_EXCESS_MM=";

// 以下为向PLC发送指令时写入的地址偏移量
#define WX_SPEED 5050 // 写入三轴速度和坐标
#define WY_SPEED 5054
#define WZ_SPEED 5058
#define WX_COORDINATE 5062
#define WY_COORDINATE 5066
#define WZ_COORDINATE 5070
#define X_COORDINATE_ONE 5510
#define Y_COORDINATE_ONE 5512
#define Z_COORDINATE_ONE 5514
#define RZ_COORDINATE_ONE 5516

#define X_COORDINATE_TWO 5520
#define Y_COORDINATE_TWO 5522
#define Z_COORDINATE_TWO 5524
#define RZ_COORDINATE_TWO 5526

#define X_COORDINATE_THREE 5530
#define Y_COORDINATE_THREE 5532
#define Z_COORDINATE_THREE 5534
#define RZ_COORDINATE_THREE 5536

#define X_COORDINATE_FOUR 5540
#define Y_COORDINATE_FOUR 5542
#define Z_COORDINATE_FOUR 5544
#define RZ_COORDINATE_FOUR 5546

#define X_COORDINATE_FIVE 5550
#define Y_COORDINATE_FIVE 5552
#define Z_COORDINATE_FIVE 5554
#define RZ_COORDINATE_FIVE 5556

#define X_COORDINATE_SIX 5560
#define Y_COORDINATE_SIX 5562
#define Z_COORDINATE_SIX 5564
#define RZ_COORDINATE_SIX 5566

#define X_COORDINATE_SEVEN 5570
#define Y_COORDINATE_SEVEN 5572
#define Z_COORDINATE_SEVEN 5574
#define RZ_COORDINATE_SEVEN 5576

#define X_COORDINATE_EIGHT 5580
#define Y_COORDINATE_EIGHT 5582
#define Z_COORDINATE_EIGHT 5584
#define RZ_COORDINATE_EIGHT 5586

#define X_COORDINATE_NINE 5590
#define Y_COORDINATE_NINE 5592
#define Z_COORDINATE_NINE 5594
#define RZ_COORDINATE_NINE 5596

#define MODULE_STOP 6456 // 置0x0001时停止x轴运动，置0x0002时停止y轴运动，置0x0004时停止z轴运动，置0x0008时停止三轴运动
#define EN_DISABLE 5076 // 置1时使能模组运动，绑扎枪打开；置0时模组失能，绑扎枪关闭
#define WARNING_RESET 5077 // 故障复位
#define LASHING 5078 // 执行绑扎动作
#define WRITING_ANGLE 5080 // 写入旋转电机目标角度（°）
#define WRITING_RMOTOR_SPEED 5082// 写入电机旋转速度（°/s）
#define EN_DISABLE_RMOTOR 5087 // 置1时失能旋转电机，置0时使能旋转电机

#define RESET_RMOTOR 5088 // 置1时旋转电机报警时复位，需要手动清零

#define CEJU 5068
#define ARRIVEZ 5069
#define FINISHALL 5210

// 以下为读取PLC状态的地址偏移量
#define RX_SPEED 5150 // 读取三轴速度和坐标
#define RY_SPEED 5154
#define RZ_SPEED 5158
#define RX_COORDINATE 5162  
#define RY_COORDINATE 5166
#define RZ_COORDINATE 5170
#define IS_ZERO 5072
#define IS_STOP 5074 // 1暂停 0恢复
#define IS_ERROR 5174 // 1异常 0正常
#define IS_LASHING 5173 // 1绑扎 0不绑扎
#define ERROR_INQUIRE 5175 // bit0=1代表x轴出现异常，bit1=1代表y轴出现异常，bit2=1代表z轴出现异常
#define MODULE_STATUS 5176 // 读取到1时代表设备处于远程控制状态，读取到0代表设备正处于显示屏操作状态
#define EMERGENCY_STOP 5177 // 急停状态
#define READING_ANGLE 5178 // 读取旋转电机当前角度（°）
#define READING_RMOTOR_SPEED 5182// 读取电机旋转速度（°/s）

#define BATTERY_VOLTAGE 5187 // 电池电压
#define INNER_TEM 5191 // 机器人内腔温度
#define X_GESTURE 5196
#define Y_GESTURE 5198

#define AXIS_X 0
#define AXIS_Y 3
#define AXIS_Z 4
#define AXIS_MOTOR 5

// 全局变量，modbust_t 套接字
modbus_t* plc;
double left_motor_speed = 0;
double right_motor_speed = 0;
double gain_speed = 0;
int channel1_percent=0;
int channel2_percent=0;
int channel3_percent=0;
int state = 0;
int send_odd_points = 3;  // 1 2 3
int fd = -1;
uint8_t data;
ros::Publisher pub_ctl_vel;
ros::Publisher pub_wheel_direction;
std_msgs::Int32 msg_state;
std_msgs::Float32 msg_vel;
// 用于存储浮点数转化成的2个uint16_t数据
uint16_t UtoF_Register[2];
uint16_t DtoU_Register[2];
// 数组中存储2个uint16_t数据转化为浮点数
uint16_t UtoD_Register[2];
static uint16_t last_JULI = 0;
// 线性模组的运行速度
double module_speed= 250;
// 旋转电机运行速度
double motor_speed= 360;
double current_z = 0;
bool current_z_flag = false;
std_msgs::Int32 msg;
// 旋转电机默认角度
double reset_angle =0;
static double last_left_motor_speed = 0;
static double last_right_motor_speed = 0;
// 电机角度发布者
ros::Publisher pub_motor_theata;
// 线性模组返回原点的标志位
ros::Publisher pub_linear_module_gb_origin;
// 线性模组节点急停信号发布者
ros::Publisher pub_forced_stop;
// 末端报警信号发布者
ros::Publisher pub_moduan_warning;
// 绑扎枪报警信号发布者
ros::Publisher pub_lashing_warning;
// 末端作业信号
ros::Publisher pub_moduan_work;

ros::Publisher pub_pause;
ros::Publisher pub_linear_module_data_upload;
// 暂停中断标志位 0为未启用暂停中断或恢复，1为启用暂停中断
int pause_interrupt = 0;

// 定义互斥锁来保护 module_state 的读写操作
std::mutex module_state_mutex;
std::mutex plc_mutex;
uint16_t  MODULE;
// 线性模组状态的结构体
typedef struct {
    double X;
    double Y;
    double Z;
    double X_SPEED;
    double Y_SPEED;
    double Z_SPEED;
    uint16_t ERROR_FLAG_X;
    uint16_t ERROR_FLAG_Y;
    uint16_t ERROR_FLAG_Z;
    uint16_t ERROR_FLAG_LASHING;
    float x_gesture;
    float y_gesture;
    uint16_t JULI;
    uint16_t ARRIVEZ_FLAG;
    uint16_t FINISH_ALL_FLAG;
} Module_State;
Module_State module_state;

// 旋转电机状态的结构体
typedef struct {
    double MOTOR_ANGLE;
    double MOTOR_SPEED;
    uint16_t ERROR_FLAG_MOTOR;
} Motor_State;
Motor_State motor_state;

bool enable_lashing=true;
std::mutex lashing_mutex;  // 新增互斥锁
ros::Subscriber enb_las_sub;  // 新增订阅者

// 在全局变量部分添加计数器和发布者
int pause_interrupt_count = 0; // 记录暂停信号的次数
bool handle_pause_interrupt = false;
std::pair<std::vector<int>, std::vector<float>> bind_data; // 理论绑扎点数量
std::vector<std::pair<std::vector<int>, std::vector<float>>> bind_all_data;
void enable_lashing_callback(const std_msgs::Float32::ConstPtr& msg);  
void moduan_move_zero_forthread(double x, double y, double z, double angle);
void moveLinearModule(double x, double y, double z,double angle );
void move_linear_module_to_origin();
void handle_system_error(const std::string& error_msg);

void pub_moduan_state( ros::Publisher &pub_linear_module_data_upload, Module_State* state, Motor_State* mot_state, float robot_battery_voltage, float robot_temperature)
{
    chassis_ctrl::linear_module_upload linear_module_data_upload;
    // 线性模组数据整合
    linear_module_data_upload.linear_module_position_X = state->X;
    linear_module_data_upload.linear_module_position_Y = state->Y;
    linear_module_data_upload.linear_module_position_Z = state->Z;
    linear_module_data_upload.linear_module_speed_X = state->X_SPEED;
    linear_module_data_upload.linear_module_speed_Y = state->Y_SPEED;
    linear_module_data_upload.linear_module_speed_Z = state->Z_SPEED;
    linear_module_data_upload.linear_module_error_flag_X = state->ERROR_FLAG_X;
    linear_module_data_upload.linear_module_error_flag_Y = state->ERROR_FLAG_Y;
    linear_module_data_upload.linear_module_error_flag_Z = state->ERROR_FLAG_Z;
    // 旋转电机整合
    linear_module_data_upload.motor_angle = mot_state->MOTOR_ANGLE;
    linear_module_data_upload.motor_speed = mot_state->MOTOR_SPEED;
    linear_module_data_upload.motor_error_flag = mot_state->ERROR_FLAG_MOTOR;
    // 机器人数据1整合
    linear_module_data_upload.robot_battery_voltage = robot_battery_voltage;
    linear_module_data_upload.robot_temperature = robot_temperature;
    linear_module_data_upload.x_gesture=state->y_gesture;
    linear_module_data_upload.y_gesture=state->x_gesture;
    pub_linear_module_data_upload.publish(linear_module_data_upload);
    return;
}

void pub_moduan_work_state(bool moduan_work_flag)
{
    std_msgs::Bool msg;
    msg.data = moduan_work_flag;
    pub_moduan_work.publish(msg);
}

void enable_lashing_callback(const std_msgs::Float32::ConstPtr& msg) {
    enable_lashing = bool(msg->data);
    printCurrentTime();
    printf("Moduan_log: 绑扎使能状态已更新为：%s\n", enable_lashing ? "允许绑扎" : "禁止绑扎");
    if(enable_lashing){
        std::lock_guard<std::mutex> lock(plc_mutex);
        PLC_Order_Write(IS_LASHING, 1, plc);
    }else {
        std::lock_guard<std::mutex> lock(plc_mutex);
        PLC_Order_Write(IS_LASHING, 0, plc);
    }
    return;
}

/*************************************************************************************************************************************************************/
// 函数功能：将双精度数放大100倍后转化为2个uint16_t储存在DtoU_Register中
// 函数输入：MyDouble 待转换的浮点数
// 函数输出：无
/*************************************************************************************************************************************************************/
void Double_to_Int32(double* MyDouble) {
    // 假设需要放大100倍
    int32_t temp32 = static_cast<int32_t>(*MyDouble * 100); // 放大100倍并转换为uint32_t
    
    DtoU_Register[1] = (temp32 & 0xFFFF0000) >> 16; // 高16位
    DtoU_Register[0] = temp32 & 0x0000FFFF;         // 低16位
}
/*************************************************************************************************************************************************************/
// 函数功能：将双精度数放大100倍后转化为2个uint16_t储存在DtoU_Register中
// 函数输入：MyDouble 待转换的浮点数
// 函数输出：无
/*************************************************************************************************************************************************************/
void ggbom(double* MyDouble) {
    // 假设需要放大100倍
    int32_t temp32 = static_cast<int32_t>(*MyDouble ); // 放大100倍并转换为uint32_t
    
    DtoU_Register[1] = (temp32 & 0xFFFF0000) >> 16; // 高16位
    DtoU_Register[0] = temp32 & 0x0000FFFF;         // 低16位
}
/*************************************************************************************************************************************************************/
// 函数功能：将双精度数放大100倍后转化为2个uint16_t储存在DtoU_Register中
// 函数输入：MyDouble 待转换的浮点数
// 函数输出：无
/*************************************************************************************************************************************************************/

void Double_to_Double(double* MyDouble) {
    // 假设需要放大100倍
    int32_t temp32 = static_cast<int32_t>(*MyDouble); // 放大100倍并转换为uint32_t
    DtoU_Register[1] = (temp32 & 0xFF00) >> 8; // 高16位
    DtoU_Register[0] = temp32 & 0x00FF;         // 低16位
}
/*************************************************************************************************************************************************************/
// 函数功能：驱动层函数，转化双精度数前需要将读取到的数据放入UtoD_Register中
// 函数输入：无
// 函数输出：UtoD_Register转化成的双精度数
/*************************************************************************************************************************************************************/
double Int32_to_Double() {
    int32_t temp32 = 0;
    temp32 |= ((int32_t)UtoD_Register[1]) << 16; // 高16位
    temp32 |= (int32_t)UtoD_Register[0];         // 低16位
    return static_cast<double>(temp32) / 100.0;   // 缩小100倍
}

/*************************************************************************************************************************************************************/
// 函数功能：驱动层函数，建立与PLC之间的通信连接
// 函数输入：无
// 函数输出：上位机和PLC间的连接字指针
/*************************************************************************************************************************************************************/
modbus_t* PLC_Connection()
{
    // 创建modbus上下文，用于TCP连接  
    modbus_t *ctx = modbus_new_tcp("192.168.6.167", 502); // 替换为PLC的IP地址和端口号  
    if (ctx == NULL) { 
        printCurrentTime(); 
        fprintf(stderr, "Moduan_Error:未能创建modbus tcp上下文。\n");  
        return NULL;  
    }  

    // 设置超时时间（秒和微秒）
    uint32_t timeout_sec = 30;     // 3秒超时
    uint32_t timeout_usec = 0;    // 微秒

    // 设置Modbus响应超时时间
    modbus_set_response_timeout(ctx, timeout_sec, timeout_usec);

    // 连接到PLC  
    int rc = modbus_connect(ctx);  
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:PLC连接失败: %s\n", modbus_strerror(errno));  
        modbus_free(ctx);  
        return NULL;  
    } 
    return ctx;
}

/*************************************************************************************************************************************************************/
// 函数功能：设置模组运动速度
// 函数输入：axsis WX_SPEED，WY_SPEED，W_ZSPEED  speed 设定的速度（mm/s） ctx 上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
int Set_Module_Speed(int axis, double* speed, modbus_t* ctx)
{   
    Double_to_Int32(speed); // 使用新的转换函数
    int rc = modbus_write_registers(ctx, axis, 2, DtoU_Register); // 发送两个字
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:设置模组运动速度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    return 0;
}

/*************************************************************************************************************************************************************/
// 函数功能：设置模组运动坐标
// 函数输入：axsis WX_COORDINATE，WY_COORDINATE，WZ_COORDINATE / coordinate 设定模组某个轴要移动到的坐标(mm) / ctx 上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
int Set_Module_Coordinate(int axis, double* coordinate, modbus_t* ctx)
{   
    Double_to_Int32(coordinate); // 使用新的转换函数
    int rc = modbus_write_registers(ctx, axis, 2, DtoU_Register); // 发送两个字
    if (rc == -1) { 
        printCurrentTime(); 
        fprintf(stderr, "Moduan_Error:设置模组运动坐标失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    
    return 0;
}

/*************************************************************************************************************************************************************/
// 函数功能：参考文件开头定义的宏写入其它命令
// 函数输入：order 向PLC下达的命令类型 ctx 上位机和PLC间的连接字指针 control_word 命令字
// 函数输出：
/*************************************************************************************************************************************************************/
int PLC_Order_Write(int order, uint16_t control_word, modbus_t* ctx)
{
    int rc = modbus_write_register(ctx, order, control_word); // 注意：Modbus地址从0开始，但实际PLC地址可能不同  
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:写入命令失败: %s\n", modbus_strerror(errno));  
    } else {  
        printCurrentTime();
        printf("Moduan_log:已成功向寄存器写入命令。\n");  
    } 
    return 0;
}

/*************************************************************************************************************************************************************/
// 函数功能：读取模组运动速度
// 函数输入：axsis RX_SPEED，RY_SPEED，RZ_SPEED ctx 上位机和PLC间的连接字指针
// 函数输出：读取到的速度
/*************************************************************************************************************************************************************/
float Read_Module_Speed(int axis, modbus_t* ctx)
{   
    double speed;
    int rc = modbus_read_registers(ctx, axis, 2, UtoD_Register); // 读取两个字
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:读取模组运动速度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    speed = Int32_to_Double();
    return speed;
}

/*************************************************************************************************************************************************************/
// 函数功能：读取模组当前坐标
// 函数输入：axsis RX_COORDINATE,RY_COORDINATE,RZ_COORDINATE ctx 上位机和PLC间的连接字指针
// 函数输出：读取到的坐标
/*************************************************************************************************************************************************************/
float Read_Module_Coordinate(int axis, modbus_t* ctx)
{   
    double coordinate;
    int rc = modbus_read_registers(ctx, axis, 2, UtoD_Register); // 读取两个字
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:读取模组当前坐标失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    coordinate = Int32_to_Double();
    return coordinate;
}

/*************************************************************************************************************************************************************/
// 函数功能：读取模组或旋转电机的其它状态
// 函数输入：status 文件开头定义的状态读取命令 ctx 上位机和PLC间的连接字指针
// 函数输出：读取到的状态字
/*************************************************************************************************************************************************************/
uint16_t Read_Module_Status(int status, modbus_t* ctx)
{   
    uint16_t status_word;
    int rc = modbus_read_registers(ctx, status, 1, &status_word); // 注意：Modbus地址从0开始，但实际PLC地址可能不同  
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:读取模组命令失败： %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    return status_word;
}

/*************************************************************************************************************************************************************/
// 函数功能：设置旋转电机运行速度
// 函数输入：num 旋转电机速度 ctx 上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
int Set_Motor_Speed(double* num, modbus_t* ctx)
{   
    Double_to_Double(num);
 
    int rc = modbus_write_registers(ctx, WRITING_RMOTOR_SPEED, 2, DtoU_Register);  
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Motor_Error:设置旋转电机速度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    return 0;
}

/*************************************************************************************************************************************************************/
// 函数功能：设置旋转电机目标角度
// 函数输入：num 旋转电机角度 ctx 上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
int Set_Motor_Angle(double* num, modbus_t* ctx)
{   
    ggbom(num);

    int rc = modbus_write_registers(ctx, WRITING_ANGLE, 2, DtoU_Register);  

    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Motor_Error:设置旋转电机角度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    return 0;
}
int Set_Motor_Angle(int motor_num, double* num, modbus_t* ctx)
{   
    ggbom(num);

    int rc = modbus_write_registers(ctx, motor_num, 2, DtoU_Register);  

    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Motor_Error:设置旋转电机角度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    return 0;
}

/*************************************************************************************************************************************************************/
// 函数功能：读取旋转电机当前速度
// 函数输入：上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
double Read_Motor_Speed(modbus_t* ctx)
{   
    double output;
    int rc = modbus_read_registers(ctx, READING_RMOTOR_SPEED, 2, UtoD_Register); // 读取两个字
    if (rc == -1) {  
        fprintf(stderr, "Motor_Error:读取旋转电机速度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    output = Int32_to_Double();
    return output;
}

/*************************************************************************************************************************************************************/
// 函数功能：读取旋转电机当前角度
// 函数输入：上位机和PLC间的连接字指针
// 函数输出：
/*************************************************************************************************************************************************************/
double Read_Motor_Angle(modbus_t* ctx)
{   
    double output;
    int rc = modbus_read_registers(ctx, READING_ANGLE, 2, UtoD_Register); // 读取两个字
    if (rc == -1) {  
        fprintf(stderr, "Motor_Error:读取旋转电机角度失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    output = Int32_to_Double()*100;
    return output;
}
/******
/*************************************************************************************************************************************************************/
// 函数功能：线性模组和旋转单机状态读取函数，从读命令字的返回值中获取索驱的X,Y,Z和三轴运行速度，三轴状态检测
// 输入：Module_State类型下的module_state变量
// 输出：Module_State 的属性X，属性Y，属性Z和属性三轴运动速度，和三轴状态检测
/*************************************************************************************************************************************************************/

float Read_Gesture(int axis,modbus_t* ctx)
{
    uint32_t temp32 = 0xFFFFFFFF;
    float temp;
    int rc = modbus_read_registers(ctx, axis, 2, UtoF_Register); // 注意：Modbus地址从0开始，但实际PLC地址可能不同  
    if (rc == -1) {  
        printCurrentTime();
        fprintf(stderr, "Moduan_Error:读取姿态失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    temp32&=((uint32_t)UtoF_Register[1]) << 16;
    temp32 |= (uint32_t)UtoF_Register[0];
    temp = *(float*)(&temp32);
    return temp;
}

void delay_time(int Axis, double traget_coordinate, int mode)
{
    double cur = 0.;
    const double TOL = 80;          // 1 mm 到位容差
    const double DT  = 0.02;         // 20 ms 轮询周期

    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(module_state_mutex);
            if (Axis == AXIS_X) cur = module_state.X;
            else if (Axis == AXIS_Y) cur = module_state.Y;
            else if (Axis == AXIS_Z) cur = module_state.Z;
            else if (Axis == AXIS_MOTOR) cur = motor_state.MOTOR_ANGLE;
        }
        if (std::fabs(cur - traget_coordinate) <= TOL) break;
        ros::Duration(DT).sleep();   // 小睡，降低 CPU 占用
    }
}

bool arrive_z(int axis_z, double &z, bool &is_current_z)
{
    float t =  z / module_speed;
    ros::Duration(t).sleep();
    while (true)
    {
        double lastest_z = 0.;
        double ceju_current_z_ = 0.;
        bool current_z_flag_ = false;
        int arrivez_flag = 0;
        {
            // std::lock_guard<std::mutex> lock2(module_state_mutex);
            // std::lock_guard<std::mutex> lock3(plc_mutex);
            ceju_current_z_ = current_z;
            current_z_flag_ = current_z_flag;
            arrivez_flag = (int)module_state.ARRIVEZ_FLAG;
        }
        
            if (current_z_flag_) 
            {
                z = ceju_current_z_;
                current_z_flag = false;
                is_current_z = true;
            }
            
            if (arrivez_flag == 1)
            {
                printCurrentTime();
                printf("当前z轴状态: [%d, %d, %lf mm]\n", arrivez_flag, (int)current_z_flag_,z);
                break;
            }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true; // 正常完成
}

bool finish_all(int inter_time)
{
    while (true)
    {
        int finishall_flag = 0;
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            finishall_flag = (int)module_state.FINISH_ALL_FLAG;
        }
        
        if (finishall_flag) break;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(inter_time));
    }
    printCurrentTime();
    printf("当前子区域绑扎完成，准备启动置0\n");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(FINISHALL,0,plc);
    }
    return true; // 正常完成
}

/**
 * @brief 调用 linear_module_move 服务控制线性模组移动到指定位置
 * 
 * @param client 服务客户端对象
 * @param x X 轴目标坐标
 * @param y Y 轴目标坐标
 * @param z Z 轴目标坐标
 */
void moveLinearModule(double x, double y, double z,double angle ) {
    printCurrentTime();
    printf("Moduan_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);
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
        printf("Moduan_Error: waiting on current target pt \n");
    }
    
    // 旋转电机先动
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        ROS_WARN("Cur Angle: %f\n", angle);
        Set_Motor_Angle(&angle, plc);
    }
   
    // 再让x,y同时移动到目标点上方
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE,&x,plc);
        Set_Module_Coordinate(WY_COORDINATE,&y,plc);
    }
    
    delay_time(AXIS_X,x,0);
    delay_time(AXIS_Y,y,0);
    // 最后再让Z伸下去
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z,plc);
    }
    delay_time(AXIS_Z,z,0);
// bool is_current_z = false;
// if (arrive_z(AXIS_Z, z, is_current_z) && is_current_z)
//     printf("Moduan_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,current_z);
// else 
//     printf("Moduan_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);
}

/*************************************************************************************************************************************************************/
// 函数功能：驱动/应用层函数，控制线性模组单轴回到原点。
// 函数输入：轴体标号 axis(\)
// 函数输出：
/*************************************************************************************************************************************************************/
int linear_module_move_origin_single(int Axis)
{
    std::lock_guard<std::mutex> lock2(plc_mutex);
    double zero_target = 0;
    if(Axis == AXIS_X )
        Set_Module_Coordinate(WX_COORDINATE,&zero_target,plc);//设置x轴运动到坐标0mm
    if(Axis == AXIS_Y )
        Set_Module_Coordinate(WY_COORDINATE,&zero_target,plc);//设置y轴运动到坐标0mm
    if(Axis == AXIS_Z )
        Set_Module_Coordinate(WZ_COORDINATE,&zero_target,plc);//设置z轴运动到坐标0mm
    return 0;
}

void move_linear_module_to_origin()
{
    double zero_target = 0;
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&zero_target,plc);
    }
    delay_time(AXIS_Z, zero_target, 0);

    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE,&zero_target,plc);
        Set_Module_Coordinate(WY_COORDINATE,&zero_target,plc);
    }
    delay_time(AXIS_X, zero_target, 0);
    delay_time(AXIS_Y, zero_target, 0);
}

/*************************************************************************************************************************************************************/
// 函数功能：暂停中断的回调函数
// 函数输入：暂停中断的信号
// 函数输出：
/*************************************************************************************************************************************************************/
void pause_interrupt_Callback(const std_msgs::Float32 &debug_mes)
{
    if (debug_mes.data == 1.0)
    {
        std::lock_guard<std::mutex> lock(plc_mutex);
        PLC_Order_Write(IS_STOP, 1, plc);
        handle_pause_interrupt = true;
        printCurrentTime();
        printf("Moduan_log:手动暂停，正在暂停末端运动。\n");
        
    }
    return;
}
/*************************************************************************************************************************************************************/
// 函数功能：全局统一错误处理回调函数
// 函数输入：错误信息字符串
// 函数输出：无
/*************************************************************************************************************************************************************/
void handle_system_error(const std::string& error_msg) {
    if(error_detected) return; // 避免重复处理
    printCurrentTime();
    printf("Moduan_ERROR: %s\n", error_msg.c_str());
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex); // 新增互斥锁保护错误信息
        error_detected = true;
        last_error_msg = error_msg;
    }
    printCurrentTime();
    printf("Moduan_log: 暂停中。\n");
    // 发布暂停信号（保持原有逻辑）
    // std_msgs::Bool pause_msg;
    // pause_msg.data = true;
    // pub_pause.publish(pause_msg);
    return;
}
// 函数功能：对点进行蛇形排序（C++实现）
// 参数：centers 包含点坐标的列表（格式与PointCoords一致）
// 返回：排序后的点列表
std::vector<fast_image_solve::PointCoords> snake_sort(const std::vector<fast_image_solve::PointCoords>& centers, int row_threshold = 20) {
    if (centers.empty()) return {};

    // 1. 按y坐标排序（从上到下）
    auto sortedByY = centers;
    std::sort(sortedByY.begin(), sortedByY.end(), 
        [](const fast_image_solve::PointCoords& a, const fast_image_solve::PointCoords& b) {
            return a.Pix_coord[1] < b.Pix_coord[1];  // 使用像素y坐标排序
        });

    // 2. 按行分组（y坐标差值阈值内为同一行）
    std::vector<std::vector<fast_image_solve::PointCoords>> rows;
    std::vector<fast_image_solve::PointCoords> current_row;
    current_row.push_back(sortedByY[0]);
    int last_y = sortedByY[0].Pix_coord[1];

    for (size_t i = 1; i < sortedByY.size(); ++i) {
        if (std::abs(sortedByY[i].Pix_coord[1] - last_y) <= row_threshold) {
            current_row.push_back(sortedByY[i]);
        } else {
            rows.push_back(current_row);
            current_row.clear();
            current_row.push_back(sortedByY[i]);
            last_y = sortedByY[i].Pix_coord[1];
        }
    }
    if (!current_row.empty()) rows.push_back(current_row);

    // 3. 每行按x坐标排序，偶数行反向
    std::vector<fast_image_solve::PointCoords> sorted_points;
    for (size_t i = 0; i < rows.size(); ++i) {
        auto& row = rows[i];
        // 按x坐标排序（从左到右）
        std::sort(row.begin(), row.end(), 
            [](const fast_image_solve::PointCoords& a, const fast_image_solve::PointCoords& b) {
                return a.Pix_coord[0] < b.Pix_coord[0];  // 使用像素x坐标排序
            });
        // 偶数行反向（注意行索引从0开始，i%2==1表示第二行及以后的偶数索引行）
        if (i % 2 == 1) {
            std::reverse(row.begin(), row.end());
        }
        sorted_points.insert(sorted_points.end(), row.begin(), row.end());
    }

    return sorted_points;
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

void inputAllPoints(int i, double x, double y, double z, double rz)
{
    if (i == 0)
    {   
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_ONE,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_ONE,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_ONE,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_ONE,&rz, plc);
    }
    else if (i == 1)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_TWO,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_TWO,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_TWO,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_TWO,&rz, plc);
    }
    else if (i == 2)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_THREE,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_THREE,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_THREE,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_THREE,&rz, plc);
    }
    else if (i == 3)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_FOUR,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_FOUR,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_FOUR,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_FOUR,&rz, plc);
    }
    else if (i == 4)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_FIVE,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_FIVE,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_FIVE,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_FIVE,&rz, plc);
    }
    else if (i == 5)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_SIX,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_SIX,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_SIX,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_SIX,&rz, plc);
    }
    else if (i == 6)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_SEVEN,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_SEVEN,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_SEVEN,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_SEVEN,&rz, plc);
    }
    else if (i == 7)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_EIGHT,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_EIGHT,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_EIGHT,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_EIGHT,&rz, plc);
    }
    else if (i == 8)
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(X_COORDINATE_NINE,&x,plc);
        Set_Module_Coordinate(Y_COORDINATE_NINE,&y,plc);
        Set_Module_Coordinate(Z_COORDINATE_NINE,&z,plc);
        Set_Motor_Angle(RZ_COORDINATE_NINE,&rz, plc);
    }
    return;
}
/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，区域绑扎作业的服务处理函数，完成单个区域的全部绑扎作业（改为服务）
// 输入：std_srvs/Trigger 请求（无实际输入）
// 输出：std_srvs/Trigger 响应（包含执行结果和消息）
/*************************************************************************************************************************************************************/
float last_x = 0.; float last_y = 0.;
bool moduan_bind_service(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    {
        int finishall_flag = 0;
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            finishall_flag = (int)module_state.FINISH_ALL_FLAG;
        }
        if (finishall_flag)
        {
            std::lock_guard<std::mutex> lock2(plc_mutex);
            PLC_Order_Write(FINISHALL, 0, plc); 
        }      
    }
    pub_moduan_work_state(true);
    const int max_retries = 3;  // 最大重试次数
    int retry_count = 0;
    bool found_points = false;
    srv.request.request_mode = kProcessImageModeBindCheck;
    if (!AI_client.call(srv)) {
        res.success = false;
        res.message = "调用视觉服务失败";
        pub_moduan_work_state(false);
        return true;
    }else{
        
        ROS_INFO("Service call successful");
        // break;
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
        const double bind_height_excess_mm =
            max_bind_height_excess_mm(srv.response.out_of_height_z_values);
        res.success = false;
        res.message = append_bind_height_excess_message(
            srv.response.message,
            bind_height_excess_mm
        );
        pub_moduan_work_state(false);
        return true;
    }
    
    // 2：打印实际获取的点数量
    printf("Moduan_log: 子区域内钢筋绑扎点数量:%zu.\n", srv.response.PointCoordinatesArray.size());
    auto sortedArray = srv.response.PointCoordinatesArray;

    // 新增步骤1：过滤满足坐标范围的点（0 < world_x < 380 且 0 < world_y < 330）
    std::vector<fast_image_solve::PointCoords> filteredPoints;
    for (auto& point : sortedArray) {
        if (0 < (double)point.World_coord[0] && (double)point.World_coord[0] < 320 && 
            0 < (double)point.World_coord[1] && (double)point.World_coord[1] < 360 &&
           0 < (double)point.World_coord[2] && (double)point.World_coord[2] < 94) {
            filteredPoints.push_back(point);
        }
    }
    
    // 新增步骤2：对过滤后的点进行蛇形排序（C++实现）
    auto snakeSortedPoints = snake_sort(filteredPoints);  // 需要实现snake_sort函数
    ROS_WARN("Cur pt_vec size: %zu\n", snakeSortedPoints.size());
    if (snakeSortedPoints.empty()) {
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
    bind_data.first.push_back(int(snakeSortedPoints.size()));
    long long int total_ = 0;
    // 3：遍历处理排序后的绑扎点（修改为遍历排序后的数组）
    for (int i = 0; i < snakeSortedPoints.size(); i++) {

        auto start_time = std::chrono::steady_clock::now();
        if (i != 0 && send_odd_points == 1 && i % 2 == 0)
            continue;
        const auto& point = snakeSortedPoints[i];  // 使用排序后的点
        int32_t idx = point.idx;
        int32_t pix_x = point.Pix_coord[0];
        int32_t pix_y = point.Pix_coord[1];
        float_t world_x = point.World_coord[0];
        float_t world_y = point.World_coord[1];
        float_t world_z = point.World_coord[2];
        float_t angle = point.Angle;
        bool is_shuiguan = point.is_shuiguan;
    
        // double module_move_x = static_cast<double>(world_x);
        // double module_move_y = static_cast<double>(world_y);
        // double module_move_z = static_cast<double>(world_z);
        // double motor_theata = static_cast<double>(angle);
    
        printCurrentTime();
        ROS_INFO("Current %d, 线性模组移动x:%f, y:%f, z:%f, 旋转角度:%f\n", i,world_x, world_y, world_z, angle);
        bind_data.second.push_back(float(world_x));
        bind_data.second.push_back(float(world_y));
        bind_data.second.push_back(float(world_z));
        bind_data.second.push_back(float(angle));
        
        if(i != 0 && send_odd_points == 1)
            inputAllPoints((i+1)/2, world_x,world_y,world_z,angle);
        else 
            inputAllPoints(i, world_x,world_y,world_z,angle);

    }
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
    }
    if (snakeSortedPoints.size() != 0)
        finish_all(150);
    bind_all_data.push_back(bind_data);
    pub_moduan_work_state(false);
    // std::cout << "总耗时为："<< total_ << " ms" << std::endl;
    res.success = true;
    res.message = "区域绑扎作业完成";
    return true;
}

void signalHandler(int signum)
{
    try{
        std::lock_guard<std::mutex> lock2(plc_mutex);
        printCurrentTime();
        printf("Moduan_log:ctrl+c已被触发,关闭末端节点。\n");
         
        ros::shutdown();
        _exit(2);
    }
    catch (const std::exception &e) {
        printCurrentTime();
        printf("Moduan_error:线性模组操作异常。\n");
        _exit(2); //无法操作线性模组时，直接强制退出程序。
    }
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
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        Set_Motor_Speed(&motor_speed, plc);
        Set_Motor_Angle(&reset_angle, plc);
    }
    move_linear_module_to_origin();
    return ;
}

void moduan_move_zero_forthread(double x, double y, double z, double angle)
{
    (void)x;
    (void)y;
    (void)z;
    (void)angle;
    request_moduan_zero("末端回零线程触发");
}

void moduan_move_zero_callback(const std_msgs::Float32::ConstPtr& msg)
{
    (void)msg;
    request_moduan_zero("收到末端回零命令");
}

bool moduan_move_service(chassis_ctrl::linear_module_move::Request &req, 
    chassis_ctrl::linear_module_move::Response &res)
{
    double x = req.pos_x;
    double y = req.pos_y;
    double z = req.pos_z;
    double angle = req.angle;
    double z_zero = 0;
    pub_moduan_work_state(true);
    
    printCurrentTime();
    printf("Moduan_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);
    if(x < 0 || x > 320 || y < 0 || y > 360 || z < 0 || z > 94)
    {
        printCurrentTime();
        printf("Moduan_log:目标点超出范围。\n");
        res.success = false;
        res.message = "目标点超出范围";
        return res.success;
    }
    
    inputAllPoints(0, x,y,z,angle);
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
    }
    finish_all(150);
    pub_moduan_work_state(false);
    printf("Moduan_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);
    res.success = true;
    res.message = "运动完成";
    return true;
}

void light_switch(const std_msgs::Bool &debug_mes)
{
    
    if (debug_mes.data == false) {
        printCurrentTime();
        printf("Moduan_log:灯已关闭。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x00, plc); // 关闭灯
    } else if (debug_mes.data == true) {
        printCurrentTime();
        printf("Moduan_log:灯已打开。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x01, plc); // 打开灯
    }
}

void send_odd_points_callback(const std_msgs::Bool &debug_mes)
{
    if(debug_mes.data)
    {
        printCurrentTime();
        printf("Moduan_log:跳绑已开启。\n");
        send_odd_points  = 1;
    }
    else
    {
        printCurrentTime();
        printf("Moduan_log:跳绑已关闭。\n");
        send_odd_points  = 3;
    }
}

void change_speed_callback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    printf("Moduan_log:设置速度为%lf。\n",debug_mes.data);
    module_speed = static_cast<double>(debug_mes.data);
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Speed(WX_SPEED, &module_speed, plc);//设置x轴速度
        Set_Module_Speed(WY_SPEED, &module_speed, plc);//设置y轴速度
        Set_Module_Speed(WZ_SPEED, &module_speed, plc);//设置z轴速度
    }
}

// void change_motor_speed_callback(const std_msgs::Float32 &debug_mes)
// {   
//     printCurrentTime();
//     printf("Moduan_log:设置电机速度为%lf。\n",debug_mes.data);
//     motor_speed = static_cast<double>(debug_mes.data);
//     {
//         std::lock_guard<std::mutex> lock2(plc_mutex);
//         Set_Motor_Speed(&motor_speed, plc);
//     }
// }

void handSolveWarnCallback(const std_msgs::Float32 &warn_msg)
{
    if (warn_msg.data == 1.0 && !handle_pause_interrupt)
    {
        printCurrentTime();
        ROS_WARN("手动恢复报警复位\n");
        {
            
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 1, plc); //尝试清除伺服器异常
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
    return;
}

void read_module_motor_state(Module_State *state, Motor_State *mot_state) 
{  
    while (true)
    {
        float robot_battery_voltage = 0.;
        float robot_temperature = 0.;

        // 保护 module_state 读写操作，避免数据竞争
        {
            std::lock_guard<std::mutex> lock1(module_state_mutex);
            std::lock_guard<std::mutex> lock2(plc_mutex);
            // 线性模组坐标反馈获取
            // uint16_t MODULE_z = (MODULE & 0x04) != 0;
            // printf("MODULE,MODULE_z:%d,%d\n",MODULE,MODULE_z);
            state->X = Read_Module_Coordinate(RX_COORDINATE, plc);
            state->Y = Read_Module_Coordinate(RY_COORDINATE, plc);
            state->Z = Read_Module_Coordinate(RZ_COORDINATE, plc);
            // printf("state->Z:%f,module_state.Z:%f\n",state->Z,module_state.Z);
            // 线性模组运动速度反馈获取
            state->X_SPEED = Read_Module_Speed(RX_SPEED, plc);
            state->Y_SPEED = Read_Module_Speed(RY_SPEED, plc);
            state->Z_SPEED = Read_Module_Speed(RZ_SPEED, plc);
            // 线性模组三轴运动状态反馈
            uint16_t status_moudle_error_all = Read_Module_Status(ERROR_INQUIRE, plc);
            uint16_t status_moudle_error_ceju = Read_Module_Status(CEJU, plc);
            uint16_t status_moudle_arrive_z = Read_Module_Status(ARRIVEZ, plc);
            uint16_t status_moudle_finish = Read_Module_Status(FINISHALL, plc);
            state->JULI = (status_moudle_error_ceju & 0x02) != 0;
            state->ARRIVEZ_FLAG = (status_moudle_arrive_z & 0x02) != 0;
            state->FINISH_ALL_FLAG = (status_moudle_finish & 0x01) != 0;

        // if ((int)state->FINISH_ALL_FLAG)
        // {
        //     printCurrentTime();
        //     printf("daowei\n");
        //     {
        //         PLC_Order_Write(FINISHALL,0,plc);
        //     }
        // }
            

            // printf("status_moudle_error_ceju,state->JULI:%d,%d\n",status_moudle_error_ceju,state->JULI);
            state->ERROR_FLAG_X = (status_moudle_error_all & 0x01) != 0;      // BIT0: x轴伺服器处于报警状态
            state->ERROR_FLAG_Y = (status_moudle_error_all & 0x02) != 0;      // BIT1: y轴伺服器处于报警状态
            state->ERROR_FLAG_Z = (status_moudle_error_all & 0x04) != 0;      // BIT2: z轴伺服器处于报警状态
            mot_state->ERROR_FLAG_MOTOR = (status_moudle_error_all & 0x08) != 0;     // BIT3: Xz轴伺服器处于报警状态
            state->ERROR_FLAG_LASHING = (status_moudle_error_all & 0x10) != 0; // BIT4: 绑扎枪故障
            // state->x_gesture=Read_Gesture(X_GESTURE,plc);
            // state->y_gesture=Read_Gesture(Y_GESTURE,plc);
            // printf("state->x_gesture:%f,state->y_gesture:%f\n",state->x_gesture,state->y_gesture);
            // 旋转电机速度和角度和状态获取
            mot_state->MOTOR_SPEED = Read_Motor_Speed(plc);
            mot_state->MOTOR_ANGLE = Read_Motor_Angle(plc);
            robot_battery_voltage = Read_Module_Speed(BATTERY_VOLTAGE, plc);
            robot_temperature = Read_Module_Speed(INNER_TEM, plc);
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
            if(state->ERROR_FLAG_X == 1 || state->ERROR_FLAG_Y == 1 || state->ERROR_FLAG_Z == 1 || mot_state->ERROR_FLAG_MOTOR == 1 )
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
                // 急停，停止在当前位置，并尝试清理故障后退出。
                // std_msgs::Float32 forced_stop_flag;
                // forced_stop_flag.data = 1.0;
                // pub_forced_stop.publish(forced_stop_flag);
                ros::Duration(2.0).sleep();
            }
            // only one
            if (!current_z_flag && state->JULI == 1 && last_JULI == 0) {
                printCurrentTime();
                printf("Moduan_log: 检测到JULI上升沿信号！当前Z轴位置: %f mm\n", state->Z);
                current_z = state->Z; //
                current_z_flag = true; 
           
            }else if (state->JULI == 0){
                current_z = state->Z; //
                current_z_flag = false;
            }
            
            // 更新上一次的值
            last_JULI = state->JULI;
            
            
            // printCurrentTime();
            // printf("IMU: roll=%.3f° pitch=%.3f°\n",
            // Read_Gesture(X_GESTURE, plc) ,
            // Read_Gesture(Y_GESTURE, plc) );
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

std::string getBeijingTimeString()
{
    using clock = std::chrono::system_clock;

    auto now = clock::now();

    // 转换为 time_t
    std::time_t now_time = clock::to_time_t(now);

    // 转为本地时间（Linux / ROS 下通常就是北京时间）
    std::tm local_time{};
#ifdef __linux__
    localtime_r(&now_time, &local_time);  // Linux 推荐
#else
    localtime_s(&local_time, &now_time);  // Windows
#endif

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");

    return oss.str();
}

void robotSaveBindingDataCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::string filename = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bindData.txt";
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

        if (count == bind_all_data.size())
        {
            // 写入 int
            ofs << "Ints:";
            for (int v : ints)
            {
                ofs << " " << v;
            }
            ofs << "\n";
            // 写入 float（按每组 floats_per_int 个）
            ofs << "Floats:\n";
            for (size_t i = 0; i < floats.size(); ++i)
            {
                ofs << floats[i];

                // 每 floats_per_int 个换行
                if ((i + 1) % floats_per_int == 0)
                    ofs << "\n";
                else
                    ofs << " ";
            }

            ofs << "\n";
        }
    }

    ofs.close();
    
    return;
}

void initPLC()
{
    // 和PLC创建TCP连接
    plc=PLC_Connection();
    PLC_Order_Write(WARNING_RESET, 1, plc); // 尝试清除伺服器异常
    PLC_Order_Write(WARNING_RESET, 0, plc); // 尝试清除伺服器异常
    PLC_Order_Write(EN_DISABLE, 1, plc);
    PLC_Order_Write(FINISHALL, 0, plc);
    
    // 设置三轴运动速度
    Set_Module_Speed(WX_SPEED, &module_speed, plc);//设置x轴速度
    Set_Module_Speed(WY_SPEED, &module_speed, plc);//设置y轴速度
    Set_Module_Speed(WZ_SPEED, &module_speed, plc);//设置z轴速度
    Set_Motor_Speed(&motor_speed, plc);
    Set_Motor_Angle(&reset_angle,plc);
    return;
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

int main(int argc, char **argv) {

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moduanNode");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    printCurrentTime();
    printf("<---Linear_module_node Started.--->\n");
    // 注册信号处理函数，捕获 SIGINT (Ctrl + C)
    signal(SIGINT, signalHandler);
    initPLC();

    // 线性模组模块急停信号的话题
    pub_moduan_warning = nh_.advertise<std_msgs::Float32>("/robot/moduan_status", 5);
    pub_lashing_warning = nh_.advertise<std_msgs::Float32>("/robot/binding_gun_status", 5);
    pub_forced_stop = nh_.advertise<std_msgs::Float32>("/cabin/forced_stop", 5);
    pub_pause = nh_.advertise<std_msgs::Bool>("/cabin/pause_interrupt", 5);
    pub_moduan_work = nh_.advertise<std_msgs::Bool>("/moduan_work", 5);
    // pub_linear_module_gb_origin = nh_.advertise<std_msgs::Float32>("/moduan/linear_module_gb_origin", 5);

    AI_client = nh_.serviceClient<fast_image_solve::ProcessImage>("/pointAI/process_image");
    ros::ServiceServer linear_service = nh_.advertiseService("/moduan/single_move", moduan_move_service);
    ros::ServiceServer lashing_service = nh_.advertiseService("/moduan/sg", moduan_bind_service); 

    // 线性模组数据反馈的话题
    // pub_linear_module_data_upload = nh_.advertise<chassis_ctrl::linear_module_upload>("/moduan/linear_module_data_upload", 5);
    // 设置持续获取线性模组模块反馈
    std::thread get_module_state_thread(read_module_motor_state,&module_state,&motor_state);
    get_module_state_thread.detach();

    auto_zero_on_startup(private_nh);

    // 末端回零
    ros::Subscriber moduan_zero_sub = nh_.subscribe("/web/moduan/moduan_move_zero", 5, &moduan_move_zero_callback);
    // 订阅是否开绑
    ros::Subscriber enb_las_sub = nh_.subscribe("/web/moduan/enb_las", 5, &enable_lashing_callback);
    // 订阅急停信息
    ros::Subscriber forced_stop = nh_.subscribe("/web/moduan/forced_stop", 5, &forced_stop_nodeCallback);
    ros::Subscriber hand_solve_warn = nh_.subscribe("/web/moduan/hand_sovle_warn", 5, &handSolveWarnCallback);
    // 订阅暂停中断信息
    ros::Subscriber interrupt0 = nh_.subscribe("/web/moduan/interrupt_stop", 5, &pause_interrupt_Callback);
    // 跳绑
    ros::Subscriber send_odd = nh_.subscribe("/web/moduan/send_odd_points", 5, &send_odd_points_callback);
    // 改变末端速度
    ros::Subscriber change_speed = nh_.subscribe("/web/moduan/set_moduan_speed", 5, &change_speed_callback);
    // // 改变旋转电机速度
    // ros::Subscriber change_motor_speed = nh_.subscribe("/web/moduan/set_motor_speed", 5, &change_motor_speed_callback);

    // 订阅灯的开关指令
    ros::Subscriber light_order= nh_.subscribe("/web/moduan/light", 5, &light_switch);
    ros::Subscriber save_bind_data_sub = nh_.subscribe("/web/moduan/save_binding_data", 5, robotSaveBindingDataCallback);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();


    return 0;
}
