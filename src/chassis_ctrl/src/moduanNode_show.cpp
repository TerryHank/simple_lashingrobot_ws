#include <mutex>
#include <thread>
#include <chrono>
#include <thread>
#include <ctype.h>
#include <stdio.h>
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
#include <ros/ros.h>
#include <pthread.h>
#include <sys/time.h>  
#include <sys/socket.h>  
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "simulated_annealing.h"
#include "chassis_ctrl/motion.h"
#include <geometry_msgs/Twist.h>  
#include "modbus_connect.h"
#include "sbus_decoder.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include "chassis_ctrl/linear_module_upload.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_move_single.h"
#include "chassis_ctrl/linear_module_move.h"
#include <fast_image_solve/ProcessImage.h>
#include "fast_image_solve/PointCoords.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

// xy轴移动范围为0到380mm
// 旋转电机角度 0~180°
#include "modbus_connect.h"
#define LIGHT 5084
bool light_state =0;
std::atomic<bool> error_detected(false);
std::mutex error_msg_mutex;
// 新增错误信息存储变量
std::string last_error_msg;

chassis_ctrl::motion transform_msg;
chassis_ctrl::linear_module_upload linear_module_data_upload;
fast_image_solve::ProcessImage srv;
ros::ServiceClient client;
ros::ServiceClient linear_client;
std_srvs::Trigger Trigger_srv;
ros::ServiceClient trigger_client;
// 以下为向PLC发送指令时写入的地址偏移量
#define WX_SPEED 5050 // 写入三轴速度和坐标
#define WY_SPEED 5054
#define WZ_SPEED 5058
#define WX_COORDINATE 5062
#define WY_COORDINATE 5066
#define WZ_COORDINATE 5070
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

// 以下为读取PLC状态的地址偏移量
#define RX_SPEED 5150 // 读取三轴速度和坐标
#define RY_SPEED 5154
#define RZ_SPEED 5158
#define RX_COORDINATE 5162  
#define RY_COORDINATE 5166
#define RZ_COORDINATE 5170
#define IS_ERROR 5174 // 读取到1时代表设备正常
#define ERROR_INQUIRE 5175 // bit0=1代表x轴出现异常，bit1=1代表y轴出现异常，bit2=1代表z轴出现异常
#define MODULE_STATUS 5176 // 读取到1时代表设备处于远程控制状态，读取到0代表设备正处于显示屏操作状态
#define EMERGENCY_STOP 5177 // 读取到1时代表模组处于紧急停止状态
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
double moduleXY_speed= 150;
double moduleZ_speed= 150;
// 旋转电机运行速度
double motor_speed= 360;
double current_z = 0;
bool current_z_flag = false;
std_msgs::Int32 msg;
// 旋转电机默认角度
double reset_angle =0;
static double last_left_motor_speed = 0;
static double last_right_motor_speed = 0;
// 子区域绑扎结束的标志位
ros::Publisher pub_lashing_finish;
// 电机角度发布者
ros::Publisher pub_motor_theata;
// 线性模组返回原点的标志位
ros::Publisher pub_linear_module_gb_origin;
// 线性模组节点急停信号发布者
ros::Publisher pub_forced_stop;

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
ros::Publisher pub_pause_interrupt_count; // 发布者，用于发送故障次数

void enable_lashing_callback(const std_msgs::Bool::ConstPtr& msg);  // 添加此行
void linear_module_move_zero(double x, double y, double z, double angle);
void linear_module_move_zero_forthread(double x, double y, double z, double angle);
void moveLinearModule(double x, double y, double z,double angle );
void handle_system_error(const std::string& error_msg);
/*************************************************************************************************************************************************************/
// 函数功能：决定是否开绑
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void enable_lashing_callback(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(lashing_mutex);
    enable_lashing = msg->data;
    printCurrentTime();
    printf("GPIO_log: 绑扎使能状态已更新为：%s\n", enable_lashing ? "允许绑扎" : "禁止绑扎");
}
/*************************************************************************************************************************************************************/
// 函数功能：驱动层函数，打印当前系统时间
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void printCurrentTime() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    
    // 转换为 time_t 格式
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // 转换为本地时间
    std::tm* local_time = std::localtime(&now_time_t);
    
    // 获取当前时间的毫秒部分
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
    
    // 缓冲区存储格式化后的时间
    char time_str[10];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);
    
    // 使用 printf 输出时间，精确到ms
    printf("%s.%03d", time_str, static_cast<int>(millis.count()));
    printf(" - ");
}
/*************************************************************************************************************************************************************/
// 函数功能：打印log信息，带时间戳
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void printLog(const char* format, ...) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time_t);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);

    printf("%s.%03d - ", time_str, (int)millis.count());

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf("\n");
    fflush(stdout);  // 强制刷新缓冲区
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
    modbus_t *ctx = modbus_new_tcp("192.168.6.73", 502); // 替换为PLC的IP地址和端口号  
    if (ctx == NULL) { 
        printCurrentTime(); 
        fprintf(stderr, "GPIO_Error:未能创建modbus tcp上下文。\n");  
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
        fprintf(stderr, "GPIO_Error:PLC连接失败: %s\n", modbus_strerror(errno));  
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
        fprintf(stderr, "GPIO_Error:设置模组运动速度失败: %s\n", modbus_strerror(errno));  
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
        fprintf(stderr, "GPIO_Error:设置模组运动坐标失败: %s\n", modbus_strerror(errno));  
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
        fprintf(stderr, "GPIO_Error:写入命令失败: %s\n", modbus_strerror(errno));  
    } else {  
        printCurrentTime();
        printf("GPIO_log:已成功向寄存器写入命令。\n");  
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
        fprintf(stderr, "GPIO_Error:读取模组运动速度失败: %s\n", modbus_strerror(errno));  
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
        fprintf(stderr, "GPIO_Error:读取模组当前坐标失败: %s\n", modbus_strerror(errno));  
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
        fprintf(stderr, "GPIO_Error:读取模组命令失败： %s\n", modbus_strerror(errno));  
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
    // PLC_Order_Write(EN_DISABLE, 1, plc);
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
        fprintf(stderr, "Gpio_Error:读取姿态失败: %s\n", modbus_strerror(errno));  
        modbus_close(ctx);  
        modbus_free(ctx);  
        return -1;  
    } 
    temp32&=((uint32_t)UtoF_Register[1]) << 16;
    temp32 |= (uint32_t)UtoF_Register[0];
    temp = *(float*)(&temp32);
    return temp;
}
/******
/*************************************************************************************************************************************************************/
// 函数功能：线性模组和旋转单机状态读取函数，从读命令字的返回值中获取索驱的X,Y,Z和三轴运行速度，三轴状态检测
// 输入：Module_State类型下的module_state变量
// 输出：Module_State 的属性X，属性Y，属性Z和属性三轴运动速度，和三轴状态检测
/*************************************************************************************************************************************************************/
void read_module_motor_state(Module_State *state, Motor_State *mot_state) 
{  
    while (true)
    {
        double robot_battery_voltage = 0.;
        double robot_temperature = 0.;

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
            state->JULI = (status_moudle_error_ceju & 0x02) != 0;
            state->ARRIVEZ_FLAG = (status_moudle_arrive_z & 0x02) != 0;
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
                handle_system_error("检测绑扎枪报警");

            }
            if(state->ERROR_FLAG_X == 1 || state->ERROR_FLAG_Y == 1 || state->ERROR_FLAG_Z == 1 || mot_state->ERROR_FLAG_MOTOR == 1 )
            {
                printCurrentTime();
                printf("GPIO_error:");
    
                if(state->ERROR_FLAG_X == 1)
                    printf("X轴异常！！！\n");
                if(state->ERROR_FLAG_Y == 1)
                    printf("Y轴异常！！！\n");
                if(state->ERROR_FLAG_Z == 1)
                    printf("Z轴异常！！！\n");
                if(mot_state->ERROR_FLAG_MOTOR == 1)
                    printf("旋转电机异常！！！");
                printf("正在急停线性模组运动和旋转电机转动，并紧急关闭程序。\n");
                // 急停，停止在当前位置，并尝试清理故障后退出。
                std_msgs::Float32 forced_stop_flag;
                forced_stop_flag.data = 1.0;
                pub_forced_stop.publish(forced_stop_flag);
            }
            // only one
            if (!current_z_flag && state->JULI == 1 && last_JULI == 0) {
                printCurrentTime();
                printf("GPIO_log: 检测到JULI上升沿信号！当前Z轴位置: %f mm\n", state->Z);
                current_z = state->Z; //
                current_z_flag = true; 
           
            }else if (state->JULI == 0){
                current_z = state->Z; //
                current_z_flag = false;
            }
            
            // 更新上一次的值
            last_JULI = state->JULI;
            // 机器人内腔温度和电池电压状态获取
            
            // printf("state->ERROR_FLAG_LASHING:%d\n",state->ERROR_FLAG_LASHING);
            
               // 整合反馈以上传
            // chassis_ctrl::linear_module_upload linear_module_data_upload;
            // // 线性模组数据整合
            // linear_module_data_upload.linear_module_position_X = state->X;
            // linear_module_data_upload.linear_module_position_Y = state->Y;
            // linear_module_data_upload.linear_module_position_Z = state->Z;
            // linear_module_data_upload.linear_module_speed_X = state->X_SPEED;
            // linear_module_data_upload.linear_module_speed_Y = state->Y_SPEED;
            // linear_module_data_upload.linear_module_speed_Z = state->Z_SPEED;
            // linear_module_data_upload.linear_module_error_flag_X = state->ERROR_FLAG_X;
            // linear_module_data_upload.linear_module_error_flag_Y = state->ERROR_FLAG_Y;
            // linear_module_data_upload.linear_module_error_flag_Z = state->ERROR_FLAG_Z;
            // // 旋转电机整合
            // linear_module_data_upload.motor_angle = mot_state->MOTOR_ANGLE;
            // linear_module_data_upload.motor_speed = mot_state->MOTOR_SPEED;
            // linear_module_data_upload.motor_error_flag = mot_state->ERROR_FLAG_MOTOR;
            // // 机器人数据1整合
            // linear_module_data_upload.robot_battery_voltage = robot_battery_voltage;
            // linear_module_data_upload.robot_temperature = robot_temperature;
            // linear_module_data_upload.x_gesture=state->y_gesture;
            // linear_module_data_upload.y_gesture=state->x_gesture;
            // pub_linear_module_data_upload.publish(linear_module_data_upload);

            // printCurrentTime();
            // printf("IMU: roll=%.3f° pitch=%.3f°\n",
            //     Read_Gesture(X_GESTURE, plc) ,

            //     Read_Gesture(Y_GESTURE, plc) );
// -------------------------------------
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }
}

/*************************************************************************************************************************************************************/
// 函数功能：驱动层函数，线性模组单轴运动的延时函数，并读取暂停~恢复信号
// 函数输入：目标位置，轴体
// 函数输出：
/*************************************************************************************************************************************************************/
void delay_time(int Axis, double traget_coordinate, int mode)
{
    double cur = 0.;
    double TOL = 80;          // 1 mm 到位容差
    double DT  = 0.02;         // 20 ms 轮询周期

    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(module_state_mutex);
            if (Axis == AXIS_X) cur = module_state.X;
            else if (Axis == AXIS_Y) cur = module_state.Y;
            else if (Axis == AXIS_Z) {cur = module_state.Z; TOL = 120;}
            else if (Axis == AXIS_MOTOR) cur = motor_state.MOTOR_ANGLE;
        }

        if (std::fabs(cur - traget_coordinate) <= TOL) break;
        ros::Duration(DT).sleep();   // 小睡，降低 CPU 占用
    }
}

bool arrive_z(int axis_z, double &z, bool &is_current_z)
{
    double lastest_z = 0.;
        double ceju_current_z_ = 0.;
        bool current_z_flag_ = false;
        int arrivez_flag = 0;
    while (true)
    {
        
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            std::lock_guard<std::mutex> lock3(plc_mutex);
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
            // printCurrentTime();
            
            if (arrivez_flag == 1)
            {
                printf("当前z轴状态: [%d, %d, %lf mm]\n", arrivez_flag, (int)current_z_flag_,z);
                break;
            }
        

        // if (AXIS_Z == axis_z && current_z_flag_ && (lastest_z - ceju_current_z_ > 1)) // di xiao plc guan liang
        // {
        //     ROS_WARN("wei tiao ing\n");
        //     {
        //         std::lock_guard<std::mutex> lock1(plc_mutex);
        //         Set_Module_Coordinate(WZ_COORDINATE,&ceju_current_z_,plc);
        //     }
        //     delay_time(AXIS_Z,ceju_current_z_,0);
        //     {
        //         std::lock_guard<std::mutex> lock2(module_state_mutex);
        //         current_z_flag = false;
        //     }
        //     is_current_z = true;
        //     break;
        //     return true;
            
        // }
        // else if(AXIS_Z == axis_z && current_z_flag_ && (lastest_z - ceju_current_z_  <= 1))
        // {
        //     z = lastest_z;
        //     {
        //         std::lock_guard<std::mutex> lock2(module_state_mutex);
        //         current_z_flag = false;
        //     }
        //     is_current_z = false;
        //     break;
        //     return true;
        // }
        // else 
        // {
        //     // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        //     // continue; // zhi xing yuan shi gei ding Z value
        //     delay_time(AXIS_Z,z,0);
        //     printCurrentTime();
        //     printf("Gpio_log:Z模组现在已经运行至(%lf)mm处。\n",z);
        //     break;
        //     return true;
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
    printf("Gpio_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);
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
        printf("Gpio_Error: waiting on current target pt\n");
    }
    
    // 旋转电机先动
    // {
    //     std::lock_guard<std::mutex> lock2(plc_mutex);
    //     ROS_WARN("Cur Angle: %f\n", angle);
    //     Set_Motor_Angle(&angle, plc);
    // }
    ROS_WARN("Cur Angle: %f\n", angle);
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Motor_Angle(&angle, plc);
    }
    // ros::Duration(0.1).sleep();
    // 再让x,y同时移动到目标点上方
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Motor_Angle(&angle, plc);
        Set_Module_Coordinate(WX_COORDINATE,&x,plc);
        Set_Module_Coordinate(WY_COORDINATE,&y,plc);

    }
    
    delay_time(AXIS_X,x,0);
// {
//     std::lock_guard<std::mutex> lock2(plc_mutex);
// Set_Module_Coordinate(WX_COORDINATE,&x,plc);
//     Set_Module_Coordinate(WY_COORDINATE,&y,plc);
// }
    delay_time(AXIS_Y,y,0);
    // 最后再让Z伸下去
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z,plc);
    }
    // delay_time(AXIS_Z,z,0);
    bool is_current_z = false;
    if (arrive_z(AXIS_Z, z, is_current_z) && is_current_z)
        printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,current_z);
    else 
        printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);
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

/*************************************************************************************************************************************************************/
// 函数功能：暂停中断的回调函数
// 函数输入：暂停中断的信号
// 函数输出：
/*************************************************************************************************************************************************************/
void pause_interrupt_Callback(const std_msgs::Bool &debug_mes)
{
    if(debug_mes.data) {
        pause_interrupt = 1;
        handle_system_error("手动暂停");
    }
    else if(!debug_mes.data) 
    {
        pause_interrupt = 0;
        // 重置错误标志
        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            error_detected = false;
            last_error_msg.clear();
        }
        printCurrentTime();
        printf("GPIO_log: 暂停中断信号已清除。\n");
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
    printf("GPIO_ERROR: %s\n", error_msg.c_str());
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex); // 新增互斥锁保护错误信息
        error_detected = true;
        last_error_msg = error_msg;
    }
    
    // 发布暂停信号（保持原有逻辑）
    std_msgs::Bool pause_msg;
    pause_msg.data = true;
    pub_pause.publish(pause_msg);
    printCurrentTime();
    printf("GPIO_log: 暂停中。\n");
}
/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，区域绑扎作业的服务处理函数，完成单个区域的全部绑扎作业（改为服务）
// 输入：std_srvs/Trigger 请求（无实际输入）
// 输出：std_srvs/Trigger 响应（包含执行结果和消息）
/*************************************************************************************************************************************************************/
float last_x = 0.; float last_y = 0.;
bool GPIO_Lashing_Service(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    const int max_retries = 3;  // 最大重试次数
    int retry_count = 0;
    bool found_points = false;

    // while (retry_count < max_retries && !found_points) {
    //     if (retry_count > 0) {
    //         printCurrentTime();
    //         printf("GPIO_log: 第%d次重试获取绑扎点...\n", retry_count);
    //         ros::Duration(0.5).sleep();  // 每次重试间隔1秒
    //     }
    // int cnt_ = 0;
    // while (cnt_ == 3)
    // {
    //     cnt_++;
    if (!client.call(srv)) {
        res.success = false;
        res.message = "调用视觉服务失败";
        // continue;
        return res.success;
    }else{
        
        ROS_INFO("Service call successful");
        // break;
    }
        // 
    // } 

    //     // 检查是否获取到至少4个有效绑扎点
    //     size_t point_count = srv.response.PointCoordinatesArray.size();
    //     if (point_count >= 4) {  // 修改判断条件为至少4个点
    //         found_points = true;
    //         break;
    //     } else {
    //         printCurrentTime();
    //         printf("GPIO_log: 本次获取%ld个绑扎点（需要至少4个），继续重试...\n", point_count);
    //     }

    //     retry_count++;
    // }

    // // 三次重试后仍未满足条件
    // if (!found_points) {
    //     printCurrentTime();
    //     printf("GPIO_log: 子区域内钢筋绑扎点数量不足（至少需要4个，重试%d次后失败）。\n", max_retries);
    //     pub_lashing_finish.publish(debug);
    //     res.success = false;
    //     res.message = "子区域内钢筋绑扎点数量不足（至少需要4个，重试3次后失败）";
    //     return true;
    // }
    
    // 2：打印实际获取的点数量
    printf("GPIO_log: 子区域内钢筋绑扎点数量:%zu.\n", srv.response.PointCoordinatesArray.size());
    auto sortedArray = srv.response.PointCoordinatesArray;

    // 新增步骤1：过滤满足坐标范围的点（0 < world_x < 380 且 0 < world_y < 330）
    std::vector<fast_image_solve::PointCoords> filteredPoints;
    for (auto& point : sortedArray) {
    // point.World_coord[2] = 91.0;
        if (0 < (double)point.World_coord[0] && (double)point.World_coord[0] < 380 && 
            0 < (double)point.World_coord[1] && (double)point.World_coord[1] < 316
            &&0 < (double)point.World_coord[2] && (double)point.World_coord[2] < 120) 
        {   
            filteredPoints.push_back(point);
        }
    }
    
    ROS_WARN("Cur pt_vec size: %zu\n", filteredPoints.size());
    long long int total_ = 0;
    // 3：按视觉服务返回的顺序处理绑扎点
    for (size_t i = 0; i < filteredPoints.size(); i++) {

        // auto start_time = std::chrono::steady_clock::now();
        const auto& point = filteredPoints[i];
        if (send_odd_points == 1 && !(point.idx == 1 || point.idx == 4)) {
            continue;
        } else if (send_odd_points == 2 && !(point.idx == 2 || point.idx == 3)) {
            continue;
        }
        int32_t idx = point.idx;
        int32_t pix_x = point.Pix_coord[0];
        int32_t pix_y = point.Pix_coord[1];
        float_t world_x = point.World_coord[0];
        float_t world_y = point.World_coord[1];
        float_t world_z = point.World_coord[2];
        float_t angle = point.Angle;
        bool is_shuiguan = point.is_shuiguan;
    
        double module_move_x = static_cast<double>(world_x);
        double module_move_y = static_cast<double>(world_y);
        double module_move_z = static_cast<double>(world_z);
        double motor_theata = static_cast<double>(angle);
    
        printCurrentTime();
        printf("Current 线性模组移动x:%f, y:%f, z:%f, 旋转角度:%f\n", world_x, world_y, world_z, angle);
        
        // if (module_move_z < 0 || module_move_z > 120) continue;
        
        // auto first_time = std::chrono::steady_clock::now();
        moveLinearModule(world_x, world_y, world_z,motor_theata);
        // auto second_time = std::chrono::steady_clock::now();
        // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(second_time - first_time);
        // printf("纯移动的耗时为: %ld ms\n", duration2.count());
        // // 读取并打印当前姿态角度
        // {
        //     std::lock_guard<std::mutex> lock(module_state_mutex);
        //     printCurrentTime();
        //     printf("姿态: roll=%.2f° pitch=%.2f°\n",
        //         Read_Gesture(Y_GESTURE, plc),
        //         Read_Gesture(X_GESTURE, plc));

        // }
    //     // //采集数据集专用//
    //     {
    //         if (trigger_client.call(Trigger_srv)) {
    //             if (Trigger_srv.response.success) {
    //                 ROS_INFO_STREAM("触发成功: " << Trigger_srv.response.message);
    //             } else {
    //                 ROS_WARN_STREAM("触发返回失败: " << Trigger_srv.response.message);
    //             }
    //         } else {
    //             ROS_ERROR("调用 /v_vision/to_detect 服务失败");
    //         }
    //     }

    // // //采集数据集专用//
        last_x = float(world_x); last_y = float(world_y); 
        // ros::Duration(0.5).sleep();
        //执行绑扎（仅当允许绑扎时）
        if (enable_lashing) {
            ROS_WARN("lashing\n");
            
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(LASHING, 1, plc);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(LASHING, 0, plc);
            }
        }
        {
            double z_zero = 0;
            std::lock_guard<std::mutex> lock2(plc_mutex);
            Set_Module_Coordinate(WZ_COORDINATE,&z_zero,plc);
        }
        
        // auto over_time = std::chrono::steady_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(over_time - start_time);
        // printf("下一个点的耗时为: %ld ms\n", duration.count());
        // total_ += duration.count();
    }
    // std::cout << "总耗时为："<< total_ << " ms" << std::endl;
    std_msgs::Float32 debug;
    debug.data = 1.0;
    pub_lashing_finish.publish(debug);
    res.success = true;
    res.message = "区域绑扎作业完成";
    // 4：作业完成后回原点
    std::thread return_origin_thread(linear_module_move_zero_forthread,0.,0.,0.,reset_angle);
    return_origin_thread.detach();
    // linear_module_move_zero(0., 0., 0., reset_angle);
    
    return true;
}
/*************************************************************************************************************************************************************/
// 函数功能：信号捕获函数，按下ctrl+c触发，停止线性模组运动，关闭节点，并广播forced_stop消息
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void signalHandler(int signum)
{
    try{
        std::lock_guard<std::mutex> lock2(plc_mutex);
        printCurrentTime();
        printf("GPIO_log:ctrl+c已被触发,正在暂停线性模组运动并退出节点。\n");
        linear_module_move_zero(0., 0., 0., reset_angle);
        std_msgs::Bool pause_msg;
        pause_msg.data = true;
        pub_pause.publish(pause_msg);
         
        ros::shutdown();
        _exit(2);
    }
    catch (const std::exception &e) {
        printCurrentTime();
        printf("GPIO_error:线性模组操作异常。\n");
        _exit(2); //无法操作线性模组时，直接强制退出程序。
    }
}

/*************************************************************************************************************************************************************/
// 函数功能：急停的回调函数
// 函数输入：急停的信号
// 函数输出：
/*************************************************************************************************************************************************************/
void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    try{
        
        {
            handle_system_error("急停信号");
        } 

        {
            std::lock_guard<std::mutex> lock2(plc_mutex);
            PLC_Order_Write(EN_DISABLE, 0, plc); 
        }

        printCurrentTime();
        printf("GPIO_log:急停信号已被触发，正在强制暂停线性模组运动，stop节点。\n");
        
    }
    catch (const std::exception &e)  {
        printCurrentTime();
        printf("GPIO_error:线性模组操作异常。\n");
        _exit(1); //无法操作线性模组时，直接强制退出程序。
    }
}

/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，原子操作，话题触发，控制线性模组单轴回到原点。
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void linear_module_move_origin_single_one(const std_msgs::Float32 &debug_mes)
{
    // 将 float 值转换为 int
    int debug_mes1 = static_cast<int>(debug_mes.data);
    printCurrentTime();
    printf("GPIO_log:单轴返回原点请求已被触发，");
    switch (debug_mes1)
    {
        case AXIS_X:
            printf("X单轴正在回到原点。");
            break;
        case AXIS_Y:
            printf("Y单轴正在回到原点。");
            break;
        case AXIS_Z:
            printf("Z单轴正在回到原点。");
            break;
        default:
            printf("轴体参数不正确！");
            return;
    }
    // 使用整数值调用函数
    linear_module_move_origin_single(debug_mes1);
    return ;
}

/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，原子操作，话题触发，控制线性模组三轴回到原点，先Z轴回到原点，再X轴和Y轴回到原点。
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void linear_module_move_origin_all(const chassis_ctrl::linear_module_move_all::ConstPtr& msg)
{
    double x = msg->linear_module_X_distance;
    double y = msg->linear_module_Y_distance;
    double z = msg->linear_module_Z_distance;
    double z_zero = 0;
    printCurrentTime();
    printf("Gpio_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);
    if(x < 0 || x > 380)
    {
        printCurrentTime();
        printf("Gpio_Error:X轴距离设置超限。\n");
        return ;
    }
    if(y < 0 || y > 330)
    {
        printCurrentTime();
        printf("Gpio_Error:Y轴距离设置超限。\n");
        return ;
    }
    if(z<0 || z>120)
    {
        printCurrentTime();
        printf("Gpio_Error:Z轴距离设置超限。\n");
        return ;
    }

    // 先让Z轴上升，防止碰撞
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z_zero,plc);
    }
    delay_time(AXIS_Z,0,0);
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
    printCurrentTime();
    printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);

    // 如果是三轴回到原点的请求，则发布相应话题
    if(x== 0 && y== 0 && z == 0)
    {
            std_msgs::Float32 debug;
            debug.data = 1;
            pub_linear_module_gb_origin.publish(debug);
    }
    return ;
}

void linear_module_move_zero_forthread(double x, double y, double z, double angle)
{
    ros::Duration(0.4).sleep();
    // 再让x,y同时移动到目标点上方
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE,&x,plc);
        Set_Module_Coordinate(WY_COORDINATE,&y,plc);
    }
    if (x > y) delay_time(AXIS_X,x,0);
    else delay_time(AXIS_Y,y,0);
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Motor_Angle(&reset_angle, plc);
        
    }

    // 如果是三轴回到原点的请求，则发布相应话题
    if(x== 0. && y== 0. && z == 0.)
    {
            std_msgs::Float32 debug;
            debug.data = 1;
            pub_linear_module_gb_origin.publish(debug);
    }
    return ;
}
/*************************************************************************************************************************************************************/
// 函数功能：回原点
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void linear_module_move_zero(double x, double y, double z, double angle)
{
    printCurrentTime();
    printf("Gpio_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);

    // 先让Z轴上升，防止碰撞
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z,plc);
    }
    delay_time(AXIS_Z,z,0);
    ros::Duration(0.3).sleep();
    // 再让x,y同时移动到目标点上方
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WX_COORDINATE,&x,plc);
        Set_Module_Coordinate(WY_COORDINATE,&y,plc);
        
    }
    delay_time(AXIS_X,x,0);
    delay_time(AXIS_Y,y,0);
    // 旋转电机回零点
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Motor_Angle(&reset_angle, plc);
    }
    
    printCurrentTime();
    printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);

    // 如果是三轴回到原点的请求，则发布相应话题
    if(x== 0. && y== 0. && z == 0.)
    {
            std_msgs::Float32 debug;
            debug.data = 1;
            pub_linear_module_gb_origin.publish(debug);
    }
    return ;
}
/*************************************************************************************************************************************************************/
// 函数功能：线性模组顶点移动
// 函数输入：三轴坐标值
// 函数输出：
/*************************************************************************************************************************************************************/
bool linear_module_move_service(chassis_ctrl::linear_module_move::Request &req, 
    chassis_ctrl::linear_module_move::Response &res)
{
    double x = req.pos_x;
    double y = req.pos_y;
    double z = req.pos_z;
    double angle = req.angle;
    double z_zero = 0;
    // current_z=0;
    if(error_detected) {
        res.success = false;
        res.message = "暂停或者绑扎枪错误";
        return false;
    }
    printCurrentTime();
    printf("Gpio_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n",x,y,z);
    if(x < 0 || x > 380)
    {
        printCurrentTime();
        printf("Gpio_Error:X轴距离设置超限。\n");
        res.success = false;
        res.message = "X轴距离设置超限";
        return true;
    }
    if(y < 0 || y > 330)
    {
        printCurrentTime();
        printf("Gpio_Error:Y轴距离设置超限。\n");
        res.success = false;
        res.message = "Y轴距离设置超限";
        return true;
    }
    if(z<0 || z>120)
    {
        printCurrentTime();
        printf("Gpio_Error:Z轴距离设置超限。\n");
        res.success = false;
        res.message = "Z轴距离设置超限";
        return true;
    }

    // 先让Z轴上升，防止碰撞
    {
        double z_zero = 0;
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z_zero,plc);
    }
    delay_time(AXIS_Z,0,0);
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
    if (x > y) delay_time(AXIS_X,x,0);
    else delay_time(AXIS_Y,y,0);
    // 最后再让Z伸下去
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        Set_Module_Coordinate(WZ_COORDINATE,&z,plc);
    }
    
    bool is_current_z = false;
    if (arrive_z(AXIS_Z, z, is_current_z) && is_current_z)
        printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,current_z);
    else 
    printf("Gpio_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n",x,y,z);
    
    res.success = true;
    res.message = "运动完成";
    return true;
}

/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，原子操作，话题触发，更改旋转电机运动速度
// 函数输入：旋转电机角度值
// 函数输出：
/*************************************************************************************************************************************************************/
void motor_speed_change(const std_msgs::Float32 &debug_mes)
{
    motor_speed = static_cast<double>(debug_mes.data);
}

/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，原子操作，话题触发，更改线性模组运动速度
// 函数输入：旋转电机角度值
// 函数输出：
/*************************************************************************************************************************************************************/
void linear_module_speed_change(const std_msgs::Float32 &debug_mes)
{
    moduleXY_speed = static_cast<double>(debug_mes.data);
}
/*************************************************************************************************************************************************************/
// 函数功能：应用层函数，用于开闭灯光
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void light_switch(const std_msgs::Bool &debug_mes)
{
    std::lock_guard<std::mutex> lock2(plc_mutex);
    if (debug_mes.data == false) {
        printCurrentTime();
        printf("Gpio_log:灯已关闭。\n");
        PLC_Order_Write(LIGHT, 0x00, plc); // 关闭灯
    } else if (debug_mes.data == true) {
        printCurrentTime();
        printf("Gpio_log:灯已打开。\n");
        PLC_Order_Write(LIGHT, 0x01, plc); // 打开灯
    }
}
/*************************************************************************************************************************************************************/
// 函数功能：跳绑函数
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
void send_odd_points_callback(const std_msgs::Int32 &debug_mes)
{
    send_odd_points  = debug_mes.data;
}



void from_camera_get_lashing_point(const chassis_ctrl::motion &input_msg)

{
    transform_msg.p_index = 0; 
    transform_msg.p_count = input_msg.p_count;
    transform_msg.data = input_msg.data;
    if(transform_msg.p_count != 0)

    {

        printCurrentTime();

        printf("Camera_log:当前识别%d个点，",transform_msg.p_count);

        for (int i = 0;i<transform_msg.p_count; i++)

        {

            printf("第%d个点的坐标(%f,%f,%f)mm、角度:%f°)",i+1,transform_msg.data[(4*i)],transform_msg.data[(4*i+1)],transform_msg.data[(4*i+2)],transform_msg.data[(4*i+3)]);

            if (i == transform_msg.p_count - 1)

                printf("。");

            else if(i < transform_msg.p_count) 

                printf(",");

        }

        printf("\n");

    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

}

void handSolveWarnCallback(const std_msgs::Bool &warn_msg)
{
    if (warn_msg.data)
    {
        printCurrentTime();
        ROS_WARN("Hand solve warnings\n");
        {
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 1, plc); //尝试清除伺服器异常
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 0, plc);
            }

            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(EN_DISABLE, 1, plc);
            }

            {
                double z_zero = 0;
                std::lock_guard<std::mutex> lock2(plc_mutex);
                Set_Module_Coordinate(WZ_COORDINATE,&z_zero,plc);
            }
            delay_time(AXIS_Z,0,0);
        }
    }
}

/*************************************************************************************************************************************************************/
// 函数功能：线性模组模块主函数入口
// 函数输入：
// 函数输出：
/*************************************************************************************************************************************************************/
int main(int argc, char **argv) {

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moduanNode_show");
    ros::NodeHandle nh_;
    printCurrentTime();
    printf("<---moduanNode_show Started.--->\n");

    // 注册信号处理函数，捕获 SIGINT (Ctrl + C)
    signal(SIGINT, signalHandler);

// if(std::atoi(argv[1]) == 1) enable_lashing = true;
// else enable_lashing = false;
// send_odd_points = std::atoi(argv[2]);
    printf("enable_lashing: %d, 跳绑: %d\n", (int)enable_lashing, (int)send_odd_points);
    
    // 和PLC创建TCP连接
    plc=PLC_Connection();
    PLC_Order_Write(WARNING_RESET, 1, plc); // 尝试清除伺服器异常
    PLC_Order_Write(WARNING_RESET, 0, plc); // 尝试清除伺服器异常
    PLC_Order_Write(EN_DISABLE, 1, plc);
   // 设置三轴运动速度
    Set_Module_Speed(WX_SPEED, &moduleXY_speed, plc);//设置x轴速度
    Set_Module_Speed(WY_SPEED, &moduleXY_speed, plc);//设置y轴速度
    Set_Module_Speed(WZ_SPEED, &moduleZ_speed, plc);//设置z轴速度
    Set_Motor_Speed(&motor_speed, plc);
    Set_Motor_Angle(&reset_angle,plc);

    // 线性模组模块急停信号的话题
    pub_forced_stop = nh_.advertise<std_msgs::Float32>("/forced_stop", 5);
    pub_pause = nh_.advertise<std_msgs::Bool>("/pause_interrupt", 5);
    pub_linear_module_gb_origin = nh_.advertise<std_msgs::Float32>("/gpio/linear_module_gb_origin", 5);
    linear_module_move_zero(0., 0., 0., reset_angle);

    client = nh_.serviceClient<fast_image_solve::ProcessImage>("/process_image");
    // linear_client = nh_.serviceClient<chassis_ctrl::linear_module_move>("linear_module_move");
    ros::ServiceServer linear_service = nh_.advertiseService("/linear_module_move", linear_module_move_service);
    ros::ServiceServer lashing_service = nh_.advertiseService("/sg", GPIO_Lashing_Service); // 索驱到位请求绑扎末端
    trigger_client = nh_.serviceClient<std_srvs::Trigger>("/v_vision/to_detect");
    // 线性模组数据反馈的话题
    pub_linear_module_data_upload = nh_.advertise<chassis_ctrl::linear_module_upload>("/gpio/linear_module_data_upload", 5);
    // 区域绑扎完毕标志位的话题
    pub_lashing_finish = nh_.advertise<std_msgs::Float32>("/gpio/lashing_finish", 5);
    

    // 设置持续获取线性模组模块反馈
    std::thread get_module_state_thread(read_module_motor_state,&module_state,&motor_state);
    get_module_state_thread.detach();

    // 订阅是否开绑
    enb_las_sub = nh_.subscribe("/gpio/enb_las", 1, &enable_lashing_callback);
    
    // 订阅急停信息
    ros::Subscriber forced_stop = nh_.subscribe("/forced_stop", 5, &forced_stop_nodeCallback);
    ros::Subscriber hand_solve_warn = nh_.subscribe("/hand_sovle_warn", 5, &handSolveWarnCallback);
    // 订阅暂停中断信息
    ros::Subscriber interrupt0 = nh_.subscribe("/pause_interrupt", 5, &pause_interrupt_Callback);

    ros::Subscriber send_odd = nh_.subscribe("/send_odd_points", 5, &send_odd_points_callback);
    // 订阅线性模组单跳开关
    ros::Subscriber linear_module_forced_stop = nh_.subscribe("/linear_module/forced_stop", 5, &forced_stop_nodeCallback);
    //订阅camera视觉信息
    ros::Subscriber action_1 = nh_.subscribe("/camera/publisher_location", 5, &from_camera_get_lashing_point);
    // 
    
    // 订阅灯的开关指令
    ros::Subscriber light_order= nh_.subscribe("/gpio/light", 5, &light_switch);

    // 在 main 函数中初始化发布者
    pub_pause_interrupt_count = nh_.advertise<std_msgs::Int32>("/gpio/pause_interrupt_count", 5);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();


    return 0;
}
