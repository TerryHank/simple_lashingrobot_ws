#include <ctime>  
#include <cmath> 
#include <thread>
#include <vector>
#include <fcntl.h>
#include <iomanip> // 用于std::put_time 
#include <stdio.h>
#include <csignal> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <json.hpp>
#include <iostream>  
#include <cctype>
#include <pthread.h>
#include <ros/ros.h>
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <omp.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <chassis_ctrl/motion.h>
#include "chassis_ctrl/cabin_upload.h" //用于上传索驱模块反馈
#include "chassis_ctrl/MotionControl.h"
#include <std_msgs/Float32MultiArray.h>
#include "chassis_ctrl/cabin_move_all.h"
#include "chassis_ctrl/cabin_move_single.h"
#include "chassis_ctrl/cabin_calibration.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_upload.h"
#include <fast_image_solve/ProcessImage.h>
#include "fast_image_solve/PointCoords.h"
#include <fast_image_solve/ProcessImage.h>
#include "chassis_ctrl/area_choose.h"
#include "std_srvs/Trigger.h" 
#include <chassis_ctrl/Pathguihua.h>
#include <chassis_ctrl/SingleMove.h>
#include "common.hpp"

using namespace std;

std::string path_points_json_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json";

#define AXIS_X 0 //索驱轴体代表宏定义
#define AXIS_Y 3
#define AXIS_Z 4

// 定义互斥锁来保护 cabin_state 的读写操作
std::mutex cabin_state_mutex;
// 定义互斥锁避免同一时刻对TCP写入产生粘包现象
std::mutex socket_mutex;

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

// 由计算模块得出的索驱路径点的X序列
struct Cabin_Point
{
    float x;
    float y;  
};

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

// 索驱状态被发布的全局变量
chassis_ctrl::cabin_upload cabin_data_upload;
fast_image_solve::ProcessImage srv;
std_srvs::Trigger Trigger_srv;
chassis_ctrl::MotionControl motion_srv;
ros::ServiceClient AI_client;
ros::ServiceClient sg_client;
ros::ServiceClient save_image_client;
ros::ServiceClient motion_client;
// 暂停中断标志位 0为未启用暂停中断或恢复，1为启用暂停中断
int handle_pause_interrupt = 0;
std::mutex error_msg;

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

// 索驱全局移动函数声明：实现连续移动
void cabin_global_discontinuous_move(std::vector<Cabin_Point>& Cabin_Coor,float cabin_height,float cabin_speed);
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

float calculatePID(PID_Controller* pid, float setpoint, float feedback) {
    float error = setpoint - feedback;
    
    if(fabs(error) < 0.5f) {
        return 0.0f;
    }
    
    // 积分累积
    pid->integral += error;
    
    // 积分限位
    if(pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if(pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    
    // 增量式PID计算
    float delta_output = pid->Kp * (error - pid->last_error) +
                        pid->Ki * error +
                        pid->Kd * (error - 2 * pid->last_error + pid->prev_error);
    
    // 计算当前输出
    float output = pid->prev_output + delta_output;
    if(output > 9.5) {
        output = 9.5;
    } else if(output < -9.5) {
        output = -9.5;
    }
    // 更新状态
    pid->prev_error = pid->last_error;
    pid->last_error = error;
    pid->prev_output = output;
    
    // 打印调试信息
    printf("增量式PID调试 - 设定值: %.2f, 反馈值: %.2f, 误差: %.2f\n", 
           setpoint, feedback, error);
    printf("积分累积: %.2f (限位: %.2f~%.2f)\n", 
           pid->integral, pid->integral_min, pid->integral_max);
    printf("增量输出: %.2f (P: %.2f, I: %.2f, D: %.2f)\n",
           delta_output, 
           pid->Kp * (error - pid->last_error),
           pid->Ki * error,
           pid->Kd * (error - 2 * pid->last_error + pid->prev_error));
    
    return output;
}

float degreesToRadians(const float& degrees) {
    return degrees * (M_PI / 180.0);
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
    const int TIMEOUT_SEC = 5;
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

    timeval timeout{TIMEOUT_SEC, 0};
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

// 定义超时和缓冲区大小，避免 Magic Number
constexpr int TCP_TIMEOUT_SEC = 5;
constexpr int RECV_BUFFER_SIZE = 256;
constexpr int WRITE_DELAY_MS = 300;

int Frame_Generate(uint8_t* Control_Word, int Tlen, int Rlen, int socket = sockfd) 
{
    // 1. 参数检查
    if (Control_Word == nullptr || Tlen <= 0 || socket < 0) {
        printf("Cabin_Error: Invalid input parameters.\n");
        return -1;
    }

    // 2. 写操作带超时控制 (Select)
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(socket, &writefds);

    struct timeval timeout{TCP_TIMEOUT_SEC, 0};

    int ret = select(socket + 1, nullptr, &writefds, nullptr, &timeout);
    if (ret <= 0) {
        printf("Cabin_Error: TCP write select failed or timeout.\n");
        return -1;
    }

    // 3. 循环发送，确保所有数据发出
    ssize_t total_sent = 0;
    while (total_sent < Tlen) {
        ssize_t sent = send(socket, Control_Word + total_sent, Tlen - total_sent, 0);
        if (sent < 0) {
            printf("Cabin_Error: TCP send failed.\n");
            return -1;
        }
        total_sent += sent;
    }

    // 4. 写后延时（根据设备协议决定是否需要）
    std::this_thread::sleep_for(std::chrono::milliseconds(WRITE_DELAY_MS));

    // 5. 读操作带超时控制
    uint8_t buffer[RECV_BUFFER_SIZE];
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket, &readfds);

    ret = select(socket + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret <= 0) {
        printf("Cabin_Error: TCP read select failed or timeout.\n");
        return -1;
    }

    // 6. 循环接收或限制最大接收长度
    ssize_t recv_len = recv(socket, buffer, sizeof(buffer) - 1, 0); // 预留1字节给'\0'
    if (recv_len < 0) {
        printf("Cabin_Error: TCP recv failed.\n");
        return -1;
    }
    
    // 安全添加字符串结束符
    buffer[recv_len] = '\0';

    // 7. 状态缓存逻辑（建议后续改为指令ID判断）
    // 假设 TCP_Normal_Connection 是一个特定的指令码，而不是指针
    if (memcmp(Control_Word, TCP_Normal_Connection, Tlen) == 0) {
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
                    _exit(4); //直接发布急停信号并关闭节点
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
    _exit(4); //直接发布急停信号并关闭节点
}


/*************************************************************************************************************************************************************
功能: 驱动层函数，延时，等待索驱某个轴运动到指定位置
输入: 轴 位置
输出:
**************************************************************************************************************************************************************/
void delay_time(int Axis, double Target_position)
{
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
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
        if (handle_pause_interrupt == 1) //如果暂停中断被触发
        {
            printCurrentTime();
            printf("Cabin_log:索驱暂停中断已被触发。\n");
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                Frame_Generate_With_Retry(TCP_stop, 8, 8); //先暂停索驱运动
            }
            
            // 等待恢复信号
            bool stop_flag = false;
            while (handle_pause_interrupt == 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 添加延时，避免CPU空转
                {
                    std::lock_guard<std::mutex> lock1(error_msg);
                    stop_flag = (bool)handle_pause_interrupt;
                }
            }
            
            // 收到恢复信号
            printCurrentTime();
            printf("Cabin_log:索驱暂停中断已解除。\n");
            
            // 重新生成运动指令
            {
                std::lock_guard<std::mutex> lock2(socket_mutex);
                // 根据当前轴和目标位置重新生成运动指令
                if(Axis == AXIS_X) {
                    TCP_Move[1] = Target_position;
                }
                else if(Axis == AXIS_Y) {
                    TCP_Move[2] = Target_position;
                }
                else if(Axis == AXIS_Z) {
                    TCP_Move[3] = Target_position;
                }
                moveTCPPosition(0x01, TCP_Move);  // 假设这个函数用于更新TCP_Move_Frame
                Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8); //恢复索驱继续运动
            }
        }
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

// 接收视觉发布的绑扎点信息 
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

// 索驱全局移动函数
void cabin_global_discontinuous_move(std::vector<Cabin_Point>& Cabin_Coor,float cabin_height,float cabin_speed)
{
    printf("Cabin_log: Cabin_Coor数组数量（路径点个数）: %zu\n", Cabin_Coor.size());
    float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
    int pt_num = 0;
    auto it = Cabin_Coor.begin();
    while (it != Cabin_Coor.end()) 
    {
        pt_num++;
        const auto& Coor_point = *it;

        TCP_Move[0] = cabin_speed;
        TCP_Move[1] = Coor_point.x;
        TCP_Move[2] = Coor_point.y;
        TCP_Move[3] = cabin_height;
        // TCP_Move[4] = 0;
        // TCP_Move[5] = 0;
        // TCP_Move[6] = 0;
        printf("Coor_point.x: %f,Coor_point.y: %f, Coor_point.height: %f\n", TCP_Move[1], TCP_Move[2],TCP_Move[3]);

        printf("Cabin_log:索驱下个绑扎区域坐标点已更新。\n");
        t1 = omp_get_wtime();
        {
            std::lock_guard<std::mutex> lock2(socket_mutex);
            moveTCPPosition(0x01,TCP_Move);
            Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
        }
        delay_time(AXIS_X,Coor_point.x);
        delay_time(AXIS_Y,Coor_point.y);
        delay_time(AXIS_Z,cabin_height);
        t2 = omp_get_wtime();
    
        t3 = omp_get_wtime();
    // ros::Duration(0.7).sleep(); //等待索驱彻底停稳
        t4 = omp_get_wtime();

        float min_distance = FLT_MAX;
        t5 = omp_get_wtime();
        // 进入当前区域绑扎周期
    // if (AI_client.call(srv)) {
    //     height_sum = 0;
    //     height_avg = 0;
    //     point_count = 0;  

    //     for (const auto& point : srv.response.PointCoordinatesArray) {
    //         // 访问PointCoords的成员
    //         int32_t idx = point.idx;                     // 索引
    //         int32_t pix_x = point.Pix_coord[0];          // 像素坐标X
    //         int32_t pix_y = point.Pix_coord[1];          // 像素坐标Y
    //         float_t world_x = point.World_coord[0];    // 世界坐标X
    //         float_t world_y = point.World_coord[1];    // 世界坐标Y 
    //         float_t world_z = point.World_coord[2];    // 世界坐标Z
    //         float_t angle = point.Angle;               // 角度
    //         bool is_shuiguan = point.is_shuiguan;        // 是否有水管
    //         if ( world_z >= 0 )
    //         {
    //             height_sum += world_z;
    //             point_count++;  
    //         }
    //     }
            t6 = omp_get_wtime();

        // if (point_count != 0) {
        //     height_avg = height_sum/point_count;
        //     printf("平均高度:%f\n,初始高度:%f\n",height_avg,cabin_height);

        t7 = omp_get_wtime();
        //     {
        //         std::lock_guard<std::mutex> lock2(socket_mutex);
        //         // TCP_Move[1]=Coor_point.x+nearest_x;
        //         // TCP_Move[2]=Coor_point.y+nearest_y;
        //         TCP_Move[3]=cabin_height+(85-height_avg);
        //         if (TCP_Move[3]<-549){
        //             TCP_Move[3]=-549;
        //         }
        //         printf("Cabin_log: TCP_Move[1]  = %f\n", TCP_Move[1]);
        //         printf("Cabin_log: TCP_Move[2]  = %f\n", TCP_Move[2]);
        //         printf("Cabin_log: TCP_Move[3]  = %f\n", TCP_Move[3]);
        //         moveTCPPosition(0x01,TCP_Move);
        //         Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8); 
        //     } 
        //     delay_time(AXIS_X,TCP_Move[1]);
        //     delay_time(AXIS_Y,TCP_Move[2]);
        //     delay_time(AXIS_Z,TCP_Move[3]);
                t8 = omp_get_wtime();

                t9= omp_get_wtime();
                ros::Duration(0.2).sleep();
                t10 = omp_get_wtime();
                printf("开始视觉识别\n");
            if (sg_client.call(Trigger_srv)) {
              if (Trigger_srv.response.success) {
                    // 服务调用和响应均成功，执行放行逻辑
                    printf("绑扎完成\n");
                } else {
                    printf("Cabin_Error:调用sg服务失败，消息：%s\n", Trigger_srv.response.message.c_str());
                }
            } else {
                    ROS_ERROR("Failed to call pointAI service\n");    
            }
            it = Cabin_Coor.erase(it);  // 关键修改：删除当前元素并更新迭代器

            printCurrentTime();
            float t_12 = t2-t1;
            float t_34 = t4-t3;
            float t_56 = t6-t5;
            float t_78 = t8-t7;
            float t_910 = t10-t9;
            printf("第%d路径点:位置到位耗时 %f 秒， 自适应高度前稳定耗时 %f 秒，自适应高度识别请求耗时 %f 秒，自适应高度执行耗时 %f 秒，绑扎前稳定耗时 %f 秒\n", pt_num, t_12, t_34, t_56, t_78, t_910);           
        
    }
    printCurrentTime();
    printf("Cabin_log:全部路径点位绑扎完成，本次绑扎作业结束。\n");
    return;
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

/*
    函数功能：急停的回调函数,收到急停信号后，强制停止索驱运动，关闭节点。
*/
void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    printf("Cabin_log:急停信号已被触发，正在强制停止索驱运动，关闭节点。\n");
    try
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        Frame_Generate_With_Retry(TCP_stop, 8, 8);
        close(sockfd);
        _exit(1);  //索驱急停成功后退出程序
    }
    catch(const std::exception& e)
    {
        // 直接强制退出程序
        _exit(1);
    }
}

/*
    函数功能：暂停中断的回调函数,收到暂停中断信号后，设置暂停中断标志位为1，等待人工恢复。
*/
void pause_interrupt_Callback(const std_msgs::Bool &debug_mes)
{
    if (debug_mes.data)
    {
        printCurrentTime();
        printf("Cabin_log: 索驱人工暂停，正在等待人工恢复。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 1;
        }
    }
    else {
        printCurrentTime();
        printf("Cabin_log: 索驱人工恢复，可以继续运动。\n");
        {
            std::lock_guard<std::mutex> lock1(error_msg);
            handle_pause_interrupt = 0;
        }
    }
    return;
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
    if (cabin_height < 200) {
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
*/
void atom_cabin_move_single(const chassis_ctrl::cabin_move_single::ConstPtr& msg)
{
    char C;
    if(msg->Axis ==AXIS_X && (msg->target_distance < -1500 || msg->target_distance > 1500))
    {
        printCurrentTime();
        printf("Cabin_Error:索驱X轴距离设置超限。\n");
        return ;
    }
    if(msg->Axis ==AXIS_Y && (msg->target_distance < -3000 || msg->target_distance > 3000))
    {
        printCurrentTime();
        printf("Cabin_Error:索驱Y轴距离设置超限。\n");
        return ;
    }
    if(msg->Axis ==AXIS_Z && (msg->target_distance < -60 || msg->target_distance > 1200))
    {
        printCurrentTime();
        printf("Cabin_Error:索驱Z轴距离设置超限。\n");
        return ;
    }
    
    {
        std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        TCP_Move[1] = cabin_state.X;
        TCP_Move[2] = cabin_state.Y;
        TCP_Move[3] = cabin_state.Z;
    }

    printCurrentTime();
    printf("Cabin_log:索驱单轴运动请求已被触发，");
    if(msg->Axis == AXIS_X)
    {
        printf("X轴正在移动至%lfmm处。\n",msg->target_distance);
        TCP_Move[1] = msg->target_distance;
        C= 'X';
    }
    if(msg->Axis == AXIS_Y)
    {
        printf("Y轴正在移动至%lfmm处。\n",msg->target_distance);
        TCP_Move[2] = msg->target_distance;
        C= 'Y';
    }
    if(msg->Axis == AXIS_Z)
    {
        printf("Z轴正在移动至%lfmm处。\n",msg->target_distance);
        TCP_Move[3] = msg->target_distance;
        C= 'Z';
    }
    else
    {
        printf("轴体参数不正确。\n");
        return ;
    }
    // 移动索驱单轴
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01,TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(msg->Axis,msg->target_distance);
    printCurrentTime();
    printf("Cabin_log:索驱%C轴已移动至目标位置%lfmm处。\n",C,msg->target_distance);
    return ;
}

/*
    函数功能：索驱三轴运动调试
*/
void atom_cabin_move_all(const chassis_ctrl::cabin_move_all::ConstPtr& msg)
{
    if(msg->cabin_X_distance < -1500 || msg->cabin_X_distance > 1500)
    {
        printCurrentTime();
        printf("Cabin_Error:索驱X轴距离设置超限。\n");
        return ;
    }
    if(msg->cabin_Y_distance < -3000 || msg->cabin_Y_distance > 3000)
    {
        printCurrentTime();
        printf("Cabin_Error:索驱Y轴距离设置超限。\n");
        return ;
    }
    if(msg->cabin_Z_distance < -60 || msg->cabin_Z_distance > 1200)
    {
        printCurrentTime();
        printf("Cabin_Error:索驱Z轴距离设置超限。\n");
        return ;
    }
    
    printCurrentTime();
    printf("Cabin_log:索驱正在移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
    
    // 先将Z轴回到原点处
    {
        std::lock_guard<std::mutex> lock1(cabin_state_mutex);
        TCP_Move[1] = cabin_state.X;
        TCP_Move[2] = cabin_state.Y;
    }
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        TCP_Move[3] = 0;
        moveTCPPosition(0x01,TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(AXIS_Z,0);
    // 当Z轴回到原点后，再移动X轴和Y轴
    TCP_Move[1] = msg->cabin_X_distance;
    TCP_Move[2] = msg->cabin_Y_distance;
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01,TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(AXIS_X,msg->cabin_X_distance);
    delay_time(AXIS_Y,msg->cabin_Y_distance);
    // 当X,Y轴抵达位置后，再移动Z轴运动
    TCP_Move[3] = msg->cabin_Z_distance;
    {
        std::lock_guard<std::mutex> lock2(socket_mutex);
        moveTCPPosition(0x01,TCP_Move);
        Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);
    }
    delay_time(AXIS_Z,msg->cabin_Z_distance);
    printCurrentTime();
    printf("Cabin_log:索驱已移动至目标点(%lf,%lf,%lf)。\n",msg->cabin_X_distance,msg->cabin_Y_distance,msg->cabin_Z_distance);
    return;
}

// 订阅外部姿态传感器计算的索驱姿态
void cabin_gesture_get(const chassis_ctrl::linear_module_upload& msg)
{
    cabin_state.cabin_x_gesture = msg.x_gesture;
    cabin_state.cabin_y_gesture = msg.y_gesture;
}


bool startGlobalWork(chassis_ctrl::MotionControl::Request &req,
                            chassis_ctrl::MotionControl::Response &res) {
    printCurrentTime();
    printf("Cabin_log: 收到%s\n", req.command.c_str());
    
    // 读取保存路径点的JSON文件（之前由planGlobalMovePath生成的path_points.json）
    std::ifstream infile(path_points_json_file);
    if (!infile.is_open()) {
        throw std::runtime_error("无法打开路径点JSON文件");
    }
    
    nlohmann::json path_json;
    infile >> path_json;

    // 验证JSON结构是否包含路径点数组
    if (!path_json.contains("path_points") || !path_json["path_points"].is_array()) {
        throw std::runtime_error("JSON文件格式错误，缺少'path_points'数组");
    }

    // 验证JSON结构是否包含cabin_height和cabin_speed
    if (!path_json.contains("cabin_height") || !path_json.contains("cabin_speed")) {
        throw std::runtime_error("JSON文件格式错误，缺少'cabin_height'或'cabin_speed'");
    }
    // 从JSON中读取cabin_height和cabin_speed
    float cabin_height = path_json["cabin_height"].get<float>();
    float cabin_speed = path_json["cabin_speed"].get<float>();

    // 提取当前路径
    std::vector<Cabin_Point> con_path;
    con_path = new_path;
    if (con_path.empty())
    {
        printCurrentTime();
        printf("重复历史路径！\n");
        for (const auto& json_point : path_json["path_points"]) {
            Cabin_Point point;
            point.x = json_point["x"].get<float>();  // 提取x坐标
            point.y = json_point["y"].get<float>();  // 提取y坐标
            con_path.push_back(point);
        }
    }

    // 直接使用解析出的路径点，传入完整参数
    cabin_global_discontinuous_move(con_path, cabin_height, cabin_speed);

    res.success = true;
    res.message = "索驱已将全部点位绑扎完成";
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
                _exit(3); 
            }
            catch(const std::exception& e)
            {
                // 直接强制退出程序
                _exit(3);
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
                _exit(3); 
            }
            catch(const std::exception& e)
            {
                // 直接强制退出程序
                _exit(3);
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
    ros::init(argc, argv, "suoquNode");
    ros::NodeHandle nh;
    printCurrentTime();
    printf("<--- suoquNode Started --->\n");
    
    suoquInit();

    // 注册信号处理函数，捕获 SIGINT (Ctrl + C)
    signal(SIGINT, signalHandler_cc);
    // ros::ServiceServer service = nh.advertiseService("motion_control", handleMotionRequest);
    // motion_client = nh.serviceClient<chassis_ctrl::MotionControl>("motion_control");
    // 全局路径规划
    ros::ServiceServer path_config_service = nh.advertiseService("/cabin/plan_path", planGlobalMovePath);
    // 设置索驱速度
    ros::Subscriber change_cabin_speed_sub = nh.subscribe("/web/cabin/set_cabin_speed", 5, &change_cabin_speed_callback);
    // 开始全局作业
    ros::ServiceServer update_service = nh.advertiseService("/cabin/start_work", startGlobalWork);
    // 单点运动调试
    ros::ServiceServer cabin_single_move_service = nh.advertiseService("/cabin/single_move", cabin_single_move);
    // 订阅视觉模块的消息
    AI_client = nh.serviceClient<fast_image_solve::ProcessImage>("/pointAI/process_image");
    // 请求末端开始
    sg_client = nh.serviceClient<std_srvs::Trigger>("/moduan/sg");
    // save_image_client = nh.serviceClient<std_srvs::Trigger>("/save_depth_image");
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
    ros::Subscriber forced_stop_sub = nh.subscribe("/cabin/forced_stop", 5, &forced_stop_nodeCallback);
    // 订阅暂停中断信息（通过末端发送）
    ros::Subscriber interrupt0_sub = nh.subscribe("/cabin/pause_interrupt", 5, &pause_interrupt_Callback);
    // 订阅末端安装的姿态传感器数据 （通过末端发送）
    // ros::Subscriber cabin_gesture = nh.subscribe("/moduan/moduan_gesture_data", 5, &cabin_gesture_get);
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
