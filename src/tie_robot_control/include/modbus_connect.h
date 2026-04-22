// modbus_connection.h
#ifndef MODBUS_CONNECTION_H
#define MODBUS_CONNECTION_H


#include <modbus.h>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <chrono>
#include <thread>

void delay(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
#define LEFT_CTLBIT 6474 // 左轮控制字(单字)
#define LEFT_MODE 6475 // 左轮模式设置(单字)
#define LEFT_SPEED 6476 // 左轮速度给定
#define LEFT_MAX_SPEED 6478 // 左轮最大速度给定
#define LEFT_ACC 6480 // 左轮加速度
#define LEFT_DEC 6482 // 左轮减速度
#define LEFT_STABIT 6030 // 左轮状态字(单字)
#define LEFT_FBMODE 6031 // 左轮模式反馈(单字)
#define LEFT_FBPOS 6032// 左轮位置反馈
#define LEFT_FBVEL 6034 // 左轮速度反馈
// 右轮
#define RIGHT_CTLBIT 6494 // 右轮控制字(单字)
#define RIGHT_MODE 6495 // 右轮模式设置(单字)
#define RIGHT_SPEED 6496 // 右轮速度给定
#define RIGHT_MAX_SPEED 6498 // 右轮最大速度给定
#define RIGHT_ACC 6500 // 右轮加速度
#define RIGHT_DEC 6502 // 右轮减速度
#define RIGHT_STABIT 6040 // 右轮状态字(单字)
#define RIGHT_FBMODE 6041 // 右轮模式反馈(单字)
#define RIGHT_FBPOS 6042// 右轮位置反馈
#define RIGHT_FBVEL 6044 // 右轮速度反馈
#define control_light 6380
#define  control_word_value1  0x06
#define  control_word_value2  0x07
#define  control_word_value3  0xAFF


double left_wheel_speed= 0;
double right_wheel_speed= 0;
modbus_t* PLC_Connection();
extern modbus_t* plc; // 声明全局变量plc
void printCurrentTime();
void Double_to_Int32(double* MyDouble);
void ggbom(double* MyDouble);
double Int32_to_Double();
int PLC_Order_Write(int order, uint16_t control_word, modbus_t* ctx);
int Set_Module_Speed(int axis, double* speed, modbus_t* ctx);
// 全局变量声明
extern uint16_t DtoU_Register[2];
extern uint16_t UtoD_Register[2];
#endif // MODBUS_CONNECTION_H