#ifndef TIE_ROBOT_CONTROL_MODUAN_NUMERIC_CODEC_HPP
#define TIE_ROBOT_CONTROL_MODUAN_NUMERIC_CODEC_HPP

#include <cstdint>

#include <modbus.h>

modbus_t* PLC_Connection();
void Double_to_Int32(double* MyDouble);
void ggbom(double* MyDouble);
void Double_to_Double(double* MyDouble);
double Int32_to_Double();
float Read_Module_Float(int axis, modbus_t* ctx);
float Read_Module_Float_Ranged(int axis, modbus_t* ctx, float preferred_min, float preferred_max);
float Read_Module_Float_RangedScaled(int axis, modbus_t* ctx, float preferred_min, float preferred_max, float scale);
int Set_Module_Speed(int axis, double* speed, modbus_t* ctx);
int Set_Module_Coordinate(int axis, double* coordinate, modbus_t* ctx);
int PLC_Order_Write(int order, uint16_t control_word, modbus_t* ctx);
float Read_Module_Speed(int axis, modbus_t* ctx);
float Read_Module_Coordinate(int axis, modbus_t* ctx);
uint16_t Read_Module_Status(int status, modbus_t* ctx);
int Set_Motor_Speed(double* num, modbus_t* ctx);
int Set_Motor_Angle(double* num, modbus_t* ctx);
int Set_Motor_Angle(int motor_num, double* num, modbus_t* ctx);
double Read_Motor_Speed(modbus_t* ctx);
double Read_Motor_Angle(modbus_t* ctx);
float Read_Gesture(int axis, modbus_t* ctx);

#endif
