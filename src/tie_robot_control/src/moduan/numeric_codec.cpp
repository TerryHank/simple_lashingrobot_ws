#include "tie_robot_control/moduan/numeric_codec.hpp"

#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include <tie_robot_control/common.hpp>
#include "tie_robot_control/moduan/register_map.hpp"
#include "tie_robot_control/moduan/runtime_state.hpp"

using namespace tie_robot_control::moduan_registers;

namespace {

modbus_t* ensure_active_ctx(modbus_t* ctx)
{
    if (!g_moduan_driver_enabled.load()) {
        return nullptr;
    }
    if (ctx != nullptr) {
        return ctx;
    }
    if (plc == nullptr) {
        plc = PLC_Connection();
        if (plc != nullptr) {
            printCurrentTime();
            ros_log_printf("Moduan_log:PLC连接已自动重建。\n");
        }
    }
    return plc;
}

void invalidate_ctx(modbus_t* ctx)
{
    if (ctx == nullptr) {
        return;
    }
    modbus_close(ctx);
    modbus_free(ctx);
    if (ctx == plc) {
        plc = nullptr;
    }
}

float decode_modbus_float(uint16_t first_word, uint16_t second_word, bool first_word_is_high)
{
    uint32_t raw = 0;
    float value = 0.0f;
    if (first_word_is_high) {
        raw |= static_cast<uint32_t>(first_word) << 16;
        raw |= static_cast<uint32_t>(second_word);
    } else {
        raw |= static_cast<uint32_t>(second_word) << 16;
        raw |= static_cast<uint32_t>(first_word);
    }
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

bool is_preferred_float(float value, float preferred_min, float preferred_max)
{
    return std::isfinite(value) && value >= preferred_min && value <= preferred_max;
}

float choose_preferred_float(
    int axis,
    uint16_t first_word,
    uint16_t second_word,
    float preferred_min,
    float preferred_max,
    float scale)
{
    const float low_word_first = decode_modbus_float(first_word, second_word, false);
    const float high_word_first = decode_modbus_float(first_word, second_word, true);
    const float scaled_low_word_first = low_word_first * scale;
    const float scaled_high_word_first = high_word_first * scale;

    const float preferred_candidates[] = {
        high_word_first,
        low_word_first,
        scaled_high_word_first,
        scaled_low_word_first,
    };
    for (float candidate : preferred_candidates) {
        if (is_preferred_float(candidate, preferred_min, preferred_max)) {
            return candidate;
        }
    }
    if (std::isfinite(high_word_first) && !std::isfinite(low_word_first)) {
        return high_word_first;
    }
    if (std::isfinite(low_word_first) && !std::isfinite(high_word_first)) {
        return low_word_first;
    }
    printCurrentTime();
    ros_log_printf(
        "Moduan_Warn:浮点寄存器%d超出期望范围，原始字=[%u,%u] low_first=%.6f high_first=%.6f scaled_low=%.6f scaled_high=%.6f range=[%.2f,%.2f]\n",
        axis,
        static_cast<unsigned>(first_word),
        static_cast<unsigned>(second_word),
        low_word_first,
        high_word_first,
        scaled_low_word_first,
        scaled_high_word_first,
        preferred_min,
        preferred_max
    );
    return high_word_first;
}

float choose_preferred_scaled_float(
    int axis,
    uint16_t first_word,
    uint16_t second_word,
    float preferred_min,
    float preferred_max,
    float scale)
{
    const float low_word_first = decode_modbus_float(first_word, second_word, false);
    const float high_word_first = decode_modbus_float(first_word, second_word, true);
    const float scaled_low_word_first = low_word_first * scale;
    const float scaled_high_word_first = high_word_first * scale;

    const float preferred_candidates[] = {
        scaled_low_word_first,
        scaled_high_word_first,
        low_word_first,
        high_word_first,
    };
    for (float candidate : preferred_candidates) {
        if (is_preferred_float(candidate, preferred_min, preferred_max)) {
            return candidate;
        }
    }
    printCurrentTime();
    ros_log_printf(
        "Moduan_Warn:缩放浮点寄存器%d超出期望范围，原始字=[%u,%u] low_first=%.6f high_first=%.6f scaled_low=%.6f scaled_high=%.6f range=[%.2f,%.2f]\n",
        axis,
        static_cast<unsigned>(first_word),
        static_cast<unsigned>(second_word),
        low_word_first,
        high_word_first,
        scaled_low_word_first,
        scaled_high_word_first,
        preferred_min,
        preferred_max
    );
    return scaled_low_word_first;
}

}  // namespace

modbus_t* PLC_Connection()
{
    if (!g_moduan_driver_enabled.load()) {
        return NULL;
    }
    modbus_t *ctx = modbus_new_tcp("192.168.6.167", 502);
    if (ctx == NULL) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:未能创建modbus tcp上下文。\n");
        return NULL;
    }

    uint32_t timeout_sec = 30;
    uint32_t timeout_usec = 0;
    modbus_set_response_timeout(ctx, timeout_sec, timeout_usec);

    int rc = modbus_connect(ctx);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:PLC连接失败: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return NULL;
    }
    return ctx;
}

void Double_to_Int32(double* MyDouble) {
    int32_t temp32 = static_cast<int32_t>(*MyDouble * 100);
    DtoU_Register[1] = (temp32 & 0xFFFF0000) >> 16;
    DtoU_Register[0] = temp32 & 0x0000FFFF;
}

void ggbom(double* MyDouble) {
    int32_t temp32 = static_cast<int32_t>(*MyDouble);
    DtoU_Register[1] = (temp32 & 0xFFFF0000) >> 16;
    DtoU_Register[0] = temp32 & 0x0000FFFF;
}

void Double_to_Double(double* MyDouble) {
    int32_t temp32 = static_cast<int32_t>(*MyDouble);
    DtoU_Register[1] = (temp32 & 0xFF00) >> 8;
    DtoU_Register[0] = temp32 & 0x00FF;
}

double Int32_to_Double() {
    int32_t temp32 = 0;
    temp32 |= ((int32_t)UtoD_Register[1]) << 16;
    temp32 |= (int32_t)UtoD_Register[0];
    return static_cast<double>(temp32) / 100.0;
}

float Read_Module_Float(int axis, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, axis, 2, UtoF_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return decode_modbus_float(UtoF_Register[0], UtoF_Register[1], false);
}

float Read_Module_Float_Ranged(int axis, modbus_t* ctx, float preferred_min, float preferred_max)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, axis, 2, UtoF_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return choose_preferred_float(
        axis,
        UtoF_Register[0],
        UtoF_Register[1],
        preferred_min,
        preferred_max,
        1.0f
    );
}

float Read_Module_Float_RangedScaled(int axis, modbus_t* ctx, float preferred_min, float preferred_max, float scale)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, axis, 2, UtoF_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取浮点寄存器失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return choose_preferred_scaled_float(
        axis,
        UtoF_Register[0],
        UtoF_Register[1],
        preferred_min,
        preferred_max,
        scale
    );
}

int Set_Module_Speed(int axis, double* speed, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:设置模组运动速度失败: PLC未连接\n");
        return -1;
    }
    Double_to_Int32(speed);
    int rc = modbus_write_registers(active_ctx, axis, 2, DtoU_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:设置模组运动速度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return 0;
}

int Set_Module_Coordinate(int axis, double* coordinate, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:设置模组运动坐标失败: PLC未连接\n");
        return -1;
    }
    Double_to_Int32(coordinate);
    int rc = modbus_write_registers(active_ctx, axis, 2, DtoU_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:设置模组运动坐标失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return 0;
}

int PLC_Order_Write(int order, uint16_t control_word, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:写入命令失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_write_register(active_ctx, order, control_word);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:写入命令失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
    } else {
        printCurrentTime();
        ros_log_printf("Moduan_log:已成功向寄存器写入命令。\n");
    }
    return 0;
}

float Read_Module_Speed(int axis, modbus_t* ctx)
{
    double speed;
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组运动速度失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, axis, 2, UtoD_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组运动速度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    speed = Int32_to_Double();
    return speed;
}

float Read_Module_Coordinate(int axis, modbus_t* ctx)
{
    double coordinate;
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组当前坐标失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, axis, 2, UtoD_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组当前坐标失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    coordinate = Int32_to_Double();
    return coordinate;
}

uint16_t Read_Module_Status(int status, modbus_t* ctx)
{
    uint16_t status_word;
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组命令失败： PLC未连接\n");
        return static_cast<uint16_t>(-1);
    }
    int rc = modbus_read_registers(active_ctx, status, 1, &status_word);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Moduan_Error:读取模组命令失败： %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return static_cast<uint16_t>(-1);
    }
    return status_word;
}

int Set_Motor_Speed(double* num, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机速度失败: PLC未连接\n");
        return -1;
    }
    Double_to_Double(num);
    int rc = modbus_write_registers(active_ctx, WRITING_RMOTOR_SPEED, 2, DtoU_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机速度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return 0;
}

int Set_Motor_Angle(double* num, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机角度失败: PLC未连接\n");
        return -1;
    }
    ggbom(num);
    int rc = modbus_write_registers(active_ctx, WRITING_ANGLE, 2, DtoU_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机角度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return 0;
}

int Set_Motor_Angle(int motor_num, double* num, modbus_t* ctx)
{
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机角度失败: PLC未连接\n");
        return -1;
    }
    ggbom(num);
    int rc = modbus_write_registers(active_ctx, motor_num, 2, DtoU_Register);
    if (rc == -1) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:设置旋转电机角度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    return 0;
}

double Read_Motor_Speed(modbus_t* ctx)
{
    double output;
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:读取旋转电机速度失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, READING_RMOTOR_SPEED, 2, UtoD_Register);
    if (rc == -1) {
        ros_log_printf( "Motor_Error:读取旋转电机速度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    output = Int32_to_Double();
    return output;
}

double Read_Motor_Angle(modbus_t* ctx)
{
    double output;
    modbus_t* active_ctx = ensure_active_ctx(ctx);
    if (active_ctx == nullptr) {
        printCurrentTime();
        ros_log_printf( "Motor_Error:读取旋转电机角度失败: PLC未连接\n");
        return -1;
    }
    int rc = modbus_read_registers(active_ctx, READING_ANGLE, 2, UtoD_Register);
    if (rc == -1) {
        ros_log_printf( "Motor_Error:读取旋转电机角度失败: %s\n", modbus_strerror(errno));
        invalidate_ctx(active_ctx);
        return -1;
    }
    output = Int32_to_Double() * 100;
    return output;
}

float Read_Gesture(int axis, modbus_t* ctx)
{
    return Read_Module_Float(axis, ctx);
}
