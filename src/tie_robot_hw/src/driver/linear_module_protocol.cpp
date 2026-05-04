#include "tie_robot_hw/driver/linear_module_protocol.hpp"

#include <cstring>

namespace tie_robot_hw {
namespace driver {

namespace {

uint16_t lowerScaledWord(float value)
{
    int32_t scaled = static_cast<int32_t>(value * 100.0f);
    return static_cast<uint16_t>(scaled & 0xFFFF);
}

uint16_t upperScaledWord(float value)
{
    int32_t scaled = static_cast<int32_t>(value * 100.0f);
    return static_cast<uint16_t>((scaled >> 16) & 0xFFFF);
}

uint16_t lowerRawWord(float value)
{
    int32_t raw = static_cast<int32_t>(value);
    return static_cast<uint16_t>(raw & 0xFFFF);
}

uint16_t upperRawWord(float value)
{
    int32_t raw = static_cast<int32_t>(value);
    return static_cast<uint16_t>((raw >> 16) & 0xFFFF);
}

}  // namespace

int LinearModuleProtocol::pointBaseRegister(int slot_index)
{
    return 5510 + slot_index * 10;
}

int LinearModuleProtocol::pointAngleRegister(int slot_index)
{
    return pointBaseRegister(slot_index) + 6;
}

std::vector<uint16_t> LinearModuleProtocol::buildPointRegisterPayload(const LinearModulePoint& point)
{
    return {
        lowerScaledWord(point.x_mm), upperScaledWord(point.x_mm),
        lowerScaledWord(point.y_mm), upperScaledWord(point.y_mm),
        lowerScaledWord(point.z_mm), upperScaledWord(point.z_mm),
        lowerRawWord(point.angle_deg), upperRawWord(point.angle_deg),
    };
}

}  // namespace driver
}  // namespace tie_robot_hw
