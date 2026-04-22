#include "tie_robot_hw/driver/linear_module_protocol.hpp"

#include <cstring>

namespace tie_robot_hw {
namespace driver {

namespace {

uint16_t lowerWord(float value)
{
    int32_t scaled = static_cast<int32_t>(value * 100.0f);
    return static_cast<uint16_t>(scaled & 0xFFFF);
}

uint16_t upperWord(float value)
{
    int32_t scaled = static_cast<int32_t>(value * 100.0f);
    return static_cast<uint16_t>((scaled >> 16) & 0xFFFF);
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
        lowerWord(point.x_mm), upperWord(point.x_mm),
        lowerWord(point.y_mm), upperWord(point.y_mm),
        lowerWord(point.z_mm), upperWord(point.z_mm),
        lowerWord(point.angle_deg), upperWord(point.angle_deg),
    };
}

}  // namespace driver
}  // namespace tie_robot_hw
