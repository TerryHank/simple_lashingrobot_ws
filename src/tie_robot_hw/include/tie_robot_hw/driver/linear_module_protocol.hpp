#pragma once

#include <cstdint>
#include <vector>

#include "tie_robot_hw/driver/driver_types.hpp"

namespace tie_robot_hw {
namespace driver {

class LinearModuleProtocol {
public:
    static std::vector<uint16_t> buildPointRegisterPayload(const LinearModulePoint& point);
    static int pointBaseRegister(int slot_index);
    static int pointAngleRegister(int slot_index);
};

}  // namespace driver
}  // namespace tie_robot_hw
