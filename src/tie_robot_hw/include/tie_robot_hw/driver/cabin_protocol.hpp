#pragma once

#include <cstdint>
#include <vector>

#include "tie_robot_hw/driver/driver_types.hpp"

namespace tie_robot_hw {
namespace driver {

class CabinProtocol {
public:
    static std::vector<uint8_t> buildMoveToPoseFrame(const CabinPoseCommand& command);
    static std::vector<uint8_t> buildRelativeMoveFrame(const CabinPoseCommand& command);
    static std::vector<uint8_t> buildIncrementalMoveFrame(const CabinPoseCommand& command);
    static std::vector<uint8_t> buildStopFrame();
    static std::vector<uint8_t> buildHeartbeatFrame(float x_gesture_deg, float y_gesture_deg);
    static DriverError decodeStatus(uint16_t command_word, const std::vector<uint8_t>& response);

private:
    static std::vector<uint8_t> buildPositionMoveFrame(const CabinPoseCommand& command, uint16_t control_word);
    static void appendFloatLittleEndian(std::vector<uint8_t>& frame, float value);
    static uint16_t computeChecksum(const std::vector<uint8_t>& frame, std::size_t payload_size);
};

}  // namespace driver
}  // namespace tie_robot_hw
