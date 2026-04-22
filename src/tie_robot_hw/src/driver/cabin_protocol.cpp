#include "tie_robot_hw/driver/cabin_protocol.hpp"

#include <cstring>

namespace tie_robot_hw {
namespace driver {

void CabinProtocol::appendFloatLittleEndian(std::vector<uint8_t>& frame, float value)
{
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    frame.push_back(static_cast<uint8_t>(raw & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw >> 24) & 0xFF));
}

uint16_t CabinProtocol::computeChecksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
{
    uint16_t checksum = 0;
    for (std::size_t index = 0; index < payload_size && index < frame.size(); ++index) {
        checksum = static_cast<uint16_t>(checksum + frame[index]);
    }
    return checksum;
}

std::vector<uint8_t> CabinProtocol::buildMoveToPoseFrame(const CabinPoseCommand& command)
{
    std::vector<uint8_t> frame;
    frame.reserve(36);
    frame.push_back(0xEB);
    frame.push_back(0x90);
    frame.push_back(0x00);
    frame.push_back(0x12);
    frame.push_back(0x01);
    frame.push_back(0x00);

    appendFloatLittleEndian(frame, command.speed_mm_per_sec);
    appendFloatLittleEndian(frame, command.x_mm);
    appendFloatLittleEndian(frame, command.y_mm);
    appendFloatLittleEndian(frame, command.z_mm);
    appendFloatLittleEndian(frame, 0.0f);
    appendFloatLittleEndian(frame, 0.0f);
    appendFloatLittleEndian(frame, 0.0f);

    const uint16_t checksum = computeChecksum(frame, frame.size());
    frame.push_back(static_cast<uint8_t>(checksum & 0xFF));
    frame.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF));
    return frame;
}

std::vector<uint8_t> CabinProtocol::buildStopFrame()
{
    return {0xEB, 0x90, 0x00, 0x13, 0x01, 0x00, 0x8F, 0x01};
}

std::vector<uint8_t> CabinProtocol::buildHeartbeatFrame(float x_gesture_deg, float y_gesture_deg)
{
    std::vector<uint8_t> frame = {0xEB, 0x90, 0x00, 0x01};
    appendFloatLittleEndian(frame, x_gesture_deg);
    appendFloatLittleEndian(frame, y_gesture_deg);
    const uint16_t checksum = computeChecksum(frame, frame.size());
    frame.push_back(static_cast<uint8_t>(checksum & 0xFF));
    frame.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF));
    return frame;
}

DriverError CabinProtocol::decodeStatus(uint16_t command_word, const std::vector<uint8_t>& response)
{
    DriverError error;
    if (response.empty()) {
        error.code = "empty_response";
        error.message = "索驱协议回包为空";
        error.retryable = true;
        return error;
    }

    if (command_word == 0x0012 && response.size() >= 6) {
        const uint16_t status_word = static_cast<uint16_t>(response[4]) |
                                     (static_cast<uint16_t>(response[5]) << 8);
        if (status_word != 0) {
            error.code = "motion_command_rejected";
            error.message = "索驱上位机拒绝当前运动指令";
            error.detail = "status_word=" + std::to_string(status_word);
            error.retryable = true;
        }
    }
    return error;
}

}  // namespace driver
}  // namespace tie_robot_hw
