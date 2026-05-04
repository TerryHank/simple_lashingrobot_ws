#include "tie_robot_hw/driver/cabin_protocol.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

namespace tie_robot_hw {
namespace driver {
namespace {

constexpr float kIncrementalAxisEpsilonMm = 0.001f;

struct IncrementalAxisPayload {
    uint16_t control_word = 0;
    float distance_mm = 0.0f;
};

IncrementalAxisPayload resolveIncrementalAxisPayload(const CabinPoseCommand& command)
{
    if (std::fabs(command.x_mm) > kIncrementalAxisEpsilonMm) {
        return {
            static_cast<uint16_t>(command.x_mm >= 0.0f ? 0x0001 : 0x0002),
            std::fabs(command.x_mm)
        };
    }
    if (std::fabs(command.y_mm) > kIncrementalAxisEpsilonMm) {
        return {
            static_cast<uint16_t>(command.y_mm >= 0.0f ? 0x0004 : 0x0008),
            std::fabs(command.y_mm)
        };
    }
    if (std::fabs(command.z_mm) > kIncrementalAxisEpsilonMm) {
        return {
            static_cast<uint16_t>(command.z_mm >= 0.0f ? 0x0010 : 0x0020),
            std::fabs(command.z_mm)
        };
    }
    return {};
}

uint16_t readUInt16LittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return static_cast<uint16_t>(bytes[offset]) |
           (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

uint32_t readUInt32LittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

uint32_t readUInt32BigEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return (static_cast<uint32_t>(bytes[offset]) << 24) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 8) |
           static_cast<uint32_t>(bytes[offset + 3]);
}

void appendUInt16LittleEndian(std::vector<uint8_t>& frame, uint16_t value)
{
    frame.push_back(static_cast<uint8_t>(value & 0xFF));
    frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

std::string joinReasons(const std::vector<std::string>& reasons)
{
    std::string joined;
    for (std::size_t index = 0; index < reasons.size(); ++index) {
        if (index > 0) {
            joined += "；";
        }
        joined += reasons[index];
    }
    return joined;
}

std::vector<std::string> decodeTcpMotionStatusReasons(uint16_t command_word, uint32_t status_word)
{
    std::vector<std::string> reasons;
    const auto add_if_set = [&](uint32_t bit, const std::string& reason) {
        if ((status_word & (uint32_t{1} << bit)) != 0U) {
            reasons.push_back(reason);
        }
    };

    add_if_set(0, "逆解未激活");
    add_if_set(1, "电机未全部使能");
    add_if_set(2, command_word == 0x0013 ? "设备未运动" : "设备运动中");
    add_if_set(3, "设备报警");
    if (command_word != 0x0013) {
        add_if_set(4, "速度错误");
        add_if_set(5, "X超正限位");
        add_if_set(6, "X超负限位");
        add_if_set(7, "Y超正限位");
        add_if_set(8, "Y超负限位");
        add_if_set(9, "Z超正限位");
        add_if_set(10, "Z超负限位");
        add_if_set(11, "A超正限位");
        add_if_set(12, "A超负限位");
        add_if_set(13, "B超正限位");
        add_if_set(14, "B超负限位");
        add_if_set(15, "C超正限位");
        add_if_set(16, "C超负限位");
    }
    return reasons;
}

uint32_t knownTcpMotionStatusMask(uint16_t command_word)
{
    if (command_word == 0x0013) {
        return 0x0000000F;
    }
    return 0x0001FFFF;
}

struct NormalizedStatusWord {
    uint32_t raw_le32 = 0;
    uint32_t raw_be32 = 0;
    uint32_t status_word = 0;
    const char* source = "le32";
};

std::string formatCommandWord(uint16_t command_word);

NormalizedStatusWord normalizeTcpStatusWord(uint16_t command_word, const std::vector<uint8_t>& response)
{
    const uint32_t raw_le32 = readUInt32LittleEndian(response, 2);
    const uint32_t raw_be32 = readUInt32BigEndian(response, 2);
    const uint32_t known_mask = knownTcpMotionStatusMask(command_word);

    NormalizedStatusWord normalized;
    normalized.raw_le32 = raw_le32;
    normalized.raw_be32 = raw_be32;
    normalized.status_word = raw_le32;

    if (command_word == 0x0012) {
        normalized.status_word = raw_be32;
        normalized.source = "be32";
        return normalized;
    }

    const bool le_has_unknown_bits = (raw_le32 & ~known_mask) != 0U;
    const bool be_has_known_bits = (raw_be32 & known_mask) != 0U;
    const bool be_has_unknown_bits = (raw_be32 & ~known_mask) != 0U;
    if (le_has_unknown_bits && be_has_known_bits && !be_has_unknown_bits) {
        normalized.status_word = raw_be32;
        normalized.source = "be32";
    }
    return normalized;
}

std::string formatHex32(uint32_t value)
{
    std::ostringstream stream;
    stream << "0x"
           << std::uppercase
           << std::hex
           << std::setw(8)
           << std::setfill('0')
           << value;
    return stream.str();
}

std::string formatFloat3(float value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
}

float readFloatLittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    const uint32_t raw = readUInt32LittleEndian(bytes, offset);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

std::string formatByteFrame(const std::vector<uint8_t>& bytes)
{
    std::ostringstream stream;
    stream << "[";
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        if (index > 0) {
            stream << " ";
        }
        stream << std::uppercase
               << std::hex
               << std::setw(2)
               << std::setfill('0')
               << static_cast<unsigned int>(bytes[index]);
    }
    stream << "]";
    return stream.str();
}

std::string formatProtocolFrameDetail(
    uint16_t command_word,
    const std::vector<uint8_t>& response)
{
    std::ostringstream stream;
    stream << "command=" << formatCommandWord(command_word)
           << "，response_frame=" << formatByteFrame(response);
    return stream.str();
}

std::string formatStatusDetail(
    const NormalizedStatusWord& normalized_status,
    const std::vector<std::string>& reasons,
    const std::vector<uint8_t>& response)
{
    std::ostringstream stream;
    stream << "raw_status_le32=" << formatHex32(normalized_status.raw_le32)
           << "，status_word=" << formatHex32(normalized_status.status_word)
           << "，status_source=" << normalized_status.source;
    if (!reasons.empty()) {
        stream << "，原因=" << joinReasons(reasons);
    }
    stream << "，response_frame=" << formatByteFrame(response);
    return stream.str();
}

std::string formatUnexpectedStatusDetail(
    const NormalizedStatusWord& normalized_status,
    uint32_t known_mask,
    const std::vector<uint8_t>& response)
{
    std::ostringstream stream;
    stream << "raw_status_le32=" << formatHex32(normalized_status.raw_le32)
           << "，raw_status_be32=" << formatHex32(normalized_status.raw_be32)
           << "，status_word=" << formatHex32(normalized_status.status_word)
           << "，status_source=" << normalized_status.source
           << "，unknown_bits=" << formatHex32(normalized_status.status_word & ~known_mask)
           << "，known_status_mask=" << formatHex32(known_mask)
           << "，status_bytes_as_float=" << formatFloat3(readFloatLittleEndian(response, 2))
           << "，说明=状态字包含协议未定义高位，疑似读到状态查询包或旧回包残留"
           << "，response_frame=" << formatByteFrame(response);
    return stream.str();
}

std::string formatCommandWord(uint16_t command_word)
{
    std::ostringstream stream;
    stream << "0x"
           << std::uppercase
           << std::hex
           << std::setw(4)
           << std::setfill('0')
           << command_word;
    return stream.str();
}

}  // namespace

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

std::vector<uint8_t> CabinProtocol::buildPositionMoveFrame(const CabinPoseCommand& command, uint16_t control_word)
{
    std::vector<uint8_t> frame;
    frame.reserve(36);
    frame.push_back(0xEB);
    frame.push_back(0x90);
    frame.push_back(0x00);
    frame.push_back(0x12);
    appendUInt16LittleEndian(frame, control_word);

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

std::vector<uint8_t> CabinProtocol::buildMoveToPoseFrame(const CabinPoseCommand& command)
{
    return buildPositionMoveFrame(command, 0x0001);
}

std::vector<uint8_t> CabinProtocol::buildRelativeMoveFrame(const CabinPoseCommand& command)
{
    return buildPositionMoveFrame(command, 0x0002);
}

std::vector<uint8_t> CabinProtocol::buildIncrementalMoveFrame(const CabinPoseCommand& command)
{
    const IncrementalAxisPayload payload = resolveIncrementalAxisPayload(command);
    std::vector<uint8_t> frame;
    frame.reserve(16);
    frame.push_back(0xEB);
    frame.push_back(0x90);
    frame.push_back(0x00);
    frame.push_back(0x11);
    appendUInt16LittleEndian(frame, payload.control_word);

    appendFloatLittleEndian(frame, command.speed_mm_per_sec);
    appendFloatLittleEndian(frame, payload.distance_mm);

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

    if (command_word == 0x0011 || command_word == 0x0012 || command_word == 0x0013) {
        if (response.size() < 8) {
            error.code = "protocol_response_too_short";
            error.message = "索驱协议回包长度不足";
            error.detail = "command=" + formatCommandWord(command_word) +
                           "，response_bytes=" + std::to_string(response.size());
            error.retryable = true;
            return error;
        }

        const bool header_ok =
            response[1] == 0x90 &&
            (response[0] == 0xEB || (command_word == 0x0013 && response[0] == 0xEF));
        if (!header_ok) {
            error.code = "protocol_header_mismatch";
            error.message = "索驱协议回包头不匹配";
            error.detail = formatProtocolFrameDetail(command_word, response);
            error.retryable = true;
            return error;
        }

        const uint16_t actual_checksum = readUInt16LittleEndian(response, response.size() - 2);
        const uint16_t expected_checksum = computeChecksum(response, response.size() - 2);
        if (actual_checksum != expected_checksum) {
            error.code = "protocol_checksum_mismatch";
            error.message = "索驱协议回包校验失败";
            error.detail = "command=" + formatCommandWord(command_word) +
                           "，expected_checksum=" + std::to_string(expected_checksum) +
                           "，actual_checksum=" + std::to_string(actual_checksum);
            error.retryable = true;
            return error;
        }

        const NormalizedStatusWord normalized_status = normalizeTcpStatusWord(command_word, response);
        const uint32_t known_mask = knownTcpMotionStatusMask(command_word);
        if ((normalized_status.status_word & ~known_mask) != 0U) {
            error.code = "protocol_response_desynchronized";
            error.message = "索驱运动回包疑似错帧";
            error.detail = formatUnexpectedStatusDetail(normalized_status, known_mask, response);
            error.retryable = true;
            return error;
        }

        if (normalized_status.status_word != 0) {
            error.code = "motion_command_rejected";
            error.message = command_word == 0x0013
                ? "索驱上位机拒绝停止指令"
                : "索驱上位机拒绝当前运动指令";
            error.detail = formatStatusDetail(
                normalized_status,
                decodeTcpMotionStatusReasons(command_word, normalized_status.status_word),
                response
            );
            error.retryable = true;
        }
    }
    return error;
}

}  // namespace driver
}  // namespace tie_robot_hw
