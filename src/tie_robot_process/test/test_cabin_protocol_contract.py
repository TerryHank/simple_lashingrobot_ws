#!/usr/bin/env python3

import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
HW_DIR = WORKSPACE_ROOT / "tie_robot_hw"


class CabinProtocolContractTest(unittest.TestCase):
    def compile_and_run(self, source: str) -> subprocess.CompletedProcess:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            source_path = tmp_path / "cabin_protocol_contract.cpp"
            binary_path = tmp_path / "cabin_protocol_contract"
            source_path.write_text(source, encoding="utf-8")
            compile_cmd = [
                "g++",
                "-std=c++14",
                "-I",
                str(HW_DIR / "include"),
                str(source_path),
                str(HW_DIR / "src" / "driver" / "cabin_protocol.cpp"),
                "-o",
                str(binary_path),
            ]
            subprocess.run(compile_cmd, check=True, cwd=str(WORKSPACE_ROOT))
            return subprocess.run(
                [str(binary_path)],
                cwd=str(WORKSPACE_ROOT),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

    def test_incremental_move_frame_matches_protocol_and_decodes_status_reasons(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_protocol.hpp"

            #include <cmath>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <string>
            #include <vector>

            using tie_robot_hw::driver::CabinPoseCommand;
            using tie_robot_hw::driver::CabinProtocol;

            float readFloatLE(const std::vector<uint8_t>& bytes, std::size_t offset)
            {
                uint32_t raw = static_cast<uint32_t>(bytes[offset]) |
                    (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
                    (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
                    (static_cast<uint32_t>(bytes[offset + 3]) << 24);
                float value = 0.0f;
                std::memcpy(&value, &raw, sizeof(value));
                return value;
            }

            uint16_t checksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
            {
                uint16_t value = 0;
                for (std::size_t index = 0; index < payload_size; ++index) {
                    value = static_cast<uint16_t>(value + frame[index]);
                }
                return value;
            }

            int main()
            {
                CabinPoseCommand y_negative;
                y_negative.speed_mm_per_sec = 300.0f;
                y_negative.y_mm = -40.0f;
                const auto frame = CabinProtocol::buildIncrementalMoveFrame(y_negative);
                if (frame.size() != 16) {
                    std::cerr << "incremental frame size=" << frame.size() << "\n";
                    return 1;
                }
                if (frame[0] != 0xEB || frame[1] != 0x90 || frame[2] != 0x00 || frame[3] != 0x11) {
                    std::cerr << "incremental frame header/command mismatch\n";
                    return 2;
                }
                if (frame[4] != 0x08 || frame[5] != 0x00) {
                    std::cerr << "Y- control word mismatch: " << static_cast<int>(frame[4]) << "\n";
                    return 3;
                }
                if (std::fabs(readFloatLE(frame, 6) - 300.0f) > 0.001f) {
                    std::cerr << "speed mismatch\n";
                    return 4;
                }
                if (std::fabs(readFloatLE(frame, 10) - 40.0f) > 0.001f) {
                    std::cerr << "increment distance must be positive magnitude\n";
                    return 5;
                }
                const uint16_t expected_checksum = checksum(frame, 14);
                const uint16_t actual_checksum = static_cast<uint16_t>(frame[14]) |
                    (static_cast<uint16_t>(frame[15]) << 8);
                if (actual_checksum != expected_checksum) {
                    std::cerr << "checksum mismatch\n";
                    return 6;
                }

                const std::vector<uint8_t> inverse_not_active_response = {
                    0xEB, 0x90, 0x01, 0x00, 0x00, 0x00, 0x7C, 0x01
                };
                const auto error = CabinProtocol::decodeStatus(0x0011, inverse_not_active_response);
                if (error.code != "motion_command_rejected") {
                    std::cerr << "status bit0 should reject command\n";
                    return 7;
                }
                if (error.detail.find("status_word=0x00000001") == std::string::npos ||
                    error.detail.find("逆解未激活") == std::string::npos) {
                    std::cerr << "missing protocol reason: " << error.detail << "\n";
                    return 8;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_relative_move_frame_uses_tcp_position_relative_trigger(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_protocol.hpp"

            #include <cmath>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <vector>

            using tie_robot_hw::driver::CabinPoseCommand;
            using tie_robot_hw::driver::CabinProtocol;

            float readFloatLE(const std::vector<uint8_t>& bytes, std::size_t offset)
            {
                uint32_t raw = static_cast<uint32_t>(bytes[offset]) |
                    (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
                    (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
                    (static_cast<uint32_t>(bytes[offset + 3]) << 24);
                float value = 0.0f;
                std::memcpy(&value, &raw, sizeof(value));
                return value;
            }

            uint16_t checksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
            {
                uint16_t value = 0;
                for (std::size_t index = 0; index < payload_size; ++index) {
                    value = static_cast<uint16_t>(value + frame[index]);
                }
                return value;
            }

            int main()
            {
                CabinPoseCommand y_negative;
                y_negative.speed_mm_per_sec = 300.0f;
                y_negative.y_mm = -40.0f;
                const auto frame = CabinProtocol::buildRelativeMoveFrame(y_negative);
                if (frame.size() != 36) {
                    std::cerr << "relative frame size=" << frame.size() << "\n";
                    return 1;
                }
                if (frame[0] != 0xEB || frame[1] != 0x90 || frame[2] != 0x00 || frame[3] != 0x12) {
                    std::cerr << "relative frame header/command mismatch\n";
                    return 2;
                }
                if (frame[4] != 0x02 || frame[5] != 0x00) {
                    std::cerr << "relative control word must trigger TCP relative position\n";
                    return 3;
                }
                if (std::fabs(readFloatLE(frame, 6) - 300.0f) > 0.001f) {
                    std::cerr << "speed mismatch\n";
                    return 4;
                }
                if (std::fabs(readFloatLE(frame, 10) - 0.0f) > 0.001f ||
                    std::fabs(readFloatLE(frame, 14) - -40.0f) > 0.001f ||
                    std::fabs(readFloatLE(frame, 18) - 0.0f) > 0.001f) {
                    std::cerr << "relative xyz delta mismatch\n";
                    return 5;
                }
                if (std::fabs(readFloatLE(frame, 22) - 0.0f) > 0.001f ||
                    std::fabs(readFloatLE(frame, 26) - 0.0f) > 0.001f ||
                    std::fabs(readFloatLE(frame, 30) - 0.0f) > 0.001f) {
                    std::cerr << "relative abc deltas must be zero\n";
                    return 6;
                }
                const uint16_t expected_checksum = checksum(frame, 34);
                const uint16_t actual_checksum = static_cast<uint16_t>(frame[34]) |
                    (static_cast<uint16_t>(frame[35]) << 8);
                if (actual_checksum != expected_checksum) {
                    std::cerr << "checksum mismatch\n";
                    return 7;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_position_move_status_uses_uint32_status_bytes_from_dictionary(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_protocol.hpp"

            #include <cstdint>
            #include <iostream>
            #include <string>
            #include <vector>

            using tie_robot_hw::driver::CabinProtocol;

            uint16_t checksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
            {
                uint16_t value = 0;
                for (std::size_t index = 0; index < payload_size; ++index) {
                    value = static_cast<uint16_t>(value + frame[index]);
                }
                return value;
            }

            int main()
            {
                std::vector<uint8_t> response = {
                    0xEB, 0x90, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00
                };
                const uint16_t response_checksum = checksum(response, 6);
                response[6] = static_cast<uint8_t>(response_checksum & 0xFF);
                response[7] = static_cast<uint8_t>((response_checksum >> 8) & 0xFF);

                const auto error = CabinProtocol::decodeStatus(0x0012, response);
                if (error.code != "motion_command_rejected") {
                    std::cerr << "expected motion reject, got " << error.code << "\n";
                    return 1;
                }
                if (error.detail.find("status_word=0x00010000") == std::string::npos) {
                    std::cerr << "status word must come from bytes 2~5 as UINT32: "
                              << error.detail << "\n";
                    return 2;
                }
                if (error.detail.find("C超负限位") == std::string::npos) {
                    std::cerr << "bit16 must decode as C negative limit: "
                              << error.detail << "\n";
                    return 3;
                }
                if (error.detail.find("Y超负限位") != std::string::npos) {
                    std::cerr << "must not decode bytes 4~5 as a 16-bit status word: "
                              << error.detail << "\n";
                    return 4;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_motion_status_with_high_order_bytes_is_normalized_to_protocol_bits(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_protocol.hpp"

            #include <cstdint>
            #include <iostream>
            #include <string>
            #include <vector>

            using tie_robot_hw::driver::CabinProtocol;

            uint16_t checksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
            {
                uint16_t value = 0;
                for (std::size_t index = 0; index < payload_size; ++index) {
                    value = static_cast<uint16_t>(value + frame[index]);
                }
                return value;
            }

            int main()
            {
                std::vector<uint8_t> response = {
                    0xEB, 0x90, 0x00, 0x00, 0x82, 0xC3, 0x00, 0x00
                };
                const uint16_t response_checksum = checksum(response, 6);
                response[6] = static_cast<uint8_t>(response_checksum & 0xFF);
                response[7] = static_cast<uint8_t>((response_checksum >> 8) & 0xFF);

                const auto error = CabinProtocol::decodeStatus(0x0012, response);
                if (error.code != "motion_command_rejected") {
                    std::cerr << "expected motion reject, got " << error.code << "\n";
                    return 1;
                }
                if (error.detail.find("raw_status_le32=0xC3820000") == std::string::npos) {
                    std::cerr << "missing raw little-endian status: " << error.detail << "\n";
                    return 2;
                }
                if (error.detail.find("status_word=0x000082C3") == std::string::npos) {
                    std::cerr << "missing normalized status: " << error.detail << "\n";
                    return 3;
                }
                if (error.detail.find("status_source=be32") == std::string::npos) {
                    std::cerr << "missing normalized status source: " << error.detail << "\n";
                    return 4;
                }
                if (error.detail.find("逆解未激活") == std::string::npos ||
                    error.detail.find("电机未全部使能") == std::string::npos ||
                    error.detail.find("X超负限位") == std::string::npos ||
                    error.detail.find("Y超正限位") == std::string::npos ||
                    error.detail.find("Z超正限位") == std::string::npos ||
                    error.detail.find("C超正限位") == std::string::npos) {
                    std::cerr << "missing decoded reasons: " << error.detail << "\n";
                    return 5;
                }
                if (error.detail.find("response_frame=[EB 90 00 00 82 C3") == std::string::npos) {
                    std::cerr << "missing response frame: " << error.detail << "\n";
                    return 6;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_float_like_motion_response_is_treated_as_desynchronized_frame(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_protocol.hpp"

            #include <iostream>
            #include <string>
            #include <vector>

            using tie_robot_hw::driver::CabinProtocol;

            int main()
            {
                const std::vector<uint8_t> response = {
                    0xEB, 0x90, 0x33, 0x33, 0x21, 0x43, 0x45, 0x02
                };

                const auto error = CabinProtocol::decodeStatus(0x0012, response);
                if (error.code != "protocol_response_desynchronized") {
                    std::cerr << "expected desynchronized frame, got " << error.code
                              << ": " << error.detail << "\n";
                    return 1;
                }
                if (error.detail.find("raw_status_le32=0x43213333") == std::string::npos ||
                    error.detail.find("status_bytes_as_float=161.200") == std::string::npos ||
                    error.detail.find("response_frame=[EB 90 33 33 21 43 45 02]") == std::string::npos) {
                    std::cerr << "missing raw frame diagnostics: " << error.detail << "\n";
                    return 2;
                }
                if (error.detail.find("逆解未激活") != std::string::npos ||
                    error.detail.find("X超正限位") != std::string::npos) {
                    std::cerr << "float-like status bytes must not be decoded as motion reasons: "
                              << error.detail << "\n";
                    return 3;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
