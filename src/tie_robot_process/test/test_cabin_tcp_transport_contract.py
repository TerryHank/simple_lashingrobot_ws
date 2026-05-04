#!/usr/bin/env python3

import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
HW_DIR = WORKSPACE_ROOT / "tie_robot_hw"


class CabinTcpTransportContractTest(unittest.TestCase):
    def compile_and_run(self, source: str) -> subprocess.CompletedProcess:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            source_path = tmp_path / "cabin_tcp_transport_contract.cpp"
            binary_path = tmp_path / "cabin_tcp_transport_contract"
            source_path.write_text(source, encoding="utf-8")
            compile_cmd = [
                "g++",
                "-std=c++14",
                "-pthread",
                "-I",
                str(HW_DIR / "include"),
                str(source_path),
                str(HW_DIR / "src" / "driver" / "cabin_protocol.cpp"),
                str(HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp"),
                str(HW_DIR / "src" / "driver" / "cabin_driver.cpp"),
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

    def test_motion_receive_skips_stale_state_prefix_and_keeps_motion_status(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

            #include <arpa/inet.h>
            #include <chrono>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <sys/socket.h>
            #include <thread>
            #include <unistd.h>
            #include <vector>

            using tie_robot_hw::driver::CabinTcpTransport;
            using tie_robot_hw::driver::DriverError;

            bool sendAll(int fd, const std::vector<uint8_t>& bytes)
            {
                std::size_t sent_total = 0;
                while (sent_total < bytes.size()) {
                    const ssize_t sent = ::send(fd, bytes.data() + sent_total, bytes.size() - sent_total, 0);
                    if (sent <= 0) {
                        return false;
                    }
                    sent_total += static_cast<std::size_t>(sent);
                }
                return true;
            }

            bool recvSomeRequest(int fd)
            {
                uint8_t buffer[64] = {0};
                const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
                return got > 0 && buffer[0] == 0xEB && buffer[1] == 0x90 && buffer[2] == 0x00 && buffer[3] == 0x12;
            }

            int main()
            {
                const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
                if (listen_fd < 0) {
                    std::cerr << "listen socket failed\n";
                    return 1;
                }
                int reuse = 1;
                ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

                sockaddr_in addr;
                std::memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                addr.sin_port = 0;
                if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
                    ::listen(listen_fd, 1) != 0) {
                    std::cerr << "bind/listen failed\n";
                    return 2;
                }
                socklen_t addr_len = sizeof(addr);
                ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len);
                const uint16_t port = ntohs(addr.sin_port);

                const std::vector<uint8_t> valid_motion_status = {
                    0xEB, 0x90, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x01
                };
                std::vector<uint8_t> state_packet(144, 0x00);
                state_packet[0] = 0xEB;
                state_packet[1] = 0x90;
                state_packet[2] = 0x33;
                state_packet[3] = 0x33;
                state_packet[4] = 0x21;
                state_packet[5] = 0x43;
                state_packet[6] = 0x45;
                state_packet[7] = 0x02;

                std::thread server([&]() {
                    const int client_fd = ::accept(listen_fd, nullptr, nullptr);
                    if (client_fd < 0) {
                        return;
                    }
                    if (!recvSomeRequest(client_fd)) {
                        ::close(client_fd);
                        return;
                    }
                    sendAll(client_fd, state_packet);
                    sendAll(client_fd, valid_motion_status);
                    ::close(client_fd);
                });

                CabinTcpTransport transport("127.0.0.1", port, 2);
                DriverError error;
                std::vector<uint8_t> response;
                const std::vector<uint8_t> request = {
                    0xEB, 0x90, 0x00, 0x12, 0x02, 0x00, 0x00, 0x00
                };
                const bool ok = transport.sendAndReceive(request, &response, &error, 8);
                server.join();
                ::close(listen_fd);

                if (!ok) {
                    std::cerr << "sendAndReceive failed: " << error.code << " " << error.detail << "\n";
                    return 3;
                }
                if (response != valid_motion_status) {
                    std::cerr << "expected valid motion status after stale state packet, got";
                    for (uint8_t byte : response) {
                        std::cerr << " " << std::hex << static_cast<int>(byte);
                    }
                    std::cerr << "\n";
                    return 4;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_motion_receive_keeps_valid_rejected_status_that_looks_like_state_prefix(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

            #include <arpa/inet.h>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <sys/socket.h>
            #include <thread>
            #include <unistd.h>
            #include <vector>

            using tie_robot_hw::driver::CabinTcpTransport;
            using tie_robot_hw::driver::DriverError;

            bool sendAll(int fd, const std::vector<uint8_t>& bytes)
            {
                std::size_t sent_total = 0;
                while (sent_total < bytes.size()) {
                    const ssize_t sent = ::send(fd, bytes.data() + sent_total, bytes.size() - sent_total, 0);
                    if (sent <= 0) {
                        return false;
                    }
                    sent_total += static_cast<std::size_t>(sent);
                }
                return true;
            }

            bool recvSomeRequest(int fd)
            {
                uint8_t buffer[64] = {0};
                const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
                return got > 0 && buffer[0] == 0xEB && buffer[1] == 0x90 && buffer[2] == 0x00 && buffer[3] == 0x12;
            }

            int main()
            {
                const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
                if (listen_fd < 0) {
                    return 1;
                }
                int reuse = 1;
                ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

                sockaddr_in addr;
                std::memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                addr.sin_port = 0;
                if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
                    ::listen(listen_fd, 1) != 0) {
                    return 2;
                }
                socklen_t addr_len = sizeof(addr);
                ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len);
                const uint16_t port = ntohs(addr.sin_port);

                const std::vector<uint8_t> rejected_motion_status = {
                    0xEB, 0x90, 0x00, 0x00, 0xF5, 0x43, 0xB3, 0x02
                };

                std::thread server([&]() {
                    const int client_fd = ::accept(listen_fd, nullptr, nullptr);
                    if (client_fd < 0) {
                        return;
                    }
                    if (!recvSomeRequest(client_fd)) {
                        ::close(client_fd);
                        return;
                    }
                    sendAll(client_fd, rejected_motion_status);
                    ::close(client_fd);
                });

                CabinTcpTransport transport("127.0.0.1", port, 2);
                DriverError error;
                std::vector<uint8_t> response;
                const std::vector<uint8_t> request = {
                    0xEB, 0x90, 0x00, 0x12, 0x02, 0x00, 0x00, 0x00
                };
                const bool ok = transport.sendAndReceive(request, &response, &error, 8);
                server.join();
                ::close(listen_fd);

                if (!ok) {
                    std::cerr << "sendAndReceive failed: " << error.code << " " << error.detail << "\n";
                    return 3;
                }
                if (response != rejected_motion_status) {
                    std::cerr << "valid rejected status was not preserved\n";
                    return 4;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_driver_protocol_reject_detail_includes_request_frame(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_driver.hpp"

            #include <arpa/inet.h>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <memory>
            #include <sys/socket.h>
            #include <thread>
            #include <unistd.h>
            #include <vector>

            using tie_robot_hw::driver::CabinDriver;
            using tie_robot_hw::driver::CabinPoseCommand;
            using tie_robot_hw::driver::CabinTcpTransport;
            using tie_robot_hw::driver::DriverError;

            bool sendAll(int fd, const std::vector<uint8_t>& bytes)
            {
                std::size_t sent_total = 0;
                while (sent_total < bytes.size()) {
                    const ssize_t sent = ::send(fd, bytes.data() + sent_total, bytes.size() - sent_total, 0);
                    if (sent <= 0) {
                        return false;
                    }
                    sent_total += static_cast<std::size_t>(sent);
                }
                return true;
            }

            bool recvMotionRequest(int fd)
            {
                uint8_t buffer[64] = {0};
                const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
                return got >= 36 &&
                    buffer[0] == 0xEB &&
                    buffer[1] == 0x90 &&
                    buffer[2] == 0x00 &&
                    buffer[3] == 0x12 &&
                    buffer[4] == 0x02 &&
                    buffer[5] == 0x00;
            }

            int main()
            {
                const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
                if (listen_fd < 0) {
                    return 1;
                }
                int reuse = 1;
                ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

                sockaddr_in addr;
                std::memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                addr.sin_port = 0;
                if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
                    ::listen(listen_fd, 1) != 0) {
                    return 2;
                }
                socklen_t addr_len = sizeof(addr);
                ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len);
                const uint16_t port = ntohs(addr.sin_port);

                const std::vector<uint8_t> rejected_motion_status = {
                    0xEB, 0x90, 0x00, 0x00, 0xF5, 0x43, 0xB3, 0x02
                };

                std::thread server([&]() {
                    const int client_fd = ::accept(listen_fd, nullptr, nullptr);
                    if (client_fd < 0) {
                        return;
                    }
                    if (!recvMotionRequest(client_fd)) {
                        ::close(client_fd);
                        return;
                    }
                    sendAll(client_fd, rejected_motion_status);
                    ::close(client_fd);
                });

                std::unique_ptr<CabinTcpTransport> transport(new CabinTcpTransport("127.0.0.1", port, 2));
                CabinDriver driver(std::move(transport));
                CabinPoseCommand command;
                command.speed_mm_per_sec = 100.0f;
                command.x_mm = -100.0f;

                DriverError error;
                const bool ok = driver.moveByOffset(command, &error);
                server.join();
                ::close(listen_fd);

                if (ok) {
                    std::cerr << "rejected motion status should fail driver command\n";
                    return 3;
                }
                if (error.code != "motion_command_rejected") {
                    std::cerr << "expected motion_command_rejected, got " << error.code
                              << ": " << error.detail << "\n";
                    return 4;
                }
                if (error.detail.find("request_frame=[EB 90 00 12 02 00") == std::string::npos) {
                    std::cerr << "missing request frame: " << error.detail << "\n";
                    return 5;
                }
                if (error.detail.find("response_frame=[EB 90 00 00 F5 43 B3 02]") == std::string::npos) {
                    std::cerr << "missing response frame: " << error.detail << "\n";
                    return 6;
                }
                if (error.detail.find("status_word=0x0000F543") == std::string::npos ||
                    error.detail.find("逆解未激活") == std::string::npos ||
                    error.detail.find("C超正限位") == std::string::npos) {
                    std::cerr << "missing decoded status reasons: " << error.detail << "\n";
                    return 7;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_motion_send_drains_stale_bytes_before_request(self):
        source = textwrap.dedent(
            r'''
            #include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

            #include <arpa/inet.h>
            #include <chrono>
            #include <cstdint>
            #include <cstring>
            #include <iostream>
            #include <sys/socket.h>
            #include <thread>
            #include <unistd.h>
            #include <vector>

            using tie_robot_hw::driver::CabinTcpTransport;
            using tie_robot_hw::driver::DriverError;

            bool sendAll(int fd, const std::vector<uint8_t>& bytes)
            {
                std::size_t sent_total = 0;
                while (sent_total < bytes.size()) {
                    const ssize_t sent = ::send(fd, bytes.data() + sent_total, bytes.size() - sent_total, 0);
                    if (sent <= 0) {
                        return false;
                    }
                    sent_total += static_cast<std::size_t>(sent);
                }
                return true;
            }

            bool recvSomeRequest(int fd)
            {
                uint8_t buffer[64] = {0};
                const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
                return got > 0 && buffer[0] == 0xEB && buffer[1] == 0x90 && buffer[2] == 0x00 && buffer[3] == 0x12;
            }

            int main()
            {
                const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
                if (listen_fd < 0) {
                    return 1;
                }
                int reuse = 1;
                ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

                sockaddr_in addr;
                std::memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                addr.sin_port = 0;
                if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
                    ::listen(listen_fd, 1) != 0) {
                    return 2;
                }
                socklen_t addr_len = sizeof(addr);
                ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addr_len);
                const uint16_t port = ntohs(addr.sin_port);

                const std::vector<uint8_t> stale_prefix = {
                    0xEB, 0x90, 0x33, 0x33, 0x21, 0x43, 0x45, 0x02
                };
                const std::vector<uint8_t> valid_motion_status = {
                    0xEB, 0x90, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x01
                };

                std::thread server([&]() {
                    const int client_fd = ::accept(listen_fd, nullptr, nullptr);
                    if (client_fd < 0) {
                        return;
                    }
                    sendAll(client_fd, stale_prefix);
                    if (!recvSomeRequest(client_fd)) {
                        ::close(client_fd);
                        return;
                    }
                    sendAll(client_fd, valid_motion_status);
                    ::close(client_fd);
                });

                CabinTcpTransport transport("127.0.0.1", port, 2);
                DriverError error;
                if (!transport.ensureConnected(&error)) {
                    std::cerr << "connect failed: " << error.detail << "\n";
                    server.join();
                    ::close(listen_fd);
                    return 3;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                std::vector<uint8_t> response;
                const std::vector<uint8_t> request = {
                    0xEB, 0x90, 0x00, 0x12, 0x02, 0x00, 0x00, 0x00
                };
                const bool ok = transport.sendAndReceive(request, &response, &error, 8);
                server.join();
                ::close(listen_fd);

                if (!ok) {
                    std::cerr << "sendAndReceive failed: " << error.code << " " << error.detail << "\n";
                    return 4;
                }
                if (response != valid_motion_status) {
                    std::cerr << "expected valid motion status after pre-send drain\n";
                    return 5;
                }
                return 0;
            }
            '''
        )
        result = self.compile_and_run(source)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
