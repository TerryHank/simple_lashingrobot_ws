#ifndef TIE_ROBOT_PROCESS_COMMON_HPP
#define TIE_ROBOT_PROCESS_COMMON_HPP

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <json.hpp>
#include <tie_robot_msgs/SetExecutionMode.h>
#include <tie_robot_msgs/StartGlobalWork.h>
#include <tie_robot_msgs/StartPseudoSlamScan.h>

#define COLOR_RESET   "\033[0m"
#define COLOR_BLACK   "\033[0;30m"
#define COLOR_RED     "\033[0;31m"
#define COLOR_GREEN   "\033[0;32m"
#define COLOR_YELLOW  "\033[0;33m"
#define COLOR_BLUE    "\033[0;34m"
#define COLOR_MAGENTA "\033[0;35m"
#define COLOR_CYAN    "\033[0;36m"
#define COLOR_WHITE   "\033[0;37m"

using json = nlohmann::json;

inline void printCurrentTime() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time_t);
    const auto duration = now.time_since_epoch();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
    char time_str[100];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);
    std::printf("%s.%03d", time_str, static_cast<int>(millis.count()));
    std::printf(" - ");
}

inline void clearJsonFile(const std::string& file_path) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return;
    }

    json payload;
    try {
        ifs >> payload;
    } catch (...) {
        std::cerr << "JSON 解析错误" << std::endl;
        return;
    }

    payload.clear();

    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
        std::cerr << "无法写入文件: " << file_path << std::endl;
        return;
    }
    ofs << payload.dump(4);
}

#endif
