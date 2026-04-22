#ifndef ROBOT_INTERFACE_HUB_COMMON_HPP
#define ROBOT_INTERFACE_HUB_COMMON_HPP

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include <geometry_msgs/Pose.h>
#include <json.hpp>
#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include "tie_robot_msgs/MotionControl.h"
#include "tie_robot_msgs/Pathguihua.h"
#include "tie_robot_msgs/ProcessImage.h"
#include "tie_robot_msgs/SetExecutionMode.h"
#include "tie_robot_msgs/SingleMove.h"
#include "tie_robot_msgs/StartGlobalWork.h"
#include "tie_robot_msgs/StartPseudoSlamScan.h"
#include "tie_robot_msgs/linear_module_move.h"

#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[0;32m"

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
    std::cout << "JSON 已清空: " << file_path << std::endl;
}

#endif
