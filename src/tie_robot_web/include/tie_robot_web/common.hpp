#ifndef ROBOT_INTERFACE_HUB_COMMON_HPP
#define ROBOT_INTERFACE_HUB_COMMON_HPP

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <tie_robot_web/json.hpp>
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

inline bool ros_log_message_contains_any(const std::string& message, const char* const* tokens, size_t token_count) {
    for (size_t i = 0; i < token_count; ++i) {
        if (message.find(tokens[i]) != std::string::npos) {
            return true;
        }
    }
    return false;
}

inline void ros_log_printf(const char* format, ...) {
    if (format == nullptr) {
        return;
    }

    va_list args;
    va_start(args, format);
    va_list args_copy;
    va_copy(args_copy, args);
    const int formatted_size = std::vsnprintf(nullptr, 0, format, args_copy);
    va_end(args_copy);

    if (formatted_size < 0) {
        va_end(args);
        ROS_ERROR("日志格式化失败: %s", format);
        return;
    }

    std::vector<char> buffer(static_cast<size_t>(formatted_size) + 1U);
    std::vsnprintf(buffer.data(), buffer.size(), format, args);
    va_end(args);

    std::string message(buffer.data(), static_cast<size_t>(formatted_size));
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    if (message.empty()) {
        return;
    }

    static const char* const warn_tokens[] = {"Warn", "WARN", "warn", "警告", "超出", "跳过", "缺少"};
    static const char* const error_tokens[] = {"Error", "ERROR", "error", "错误", "异常"};
    static const char* const failure_tokens[] = {"失败", "无法"};

    const bool is_warn = ros_log_message_contains_any(message, warn_tokens, sizeof(warn_tokens) / sizeof(warn_tokens[0]));
    const bool is_error =
        ros_log_message_contains_any(message, error_tokens, sizeof(error_tokens) / sizeof(error_tokens[0]))
        || (!is_warn && ros_log_message_contains_any(message, failure_tokens, sizeof(failure_tokens) / sizeof(failure_tokens[0])));

    if (is_error) {
        ROS_ERROR("%s", message.c_str());
    } else if (is_warn) {
        ROS_WARN("%s", message.c_str());
    } else {
        ROS_INFO("%s", message.c_str());
    }
}

inline void printCurrentTime() {
    // ROS logging already prints timestamp and severity; keep legacy call sites harmless.
}

inline void clearJsonFile(const std::string& file_path) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        ROS_ERROR("无法打开文件: %s", file_path.c_str());
        return;
    }

    json payload;
    try {
        ifs >> payload;
    } catch (...) {
        ROS_ERROR("JSON 解析错误: %s", file_path.c_str());
        return;
    }

    payload.clear();

    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
        ROS_ERROR("无法写入文件: %s", file_path.c_str());
        return;
    }
    ofs << payload.dump(4);
    ROS_INFO("JSON 已清空: %s", file_path.c_str());
}

#endif
