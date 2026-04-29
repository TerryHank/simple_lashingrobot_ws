#ifndef TIE_ROBOT_PROCESS_COMMON_HPP
#define TIE_ROBOT_PROCESS_COMMON_HPP

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tie_robot_process/json.hpp>
#include <ros/ros.h>
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

inline bool ros_log_message_contains_any(const std::string& message, const char* const* tokens, size_t token_count) {
    for (size_t i = 0; i < token_count; ++i) {
        if (message.find(tokens[i]) != std::string::npos) {
            return true;
        }
    }
    return false;
}

inline std::string ros_log_to_lower(std::string value) {
    for (char& character : value) {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return value;
}

inline bool ros_log_extract_legacy_prefix(std::string& message, std::string* level_tag) {
    static const char* const source_prefixes[] = {"Cabin", "Moduan", "PointAI", "pointAI", "web_action_bridge"};
    for (size_t i = 0; i < sizeof(source_prefixes) / sizeof(source_prefixes[0]); ++i) {
        const char* const source_prefix = source_prefixes[i];
        const size_t source_prefix_len = std::strlen(source_prefix);
        if (message.compare(0, source_prefix_len, source_prefix) != 0) {
            continue;
        }

        size_t cursor = source_prefix_len;
        std::string tag = "log";
        if (cursor < message.size() && message[cursor] == '_') {
            const size_t tag_start = cursor + 1U;
            const size_t colon_pos = message.find(':', tag_start);
            if (colon_pos == std::string::npos) {
                continue;
            }
            tag = ros_log_to_lower(message.substr(tag_start, colon_pos - tag_start));
            if (tag != "log" && tag != "info" && tag != "warn" && tag != "error") {
                continue;
            }
            cursor = colon_pos + 1U;
        } else if (cursor < message.size() && message[cursor] == ':') {
            cursor += 1U;
        } else {
            continue;
        }

        message.erase(0, cursor);
        const size_t first_content = message.find_first_not_of(" \t");
        if (first_content == std::string::npos) {
            message.clear();
        } else if (first_content > 0U) {
            message.erase(0, first_content);
        }
        if (level_tag != nullptr) {
            *level_tag = tag;
        }
        return true;
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

    std::string legacy_level_tag;
    const bool has_legacy_level = ros_log_extract_legacy_prefix(message, &legacy_level_tag);
    if (message.empty()) {
        return;
    }

    static const char* const warn_tokens[] = {"Warn", "WARN", "warn", "警告", "超出", "跳过", "缺少"};
    static const char* const error_tokens[] = {"Error", "ERROR", "error", "错误", "异常"};
    static const char* const failure_tokens[] = {"失败"};

    const bool is_warn = has_legacy_level
        ? (legacy_level_tag == "warn")
        : ros_log_message_contains_any(message, warn_tokens, sizeof(warn_tokens) / sizeof(warn_tokens[0]));
    const bool is_error = has_legacy_level
        ? (legacy_level_tag == "error")
        : (
            ros_log_message_contains_any(message, error_tokens, sizeof(error_tokens) / sizeof(error_tokens[0]))
            || (!is_warn && ros_log_message_contains_any(message, failure_tokens, sizeof(failure_tokens) / sizeof(failure_tokens[0])))
        );

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
}

#endif
