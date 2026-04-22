#ifndef COMMON_HPP
#define COMMON_HPP

#include <mutex>
#include <thread>
#include <chrono>
#include <thread>
#include <ctype.h>
#include <stdio.h>
#include <errno.h>
#include <cstdlib>
#include <csignal> 
#include <modbus.h>  
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/io.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <sys/time.h>  
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <json.hpp>
#include <fstream>
#include <filesystem>  
#include "modbus_connect.h"
#include "sbus_decoder.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>  
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "chassis_ctrl/linear_module_upload.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_move_single.h"
#include "chassis_ctrl/linear_module_move.h"
#include "chassis_ctrl/area_choose.h" // 区域选择消息
#include "chassis_ctrl/motion.h"
#include "chassis_ctrl/MotionControl.h" // 运动控制服务  
#include "chassis_ctrl/StartGlobalWork.h"
#include "chassis_ctrl/Pathguihua.h"      // 路径规划服务
#include "chassis_ctrl/StartPseudoSlamScan.h"
#include "chassis_ctrl/SingleMove.h"      // 单轴移动服务

#include "chassis_ctrl/PointCoords.h"
#include "chassis_ctrl/ProcessImage.h"

#define COLOR_RESET   "\033[0m"      // 重置所有样式
#define COLOR_BLACK   "\033[0;30m"   // 黑色
#define COLOR_RED     "\033[0;31m"   // 红色
#define COLOR_GREEN   "\033[0;32m"   // 绿色
#define COLOR_YELLOW  "\033[0;33m"   // 黄色
#define COLOR_BLUE    "\033[0;34m"   // 蓝色
#define COLOR_MAGENTA "\033[0;35m"   // 洋红/紫色
#define COLOR_CYAN    "\033[0;36m"   // 青色
#define COLOR_WHITE   "\033[0;37m"   // 白色

using json = nlohmann::json;

void printCurrentTime() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    
    // 转换为 time_t 格式
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // 转换为本地时间
    std::tm* local_time = std::localtime(&now_time_t);
    
    // 获取当前时间的毫秒部分
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
    
    // 缓冲区存储格式化后的时间
    char time_str[100];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);
    
    // 使用 printf 输出时间，精确到ms
    printf("%s.%03d", time_str, static_cast<int>(millis.count()));
    printf(" - ");
}

void clearJsonFile(const std::string& file_path) {
    // 读取原文件
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return;
    }
    json j;
    try {
        ifs >> j;
    } catch (...) {
        std::cerr << "JSON 解析错误" << std::endl;
        ifs.close();
        return;
    }
    ifs.close();

    // 完全清空内容
    j.clear();  // 如果原文件是对象，则变成 {}；如果是数组，用 j = json::array();

    // 写回文件
    std::ofstream ofs(file_path);
    if (ofs.is_open()) {
        ofs << j.dump(4);  // 输出格式化 JSON
    //     ofs.close();
    std::cout << "JSON 已清空: " << file_path << std::endl;
    } else {
        std::cerr << "无法写入文件: " << file_path << std::endl;
    }
    return;
}

#endif
