#include "tie_robot_control/moduan/error_handling.hpp"

#include <csignal>
#include <cstdio>
#include <exception>

#include <ros/ros.h>

#include <tie_robot_control/common.hpp>
#include "tie_robot_control/moduan/runtime_state.hpp"

void handle_system_error(const std::string& error_msg) {
    if (error_detected) {
        return;
    }
    printCurrentTime();
    ros_log_printf("Moduan_ERROR: %s\n", error_msg.c_str());
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        error_detected = true;
        last_error_msg = error_msg;
    }
    printCurrentTime();
    ros_log_printf("Moduan_log: 暂停中。\n");
}

void signalHandler(int signum)
{
    (void)signum;
    try {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        printCurrentTime();
        ros_log_printf("Moduan_log:ctrl+c已被触发,关闭末端节点。\n");
        ros::shutdown();
        _exit(2);
    } catch (const std::exception&) {
        printCurrentTime();
        ros_log_printf("Moduan_error:线性模组操作异常。\n");
        _exit(2);
    }
}
