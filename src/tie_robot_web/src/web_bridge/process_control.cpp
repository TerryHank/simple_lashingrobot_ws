#include "tie_robot_web/web_bridge/topics_transfer_runtime.hpp"

#include <csignal>
#include <cstdio>
#include <regex>
#include <string>
#include <thread>
#include <unistd.h>

namespace tie_robot_web {
namespace web_bridge {
namespace {

ros::V_string g_nodes;
int g_moduan_node_pid = -1;
int g_move_node_pid = -1;
int g_pointai_node_pid = -1;
int g_topics_transfer_node_pid = -1;

int queryNodePid(const std::string& node_name)
{
    const std::string cmd = "rosnode info " + node_name + " 2>/dev/null | grep Pid";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
        return -1;
    }

    char buffer[128];
    std::string result;
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result = buffer;
    }
    pclose(pipe);

    if (result.empty()) {
        return -1;
    }

    const std::regex pid_regex("Pid: ([0-9]+)");
    std::smatch match;
    if (!std::regex_search(result, match, pid_regex)) {
        return -1;
    }
    return std::stoi(match[1]);
}

void refreshNodePids()
{
    if (!ros::master::getNodes(g_nodes)) {
        return;
    }

    for (const std::string& node : g_nodes) {
        const int pid = queryNodePid(node);
        if (pid <= 0) {
            ROS_WARN("无法获取节点 %s 的PID (可能已挂掉)", node.c_str());
            continue;
        }

        ROS_INFO("查询后端节点进程号: %s -> PID: %d", node.c_str(), pid);
        if (node == "/suoquNode") {
            g_move_node_pid = pid;
        } else if (node == "/moduanNode") {
            g_moduan_node_pid = pid;
        } else if (node == "/pointAINode") {
            g_pointai_node_pid = pid;
        } else if (node == "/topics_transfer_node") {
            g_topics_transfer_node_pid = pid;
        }
    }
}

void killNodeIfRunning(const char* label, int pid)
{
    if (pid <= 0) {
        return;
    }
    ROS_INFO("%s %d killed", label, pid);
    kill(pid, SIGKILL);
}

}  // namespace

void startRobotService()
{
    logMessage("/web/cabin/start", "开启机器人系统启动服务");
    system("bash /home/hyq-/simple_lashingrobot_ws/start_.sh");
    sleep(3);
}

void restartRobotService()
{
    refreshNodePids();
    killNodeIfRunning("moduan_node_pid", g_moduan_node_pid);
    killNodeIfRunning("move_node_pid", g_move_node_pid);
    logMessage("/web/cabin/restart", "重启索驱和末端节点启动服务");
    system("bash /home/hyq-/simple_lashingrobot_ws/restart.sh");
    sleep(3);
}

void stopRobotService()
{
    refreshNodePids();
    killNodeIfRunning("moduan_node_pid", g_moduan_node_pid);
    killNodeIfRunning("move_node_pid", g_move_node_pid);
    sleep(1);
    logMessage("/web/cabin/shutdown", "关闭机器人系统启动服务");
}

}  // namespace web_bridge
}  // namespace tie_robot_web
