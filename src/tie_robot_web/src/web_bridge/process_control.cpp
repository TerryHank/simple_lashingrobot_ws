#include "tie_robot_web/web_bridge/topics_transfer_runtime.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <regex>
#include <string>
#include <thread>
#include <unistd.h>

namespace tie_robot_web {
namespace web_bridge {
namespace {

constexpr const char* kStartFullStackScript = "/home/hyq-/simple_lashingrobot_ws/start_.sh";
constexpr const char* kRestartDriverStackScript = "/home/hyq-/simple_lashingrobot_ws/restart.sh";
constexpr const char* kStartDriverStackScript = "/home/hyq-/simple_lashingrobot_ws/start_driver_stack.sh";
constexpr const char* kStartAlgorithmStackScript = "/home/hyq-/simple_lashingrobot_ws/start_algorithm_stack.sh";
constexpr const char* kRestartAlgorithmStackScript = "/home/hyq-/simple_lashingrobot_ws/restart_algorithm_stack.sh";
constexpr const char* kRestartRosStackScript = "/home/hyq-/simple_lashingrobot_ws/restart_ros_stack.sh";

ros::V_string g_nodes;
int g_moduan_node_pid = -1;
int g_move_node_pid = -1;
int g_pointai_node_pid = -1;
int g_topics_transfer_node_pid = -1;

int executeShellScript(const std::string& script_path)
{
    const std::string command = "bash \"" + script_path + "\"";
    return std::system(command.c_str());
}

void runShellScriptAsync(
    const std::string& topic,
    const std::string& start_message,
    const std::string& script_path)
{
    logMessage(topic, start_message);
    std::thread([topic, script_path]() {
        const int exit_code = executeShellScript(script_path);
        if (exit_code == 0) {
            ROS_INFO("[topics_transfer] %s script finished successfully: %s", topic.c_str(), script_path.c_str());
            return;
        }
        ROS_ERROR("[topics_transfer] %s script failed (%d): %s", topic.c_str(), exit_code, script_path.c_str());
    }).detach();
}

bool triggerBackgroundControl(
    const std::string& topic,
    const std::string& accepted_message,
    const std::function<void()>& action,
    std_srvs::Trigger::Response& res)
{
    res.success = true;
    res.message = accepted_message;
    logMessage(topic, accepted_message);
    std::thread([action]() {
        action();
    }).detach();
    return true;
}

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
    runShellScriptAsync("/web/cabin/start", "开启机器人系统启动服务", kStartFullStackScript);
}

void restartRobotService()
{
    refreshNodePids();
    killNodeIfRunning("moduan_node_pid", g_moduan_node_pid);
    killNodeIfRunning("move_node_pid", g_move_node_pid);
    runShellScriptAsync("/web/cabin/restart", "重启索驱和末端节点启动服务", kRestartDriverStackScript);
}

void startDriverStackService()
{
    runShellScriptAsync("/web/system/start_driver_stack", "收到前端请求：启动驱动层", kStartDriverStackScript);
}

void restartDriverStackService()
{
    runShellScriptAsync("/web/system/restart_driver_stack", "收到前端请求：重启驱动层", kRestartDriverStackScript);
}

void startAlgorithmStackService()
{
    runShellScriptAsync("/web/system/start_algorithm_stack", "收到前端请求：启动算法层", kStartAlgorithmStackScript);
}

void restartAlgorithmStackService()
{
    runShellScriptAsync("/web/system/restart_algorithm_stack", "收到前端请求：重启算法层", kRestartAlgorithmStackScript);
}

void restartRosStackService()
{
    runShellScriptAsync("/web/system/restart_ros_stack", "收到前端请求：重启 ROS 运行栈", kRestartRosStackScript);
}

void stopRobotService()
{
    refreshNodePids();
    killNodeIfRunning("moduan_node_pid", g_moduan_node_pid);
    killNodeIfRunning("move_node_pid", g_move_node_pid);
    sleep(1);
    logMessage("/web/cabin/shutdown", "关闭机器人系统启动服务");
}

bool startDriverStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    (void)req;
    return triggerBackgroundControl(
        "/web/system/start_driver_stack",
        "已受理：正在启动驱动层。",
        []() { startDriverStackService(); },
        res);
}

bool startAlgorithmStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    (void)req;
    return triggerBackgroundControl(
        "/web/system/start_algorithm_stack",
        "已受理：正在启动算法层。",
        []() { startAlgorithmStackService(); },
        res);
}

bool restartAlgorithmStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    (void)req;
    return triggerBackgroundControl(
        "/web/system/restart_algorithm_stack",
        "已受理：正在重启算法层。",
        []() { restartAlgorithmStackService(); },
        res);
}

bool restartDriverStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    (void)req;
    return triggerBackgroundControl(
        "/web/system/restart_driver_stack",
        "已受理：正在重启驱动层。",
        []() { restartDriverStackService(); },
        res);
}

bool restartRosStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    (void)req;
    return triggerBackgroundControl(
        "/web/system/restart_ros_stack",
        "已受理：正在重启 ROS 运行栈。",
        []() { restartRosStackService(); },
        res);
}

}  // namespace web_bridge
}  // namespace tie_robot_web
