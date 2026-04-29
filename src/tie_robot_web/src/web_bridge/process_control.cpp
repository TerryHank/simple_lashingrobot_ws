#include "tie_robot_web/web_bridge/web_action_bridge_runtime.hpp"

#include <cstdlib>
#include <functional>
#include <string>
#include <thread>

namespace tie_robot_web {
namespace web_bridge {
namespace {

constexpr const char* kRestartDriverStackScript = "/home/hyq-/simple_lashingrobot_ws/restart.sh";
constexpr const char* kStartDriverStackScript = "/home/hyq-/simple_lashingrobot_ws/start_driver_stack.sh";
constexpr const char* kStartAlgorithmStackScript = "/home/hyq-/simple_lashingrobot_ws/start_algorithm_stack.sh";
constexpr const char* kRestartAlgorithmStackScript = "/home/hyq-/simple_lashingrobot_ws/restart_algorithm_stack.sh";
constexpr const char* kRestartRosStackScript = "/home/hyq-/simple_lashingrobot_ws/restart_ros_stack.sh";

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
            ROS_INFO("[web_action_bridge] %s script finished successfully: %s", topic.c_str(), script_path.c_str());
            return;
        }
        ROS_ERROR("[web_action_bridge] %s script failed (%d): %s", topic.c_str(), exit_code, script_path.c_str());
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

}  // namespace

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
