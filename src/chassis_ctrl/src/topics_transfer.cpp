/*
    本节点用于后端各部分通讯的话题转换
*/
#include "common.hpp"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <regex>
#include <unistd.h> // 用于getpid()和getppid()函数
#define image_srv_ 1 // 视觉请求服务
// 索驱单点运动请求服务
#define global_chassis_path_srv_ 3 // 索驱全局路径规划请求服务
#define global_chassis_move_srv_ 4 // 索驱全局运动控制请求服务
#define moduan_move_srv_ 5 // 末端模块运动请求服务
#define lashing_move_srv_ 6 //  lashes 运动请求服务 

ros::V_string nodes;
int moduan_node_pid = -1;
int move_node_pid = -1;
int pointAI_node_pid = -1;
int topics_transfer_node_pid = -1;
// 通过节点名获取PID的函数
int getNodePid(const std::string& node_name) {
    std::string cmd = "rosnode info " + node_name + " 2>/dev/null | grep Pid";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return -1;
    
    char buffer[128];
    std::string result = "";
    if (fgets(buffer, 128, pipe) != nullptr) {
        result = buffer;
    }
    pclose(pipe);

    // 输出通常是: "Pid: 12345"
    if (!result.empty()) {
        std::regex pid_regex("Pid: ([0-9]+)");
        std::smatch match;
        if (std::regex_search(result, match, pid_regex)) {
            return std::stoi(match[1]);
        }
    }
    return -1;
}

void getNodePid(int &move_node_pid, int &moduan_node_pid, int &pointAI_node_pid, int &topics_transfer_node_pid)
{
    // 赋值各节点进程PID
    if (ros::master::getNodes(nodes)) {
        for (const std::string& node : nodes) {
            // 2. 获取该节点的PID
            int pid = getNodePid(node);
            
            if (pid > 0) {
                ROS_INFO("查询后端节点进程号: %s -> PID: %d", node.c_str(), pid);
                if (node == "/suoquNode") { move_node_pid = pid; }
                if (node == "/moduanNode") { moduan_node_pid = pid; }
                if (node == "/pointAINode") { pointAI_node_pid = pid; }
                if (node == "/topics_transfer_node") { topics_transfer_node_pid = pid; }
            } else {
                ROS_WARN("无法获取节点 %s 的PID (可能已挂掉)", node.c_str());
            }
        }
    }
    return;
}
 // 视觉
fast_image_solve::ProcessImage image_srv;
// 索驱
chassis_ctrl::SingleMove single_chassis_move_srv;
chassis_ctrl::MotionControl global_chassis_move_srv;
chassis_ctrl::StartGlobalWork start_work_srv;
// 全局路径规划
chassis_ctrl::Pathguihua global_chassis_path_srv;
// 末端
std_srvs::Trigger trigger_srv;
chassis_ctrl::linear_module_move moduan_move_srv;

using namespace std;

// 服务客户端全局变量
typedef struct {
    ros::ServiceClient image_solve_client;
    ros::ServiceClient Moduan_client;
    ros::ServiceClient lashing_client;
    ros::ServiceClient current_area_bind_from_scan_client;
    ros::ServiceClient Chassis_client_1;
    ros::ServiceClient Chassis_client_2;
    ros::ServiceClient Chassis_client_3;
    ros::ServiceClient Chassis_start_work_with_options_client;
    ros::ServiceClient Chassis_scan_client;
    ros::ServiceClient Chassis_scan_with_options_client;
} ServiceClients;
ServiceClients g_service_clients;

// 全局日志函数
void logMessage(const string& topic, const string& message) {
    ROS_INFO_STREAM("[topics_transfer] " << topic << ": " << message);
    return;
}

void startRobotService(void)
{
    // 开启机器人系统启动服务 sudo systemctrl start start_ros.service
    logMessage("/web/cabin/start", "开启机器人系统启动服务");
    system("bash /home/hyq-/simple_lashingrobot_ws/start_.sh");
    sleep(3);
    return;
}
// 1. 启动机器人 - /web/cabin/start
void robotStartCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/start", "收到启动机器人命令，值: " + to_string(msg->data));
    std::thread startRobotService_thread(startRobotService);
    startRobotService_thread.detach();
    return;
}

void pub_planpath_service(chassis_ctrl::Pathguihua path_srv)
{
    // 发布全局路径规划请求
    if (g_service_clients.Chassis_client_2.call(global_chassis_path_srv))
    {
        ROS_INFO("Call succeeded: %s", global_chassis_path_srv.response.message.c_str());
    } else {
        ROS_ERROR("Call failed");
    }
    return;
}

// 2. 规划作业路径 - /web/cabin/plan_path
void robotPlanPathCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/plan_path", "收到规划作业路径命令");
    global_chassis_path_srv.request.marking_x = msg->position.x;
    global_chassis_path_srv.request.marking_y = msg->position.y;
    global_chassis_path_srv.request.height = msg->position.z;
    global_chassis_path_srv.request.zone_x = msg->orientation.x;
    global_chassis_path_srv.request.zone_y = msg->orientation.y;
    global_chassis_path_srv.request.robot_x_step = msg->orientation.z;
    global_chassis_path_srv.request.robot_y_step = msg->orientation.w;
    global_chassis_path_srv.request.speed = 0.0; // 索驱速度分离下发
    
    std::cout << COLOR_GREEN << "作业参数(mm):["
        << global_chassis_path_srv.request.marking_x << ","
        << global_chassis_path_srv.request.marking_y << ","
         << global_chassis_path_srv.request.height << ","
        << global_chassis_path_srv.request.zone_x << ","
        << global_chassis_path_srv.request.zone_y << ","
        << global_chassis_path_srv.request.robot_x_step << ","
        << global_chassis_path_srv.request.robot_y_step << "," 
        << global_chassis_path_srv.request.speed << "]" << COLOR_RESET << std::endl; 
    std::thread pub_planpath_service_thread(pub_planpath_service, global_chassis_path_srv);
    pub_planpath_service_thread.detach();
    return;
}

// 3. 清除作业路径 - /web/cabin/clear_path
void robotClearPathCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/clear_path", "收到清除作业路径命令，值: " + to_string(msg->data));
    // 清除一个json文件
    std::string file_path = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json";
    clearJsonFile(file_path);
    return;
}

void pub_startglobalwork_service(chassis_ctrl::StartGlobalWork start_work_srv)
{
    // 发布全局运动控制请求
    g_service_clients.Chassis_start_work_with_options_client.call(start_work_srv);
    return;

}
// 4. 开始全局作业 - /web/cabin/start_global_work
void startGlobalWorkCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    const bool clear_execution_memory = msg->data >= 2.0f;
    logMessage(
        "/web/cabin/start_global_work",
        "收到开始全局作业命令，模式="
        + string(clear_execution_memory ? "清空记忆后开始" : "保留记忆直接开始")
        + "，值: " + to_string(msg->data)
    );
    start_work_srv.request.command = "全局运动请求";
    start_work_srv.request.clear_execution_memory = clear_execution_memory;
    
    std::thread pub_startglobalwork_service_thread(pub_startglobalwork_service, start_work_srv);
    pub_startglobalwork_service_thread.detach();
    return;
}

void setGlobalExecutionModeCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    if (msg->data >= 1.0f) {
        logMessage(
            "/web/cabin/set_execution_mode",
            "收到全局执行模式切换命令，执行模式=live_visual：按路径边执行边识别"
        );
    } else {
        logMessage(
            "/web/cabin/set_execution_mode",
            "收到全局执行模式切换命令，执行模式=slam_precomputed：使用扫描后的预生成路径执行"
        );
    }
}

void pseudoSlamScanCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    const bool enable_capture_gate = msg->data >= 2.0f;
    logMessage(
        "/web/cabin/start_pseudo_slam_scan",
        "收到扫描建图命令，模式="
        + string(enable_capture_gate ? "开启最终采集门" : "关闭最终采集门")
        + "，值: " + to_string(msg->data)
    );

    chassis_ctrl::StartPseudoSlamScan scan_srv;
    scan_srv.request.enable_capture_gate = enable_capture_gate;
    if (!g_service_clients.Chassis_scan_with_options_client.call(scan_srv)) {
        ROS_ERROR("扫描建图服务调用失败");
        return;
    }
    if (!scan_srv.response.success) {
        ROS_WARN("扫描建图服务返回失败: %s", scan_srv.response.message.c_str());
        return;
    }
    ROS_INFO("扫描建图服务调用成功: %s", scan_srv.response.message.c_str());
}

void restartRobotService(void)
{
    getNodePid(move_node_pid, moduan_node_pid, pointAI_node_pid, topics_transfer_node_pid);
    if (moduan_node_pid > 0)
    {
        ROS_INFO("moduan_node_pid %d killed", moduan_node_pid);
        kill(moduan_node_pid, SIGKILL);
    }
    if (move_node_pid > 0)
    {
        ROS_INFO("move_node_pid %d killed", move_node_pid);
        kill(move_node_pid, SIGKILL);
    } 
    // if (pointAI_node_pid > 0)
    // {
    //     ROS_INFO("pointAI_node_pid %d killed", pointAI_node_pid);
    //     kill(pointAI_node_pid, SIGKILL);
    // }
    // 重启机器人系统启动服务 sudo systemctrl restart SuoquAndModuan.service
    logMessage("/web/cabin/restart", "重启索驱和末端节点启动服务");
    system("bash /home/hyq-/simple_lashingrobot_ws/restart.sh");
    sleep(3);
    return;
}

// 5. 重启机器人 - /web/cabin/restart
void robotRestartCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/restart", "收到重启机器人命令，值: " + to_string(msg->data));
    std::thread restartRobotService_thread(restartRobotService);
    restartRobotService_thread.detach();
    return;
}

void pub_moduanmovedebug_service(chassis_ctrl::linear_module_move moduan_move_srv)
{
    g_service_clients.Moduan_client.call(moduan_move_srv);
    return;
}
// 6. 末端运动调试 - /web/moduan/linear_module_move
void moduanLinearModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/linear_module_move", "收到末端运动调试命令");
    moduan_move_srv.request.pos_x = msg->position.x;
    moduan_move_srv.request.pos_y = msg->position.y;
    moduan_move_srv.request.pos_z = msg->position.z;
    moduan_move_srv.request.angle = msg->orientation.x;
    std::cout << COLOR_GREEN << "末端调试:[" 
        << moduan_move_srv.request.pos_x 
        << "," << moduan_move_srv.request.pos_y 
        << "," << moduan_move_srv.request.pos_z
        << "," << moduan_move_srv.request.angle << "]" << COLOR_RESET << std::endl;
    
    std::thread pub_moduanmovedebug_service_thread(pub_moduanmovedebug_service,moduan_move_srv);
    pub_moduanmovedebug_service_thread.detach();

    return;
}


void pub_chassismovedebug_service(chassis_ctrl::SingleMove single_chassis_move_srv)
{
     if (g_service_clients.Chassis_client_1.call(single_chassis_move_srv))
    {
        ROS_INFO("Call succeeded: %s", single_chassis_move_srv.response.message.c_str());
    } else {
        ROS_ERROR("Call failed：%s", single_chassis_move_srv.response.message.c_str());
    }
    return;
}
// 7. 索驱运动调试 - /web/cabin/single_move
void chassisModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/single_move", "收到索驱运动调试命令");
    single_chassis_move_srv.request.command = "单点运动请求";
    single_chassis_move_srv.request.x = float(msg->position.x);
    single_chassis_move_srv.request.y = float(msg->position.y);
    single_chassis_move_srv.request.z = float(msg->position.z);
    single_chassis_move_srv.request.speed = float(msg->orientation.x);
    std::thread pub_chassismovedebug_service_thread(pub_chassismovedebug_service,single_chassis_move_srv);
    pub_chassismovedebug_service_thread.detach();
    return;
}

void pub_pointAIdebug_service(fast_image_solve::ProcessImage image_srv)
{
     if (g_service_clients.image_solve_client.call(image_srv))
    {
        ROS_INFO("Call succeeded: %d", image_srv.response.count);
    } else {
        ROS_ERROR("Call failed");
    }
    return;
}

// 8. 视觉识别调试 - /web/fast_image_solve/process_image
void fastImageSolveProcessImageCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/fast_image_solve/process_image", "收到视觉识别调试命令，值: " + to_string(msg->data));
    std::thread pub_pointAIdebug_service_thread(pub_pointAIdebug_service,image_srv);
    pub_pointAIdebug_service_thread.detach();
    return;
}

void pub_singlebind_service(bool use_precomputed_current_area)
{
    std_srvs::Trigger single_bind_srv;
    ros::ServiceClient& target_client = use_precomputed_current_area
        ? g_service_clients.current_area_bind_from_scan_client
        : g_service_clients.lashing_client;

    if (target_client.call(single_bind_srv))
    {
        ROS_INFO("Call succeeded: %s", single_bind_srv.response.message.c_str());
    } else {
        ROS_ERROR("Call failed");
    }
    return;

}
// 9. 定点绑扎调试 - /web/moduan/single_bind
void moduanSingleBindCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    const bool use_precomputed_current_area = msg->data >= 1.0f;
    if (use_precomputed_current_area) {
        logMessage(
            "/web/moduan/single_bind",
            "收到定点绑扎调试命令，single_bind模式=1：使用扫描后的当前区域预计算点直执行"
        );
    } else {
        logMessage(
            "/web/moduan/single_bind",
            "收到定点绑扎调试命令，single_bind模式=0：使用原逻辑本区域识别+执行"
        );
    }
    std::thread pub_singlebind_service_thread(pub_singlebind_service, use_precomputed_current_area);
    pub_singlebind_service_thread.detach();
    return;
}

// 10. 暂停作业 - /web/moduan/interrupt_stop
void moduanInterruptStopCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/interrupt_stop", "收到暂停作业命令，值: " + to_string(msg->data));
    return;
}

// 11. 开启绑扎开关 - /web/moduan/enb_las
void moduanEnbLasCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/enb_las", "收到开启绑扎开关命令，值: " + to_string(msg->data));
    return;
}

// 12. 开启(关闭)跳绑 - /web/moduan/send_odd_points
void moduanSendOddPointsCallback(const std_msgs::Bool::ConstPtr& msg) {
    printCurrentTime();
    std::string valueStr = msg->data ? "true" : "false";
    logMessage("/web/moduan/send_odd_points", "收到开启(关闭)跳绑2/4命令，值: " + valueStr);
    return;
}

// 13. 恢复作业 - /web/hand_sovle_warn
void handSovleWarnCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/hand_sovle_warn", "收到恢复作业命令，值: " + to_string(msg->data));    
    return;
    
}

// 14. 开启(关闭)灯光 - /web/moduan/light
void moduanLightCallback(const std_msgs::Bool::ConstPtr& msg) {
    printCurrentTime();
    std::string valueStr = msg->data ? "true" : "false";
    logMessage("/web/moduan/light", "收到开启(关闭)灯光命令，值: " + valueStr);
    return;
}

// 15. 末端回零 - /web/moduan/linear_module_gb_origin
void moduanLinearModuleGbOriginCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/moduan_move_zero", "收到末端回零命令，值: " + to_string(msg->data)); 
    // moduan_move_srv.request.pos_x = 0.0;
    // moduan_move_srv.request.pos_y = 0.0;
    // moduan_move_srv.request.pos_z = 0.0;
    // moduan_move_srv.request.angle = 0.0;
    // std::cout << COLOR_GREEN << "末端回零(mm):[" 
    //     << moduan_move_srv.request.pos_x 
    //     << "," << moduan_move_srv.request.pos_y 
    //     << "," << moduan_move_srv.request.pos_z
    //     << "," << moduan_move_srv.request.angle << "]" << COLOR_RESET << std::endl;
    // // 发布末端模块运动请求
    // g_service_clients.Moduan_client.call(moduan_move_srv);
    return;
}

// 16. 关闭绑扎开关 - /web/moduan/disable_las
void moduanDisableLasCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/disable_las", "收到关闭绑扎开关命令，值: " + to_string(msg->data));
    return;
}

// 17. 关闭机器人 - /web/cabin/shutdown
void robotStopCallback(const std_msgs::Float32::ConstPtr& msg) {    
    printCurrentTime();
    logMessage("/web/cabin/shutdown", "收到关闭机器人命令，值: " + to_string(msg->data));
    getNodePid(move_node_pid, moduan_node_pid, pointAI_node_pid, topics_transfer_node_pid);
    if (moduan_node_pid > 0)
    {
        ROS_INFO("moduan_node_pid %d killed", moduan_node_pid);
        kill(moduan_node_pid, SIGKILL);
    }
    if (move_node_pid > 0)
    {
        ROS_INFO("move_node_pid %d killed", move_node_pid);
        kill(move_node_pid, SIGKILL);
    }   
    // if (pointAI_node_pid > 0)
    // {
    //     ROS_INFO("pointAI_node_pid %d killed", pointAI_node_pid);
    //     kill(pointAI_node_pid, SIGKILL);
    // }
    sleep(1);
    logMessage("/web/cabin/shutdown", "关闭机器人系统启动服务");
    // system("sudo systemctl stop start_ros.service");
    // sleep(3);
    // system("sudo systemctl stop SuoquAndModuan.service");
    // sleep(2);
    return;
}

// 18. 急停作业 - /Moduan/forced_stop
void moduanForcedStopCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/forced_stop", "收到急停作业命令，值: " + to_string(msg->data));
    return;
}

// 19. 设置高度阈值 - /web/fast_image_solve/set_height_threshold
void fastImageSolveSetHeightThresholdCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/fast_image_solve/set_height_threshold", "收到设置高度阈值命令，值: " + to_string(msg->data) + "mm");
    return;
}

// 20. 保存作业路径 - /web/cabin/save_path
void robotSavePathCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/save_path", "收到保存作业路径命令，值: " + to_string(msg->data));
    return;
}

// 21. 保存绑扎数据 - /web/cabin/save_binding_data
void robotSaveBindingDataCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/save_binding_data", "收到保存绑扎数据命令，值: " + to_string(msg->data));
    return;
}

// 22. 设置TF平移标定(兼容旧话题名) - /web/fast_image_solve/set_pointAI_offset
void fastImageSolveSetPointAIOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    printCurrentTime();
    logMessage(
        "/web/fast_image_solve/set_pointAI_offset",
        "收到TF平移标定命令，写入gripper_tf.yaml.translation_mm，值: "
        + to_string(msg->position.x) + "mm , "
        + to_string(msg->position.y) + "mm , "
        + to_string(msg->position.z) + "mm"
    );
    return;
}

// 23. 设置索驱速度 - /web/cabin/set_cabin_speed
void robotSetCabinSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/cabin/set_cabin_speed", "收到设置索驱速度命令，值: " + to_string(msg->data));
    return;
}

// 24. 设置末端速度 - /web/moduan/set_moduan_speed
void moduanSetModuanSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
    printCurrentTime();
    logMessage("/web/moduan/set_moduan_speed", "收到设置末端速度命令，值: " + to_string(msg->data));    
    return;
}



int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "topics_transfer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 初始化服务客户端（需要转换的服务）
    // 视觉调试
    ros::Subscriber sub8 = nh.subscribe("/web/fast_image_solve/process_image", 5, fastImageSolveProcessImageCallback);
    g_service_clients.image_solve_client = nh.serviceClient<fast_image_solve::ProcessImage>("/pointAI/process_image");
    // 索驱单点运动
    ros::Subscriber sub7 = nh.subscribe("/web/cabin/cabin_move_debug", 5, chassisModuleMoveCallback);
    g_service_clients.Chassis_client_1 = nh.serviceClient<chassis_ctrl::SingleMove>("/cabin/single_move");
    // 路径规划
    ros::Subscriber sub2 = nh.subscribe("/web/cabin/plan_path", 5, robotPlanPathCallback);
    g_service_clients.Chassis_client_2 = nh.serviceClient<chassis_ctrl::Pathguihua>("/cabin/plan_path");
    // 索驱全局运动
    ros::Subscriber sub4 = nh.subscribe("/web/cabin/start_global_work", 5, startGlobalWorkCallback);
    ros::Subscriber sub26 = nh.subscribe("/web/cabin/set_execution_mode", 5, setGlobalExecutionModeCallback);
    g_service_clients.Chassis_client_3 = nh.serviceClient<chassis_ctrl::MotionControl>("/cabin/start_work");
    g_service_clients.Chassis_start_work_with_options_client =
        nh.serviceClient<chassis_ctrl::StartGlobalWork>("/cabin/start_work_with_options");
    ros::Subscriber sub25 = nh.subscribe("/web/cabin/start_pseudo_slam_scan", 5, pseudoSlamScanCallback);
    g_service_clients.Chassis_scan_client = nh.serviceClient<std_srvs::Trigger>("/cabin/start_pseudo_slam_scan");
    g_service_clients.Chassis_scan_with_options_client =
        nh.serviceClient<chassis_ctrl::StartPseudoSlamScan>("/cabin/start_pseudo_slam_scan_with_options");
    // 末端单点调试（含绑扎）
    ros::Subscriber sub9 = nh.subscribe("/web/moduan/single_bind", 5, moduanSingleBindCallback);
    g_service_clients.lashing_client = nh.serviceClient<std_srvs::Trigger>("/moduan/sg");
    g_service_clients.current_area_bind_from_scan_client = nh.serviceClient<std_srvs::Trigger>("/cabin/bind_current_area_from_scan");
    // 末端单点运动
    ros::Subscriber sub6 = nh.subscribe("/web/moduan/moduan_move_debug", 5, moduanLinearModuleMoveCallback);
    g_service_clients.Moduan_client = nh.serviceClient<chassis_ctrl::linear_module_move>("/moduan/single_move");
    
    // 由前端直接控制的话题
    ros::Subscriber sub1 = nh.subscribe("/web/cabin/start", 5, robotStartCallback);
    ros::Subscriber sub3 = nh.subscribe("/web/cabin/clear_path", 5, robotClearPathCallback);
    ros::Subscriber sub5 = nh.subscribe("/web/cabin/restart", 5, robotRestartCallback);
    // ros::Subscriber sub10 = nh.subscribe("/web/moduan/interrupt_stop", 5, moduanInterruptStopCallback);
    // ros::Subscriber sub11 = nh.subscribe("/web/moduan/enb_las", 5, moduanEnbLasCallback);
    // ros::Subscriber sub12 = nh.subscribe("/web/moduan/send_odd_points", 5, moduanSendOddPointsCallback);
    // ros::Subscriber sub13 = nh.subscribe("/web/moduan/hand_sovle_warn", 5, handSovleWarnCallback);
    // ros::Subscriber sub14 = nh.subscribe("/web/moduan/light", 5, moduanLightCallback);
    // ros::Subscriber sub15 = nh.subscribe("/web/moduan/moduan_move_zero", 5, moduanLinearModuleGbOriginCallback);
    // ros::Subscriber sub16 = nh.subscribe("/web/moduan/disable_las", 5, moduanDisableLasCallback);
    ros::Subscriber sub17 = nh.subscribe("/web/cabin/shutdown", 5, robotStopCallback);
    // ros::Subscriber sub18 = nh.subscribe("/web/moduan/forced_stop", 5, moduanForcedStopCallback);
    // ros::Subscriber sub19 = nh.subscribe("/web/fast_image_solve/set_height_threshold", 5, fastImageSolveSetHeightThresholdCallback);
    // ros::Subscriber sub20 = nh.subscribe("/web/cabin/save_path", 5, robotSavePathCallback);
    // ros::Subscriber sub21 = nh.subscribe("/web/cabin/save_binding_data", 5, robotSaveBindingDataCallback);
    // ros::Subscriber sub22 = nh.subscribe("/web/fast_image_solve/set_pointAI_offset", 5, fastImageSolveSetPointAIOffsetCallback);
    // ros::Subscriber sub23 = nh.subscribe("/web/cabin/set_cabin_speed", 5, robotSetCabinSpeedCallback);
    // ros::Subscriber sub24 = nh.subscribe("/web/moduan/set_moduan_speed", 5, moduanSetModuanSpeedCallback);
    
    logMessage("topics_transfer_node", "节点已启动，正在监听25个话题...");
    
    // 启动ROS循环
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}
