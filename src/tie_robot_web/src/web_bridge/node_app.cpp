#include "tie_robot_web/web_bridge/web_action_bridge_runtime.hpp"

namespace tie_robot_web {
namespace web_bridge {

int RunWebActionBridgeNode(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "web_action_bridge_node");
    ros::NodeHandle nh;

    ros::ServiceServer start_driver_stack_srv =
        nh.advertiseService("/web/system/start_driver_stack", startDriverStackServiceCallback);
    ros::ServiceServer restart_driver_stack_srv =
        nh.advertiseService("/web/system/restart_driver_stack", restartDriverStackServiceCallback);
    ros::ServiceServer start_algorithm_stack_srv =
        nh.advertiseService("/web/system/start_algorithm_stack", startAlgorithmStackServiceCallback);
    ros::ServiceServer restart_algorithm_stack_srv =
        nh.advertiseService("/web/system/restart_algorithm_stack", restartAlgorithmStackServiceCallback);
    ros::ServiceServer restart_ros_stack_srv =
        nh.advertiseService("/web/system/restart_ros_stack", restartRosStackServiceCallback);

    g_service_clients.chassis_set_execution_mode_client =
        nh.serviceClient<tie_robot_msgs::SetExecutionMode>("/cabin/set_execution_mode");
    g_service_clients.chassis_start_work_with_options_client =
        nh.serviceClient<tie_robot_msgs::StartGlobalWork>("/cabin/start_work_with_options");
    g_service_clients.chassis_scan_with_options_client =
        nh.serviceClient<tie_robot_msgs::StartPseudoSlamScan>("/cabin/start_pseudo_slam_scan_with_options");
    g_service_clients.direct_bind_path_test_client =
        nh.serviceClient<std_srvs::Trigger>("/cabin/run_bind_path_direct_test");

    g_start_pseudo_slam_scan_action_server = std::make_unique<StartPseudoSlamScanActionServer>(
        nh, "/web/cabin/start_pseudo_slam_scan", &executeStartPseudoSlamScanAction, false);
    g_start_global_work_action_server = std::make_unique<StartGlobalWorkActionServer>(
        nh, "/web/cabin/start_global_work", &executeStartGlobalWorkAction, false);
    g_run_bind_path_direct_test_action_server = std::make_unique<RunBindPathDirectTestActionServer>(
        nh, "/web/cabin/run_bind_path_direct_test", &executeRunBindPathDirectTestAction, false);

    g_start_pseudo_slam_scan_action_server->start();
    g_start_global_work_action_server->start();
    g_run_bind_path_direct_test_action_server->start();

    logMessage("web_action_bridge_node", "节点已启动，仅暴露前端需要的Action与Service桥接；话题由前端直接连接驱动层/算法层。");

    (void)start_driver_stack_srv;
    (void)restart_driver_stack_srv;
    (void)start_algorithm_stack_srv;
    (void)restart_algorithm_stack_srv;
    (void)restart_ros_stack_srv;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}

}  // namespace web_bridge
}  // namespace tie_robot_web
