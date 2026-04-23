#include "tie_robot_web/web_bridge/topics_transfer_runtime.hpp"

namespace tie_robot_web {
namespace web_bridge {

int RunTopicsTransferNode(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "topics_transfer_node");
    ros::NodeHandle nh;

    ros::Subscriber pointai_debug_sub =
        nh.subscribe("/web/pointAI/process_image", 5, fastImageSolveProcessImageCallback);
    ros::Subscriber cabin_move_debug_sub =
        nh.subscribe("/web/cabin/cabin_move_debug", 5, chassisModuleMoveCallback);
    ros::Subscriber plan_path_sub =
        nh.subscribe("/web/cabin/plan_path", 5, robotPlanPathCallback);
    ros::Subscriber single_bind_sub =
        nh.subscribe("/web/moduan/single_bind", 5, moduanSingleBindCallback);
    ros::Subscriber moduan_move_debug_sub =
        nh.subscribe("/web/moduan/moduan_move_debug", 5, moduanLinearModuleMoveCallback);
    ros::Subscriber start_sub =
        nh.subscribe("/web/cabin/start", 5, robotStartCallback);
    ros::Subscriber clear_path_sub =
        nh.subscribe("/web/cabin/clear_path", 5, robotClearPathCallback);
    ros::Subscriber restart_sub =
        nh.subscribe("/web/cabin/restart", 5, robotRestartCallback);
    ros::Subscriber shutdown_sub =
        nh.subscribe("/web/cabin/shutdown", 5, robotStopCallback);
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

    g_service_clients.image_solve_client =
        nh.serviceClient<tie_robot_msgs::ProcessImage>("/pointAI/process_image");
    g_service_clients.chassis_single_move_client =
        nh.serviceClient<tie_robot_msgs::SingleMove>("/cabin/single_move");
    g_service_clients.chassis_plan_path_client =
        nh.serviceClient<tie_robot_msgs::Pathguihua>("/cabin/plan_path");
    g_service_clients.chassis_set_execution_mode_client =
        nh.serviceClient<tie_robot_msgs::SetExecutionMode>("/cabin/set_execution_mode");
    g_service_clients.chassis_start_work_with_options_client =
        nh.serviceClient<tie_robot_msgs::StartGlobalWork>("/cabin/start_work_with_options");
    g_service_clients.chassis_scan_with_options_client =
        nh.serviceClient<tie_robot_msgs::StartPseudoSlamScan>("/cabin/start_pseudo_slam_scan_with_options");
    g_service_clients.lashing_client =
        nh.serviceClient<std_srvs::Trigger>("/moduan/sg");
    g_service_clients.current_area_bind_from_scan_client =
        nh.serviceClient<std_srvs::Trigger>("/cabin/bind_current_area_from_scan");
    g_service_clients.direct_bind_path_test_client =
        nh.serviceClient<std_srvs::Trigger>("/cabin/run_bind_path_direct_test");
    g_service_clients.moduan_client =
        nh.serviceClient<tie_robot_msgs::linear_module_move>("/moduan/single_move");

    g_start_pseudo_slam_scan_action_server = std::make_unique<StartPseudoSlamScanActionServer>(
        nh, "/web/cabin/start_pseudo_slam_scan", &executeStartPseudoSlamScanAction, false);
    g_start_global_work_action_server = std::make_unique<StartGlobalWorkActionServer>(
        nh, "/web/cabin/start_global_work", &executeStartGlobalWorkAction, false);
    g_run_bind_path_direct_test_action_server = std::make_unique<RunBindPathDirectTestActionServer>(
        nh, "/web/cabin/run_bind_path_direct_test", &executeRunBindPathDirectTestAction, false);

    g_start_pseudo_slam_scan_action_server->start();
    g_start_global_work_action_server->start();
    g_run_bind_path_direct_test_action_server->start();

    logMessage("topics_transfer_node", "节点已启动，长任务通过Action对外暴露，短请求通过Service桥接。");

    (void)pointai_debug_sub;
    (void)cabin_move_debug_sub;
    (void)plan_path_sub;
    (void)single_bind_sub;
    (void)moduan_move_debug_sub;
    (void)start_sub;
    (void)clear_path_sub;
    (void)restart_sub;
    (void)shutdown_sub;
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
