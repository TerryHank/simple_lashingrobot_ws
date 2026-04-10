/*
    本客户端节点用于调试
*/
#include <ctime>  
#include <cmath> 
#include <thread>
#include <vector>
#include <fcntl.h>
#include <iomanip> // 用于std::put_time 
#include <stdio.h>
#include <csignal> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <json.hpp>
#include <iostream>  
#include <cctype>
#include <pthread.h>
#include <ros/ros.h>
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <chassis_ctrl/motion.h>
#include "chassis_ctrl/cabin_upload.h" //用于上传索驱模块反馈
#include "chassis_ctrl/MotionControl.h"
#include <std_msgs/Float32MultiArray.h>
#include "chassis_ctrl/cabin_move_all.h"
#include "chassis_ctrl/cabin_move_single.h"
#include "chassis_ctrl/cabin_calibration.h"
#include "chassis_ctrl/linear_module_move_all.h"
#include "chassis_ctrl/linear_module_upload.h"
#include <fast_image_solve/ProcessImage.h>
#include "fast_image_solve/PointCoords.h"
#include <fast_image_solve/ProcessImage.h>
#include "chassis_ctrl/area_choose.h"
#include "std_srvs/Trigger.h" 
#include <chassis_ctrl/Pathguihua.h>

#define image_srv_ 1 // 视觉请求服务
#define single_chassis_move_srv_ 2 // 底盘单点运动请求服务
#define global_chassis_path_srv_ 3 // 底盘全局路径规划请求服务
#define global_chassis_move_srv_ 4 // 底盘全局运动控制请求服务
#define moduan_move_srv_ 5 // 末端模块运动请求服务
#define lashing_move_srv_ 6 //  lashes 运动请求服务 

using namespace std;

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::ServiceClient image_solve_client;
    ros::ServiceClient Moduan_client, lashing_client;
    ros::ServiceClient Chassis_client_1, Chassis_client_2, Chassis_client_3;
    // 底盘
    // 全局路径规划
    chassis_ctrl::Pathguihua global_chassis_path_srv;
    global_chassis_path_srv.request.marking_x = 0;
    global_chassis_path_srv.request.marking_y = 0;
    global_chassis_path_srv.request.zone_x = 2000;
    global_chassis_path_srv.request.zone_y = 2000;
    global_chassis_path_srv.request.robot_x_step = 200;
    global_chassis_path_srv.request.robot_y_step = 200;
    global_chassis_path_srv.request.height = 0;
    global_chassis_path_srv.request.speed = 10;
    std_srvs::Trigger single_move_flag;
    
    fast_image_solve::ProcessImage image_srv;
    std_srvs::Trigger trigger_srv;
    // srv.request.action = true;
    image_solve_client = nh.serviceClient<fast_image_solve::ProcessImage>("/process_image");

    Chassis_client_2 = nh.serviceClient<chassis_ctrl::Pathguihua>("/cabin/path_config");
    Chassis_client_3 = nh.serviceClient<std_srvs::Trigger>("/cabin/single_move_flag");

    lashing_client = nh.serviceClient<std_srvs::Trigger>("/sg");

    int command = std::atoi(argv[1]);
    printf("指令 %d\n", command);
    if (command == image_srv_)
    {
        image_solve_client.call(image_srv);
        if ((int)image_srv.response.count)
        {
            ROS_INFO("Self-test succeeded\n");
        } else {
            ROS_ERROR("Self-test call failed");
        }
    }else if (command == lashing_move_srv_)
    {
        lashing_client.call(trigger_srv);
        if (trigger_srv.response.success)
        {
            ROS_INFO("Self-test succeeded\n");
        } else {
            ROS_ERROR("Self-test call failed");
        }
    }
    else if (command == 7)
    {
        Chassis_client_3.call(single_move_flag);
        // if (trigger_srv.response.success)
        // {
        //     ROS_INFO("Self-test succeeded\n");
        // } else {
        //     ROS_ERROR("Self-test call failed");
        // }
    }
   

    // ros::spin();
    return 0;
}

