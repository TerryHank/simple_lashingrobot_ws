#include "fast_image_solve/fast_image.hpp"
#include "fast_image_solve/5.18auto.hpp"
#include <std_srvs/Trigger.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::ServiceClient image_solve_client;
    ros::ServiceClient Moduan_client;
    ros::ServiceClient Chassis_client;

    // 创建客户端
    fast_image_solve::ProcessImage srv;
   
    // std_srvs::Trigger srv;
    // srv.request.action = true;
    image_solve_client = nh.serviceClient<fast_image_solve::ProcessImage>("/Moduan/process_image");
    // Moduan_client = nh.serviceClient<std_srvs::Trigger>("/Moduan/sg");
    while(1){
        if (image_solve_client.call(srv))
        {
            ROS_INFO("Self-test succeeded: %d", srv.response.count);
            } else {
                ROS_ERROR("Self-test call failed");
        }
        sleep(1);
    }
    
    ros::spin();
    return 0;
}