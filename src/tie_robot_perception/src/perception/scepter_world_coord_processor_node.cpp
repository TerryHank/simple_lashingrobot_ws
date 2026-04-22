#include "ros/ros.h"

#include "tie_robot_perception/perception/scepter_world_coord_processor.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "scepter_world_coord_processor");
    const std::string camera_name = ros::param::param<std::string>("~camera_name", "Scepter");
    ScepterWorldCoordProcessor processor(camera_name);
    ros::spin();
    return 0;
}
