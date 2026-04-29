#include <ros/ros.h>

#include "tie_robot_process/suoqu/suoqu_node_app.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "suoqu_driver_node");
    return RunSuoquNodeWithDefaultRole(argc, argv, "driver");
}
