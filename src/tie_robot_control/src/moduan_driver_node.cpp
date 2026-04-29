#include <ros/ros.h>

#include "tie_robot_control/moduan/moduan_ros_callbacks.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moduan_driver_node");
    return RunModuanNodeWithDefaultRole(argc, argv, "driver");
}
