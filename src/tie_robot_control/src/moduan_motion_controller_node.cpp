#include <ros/ros.h>

#include "tie_robot_control/moduan/moduan_ros_callbacks.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moduan_motion_controller_node");
    return RunModuanNodeWithDefaultRole(argc, argv, "motion_controller");
}
