#include "fast_image_solve/fast_image.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_image_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    
    FastImageSolve detector(nh, private_nh);

   
    
    ros::spin();
    return 0;
}