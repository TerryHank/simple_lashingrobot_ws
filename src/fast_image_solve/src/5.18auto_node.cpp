#include "fast_image_solve/5.18auto.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_image_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ImageProcessor node(nh,private_nh);

    ros::spin();

    return 0;
}