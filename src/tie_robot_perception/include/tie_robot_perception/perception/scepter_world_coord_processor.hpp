#ifndef TIE_ROBOT_PERCEPTION_SCEPTER_WORLD_COORD_PROCESSOR_HPP
#define TIE_ROBOT_PERCEPTION_SCEPTER_WORLD_COORD_PROCESSOR_HPP

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "tie_robot_perception/ConvertDepthToPointCloud.h"

class ScepterWorldCoordProcessor {
public:
    explicit ScepterWorldCoordProcessor(const std::string& camera_name = "Scepter");

private:
    void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool convertDepthToPointCloud(
        tie_robot_perception::ConvertDepthToPointCloud::Request& req,
        tie_robot_perception::ConvertDepthToPointCloud::Response& res);
    bool hasValidDepthIntrinsics() const;
    cv::Vec3f pixelDepthToWorldCoord(float pixel_x, float pixel_y, float depth_mm) const;

    std::string camera_name_;
    bool has_depth_info_;
    sensor_msgs::CameraInfo latest_depth_info_;

    ros::NodeHandle root_nh_;
    ros::NodeHandle depth_nh_;
    ros::NodeHandle world_coord_nh_;
    image_transport::ImageTransport depth_it_;
    image_transport::ImageTransport world_coord_it_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber depth_info_sub_;
    image_transport::Publisher world_coord_pub_;
    image_transport::Publisher raw_world_coord_pub_;
    ros::Publisher world_coord_camera_info_pub_;
    ros::ServiceServer convert_service_;
};

#endif
