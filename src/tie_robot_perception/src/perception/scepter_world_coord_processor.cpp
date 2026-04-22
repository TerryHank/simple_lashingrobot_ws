#include "tie_robot_perception/perception/scepter_world_coord_processor.hpp"

#include <cmath>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/image_encodings.h>

namespace {

constexpr float kInvalidDepthValue = 65535.0f;
constexpr float kRansacDistanceThresholdMeters = 0.008f;
constexpr int kRansacMaxIterations = 10000;

bool isValidDepth(float depth_mm)
{
    return depth_mm > 0.0f && depth_mm != kInvalidDepthValue;
}

}  // namespace

ScepterWorldCoordProcessor::ScepterWorldCoordProcessor(const std::string& camera_name)
    : camera_name_(camera_name),
      has_depth_info_(false),
      depth_nh_(camera_name + "/depth"),
      world_coord_nh_(camera_name + "/worldCoord"),
      depth_it_(depth_nh_),
      world_coord_it_(world_coord_nh_)
{
    depth_sub_ = depth_it_.subscribe("image_raw", 1, &ScepterWorldCoordProcessor::depthImageCallback, this);
    depth_info_sub_ = depth_nh_.subscribe("camera_info", 1, &ScepterWorldCoordProcessor::depthCameraInfoCallback, this);
    world_coord_pub_ = world_coord_it_.advertise("world_coord", 30);
    raw_world_coord_pub_ = world_coord_it_.advertise("raw_world_coord", 30);
    world_coord_camera_info_pub_ = world_coord_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 30);
    convert_service_ = root_nh_.advertiseService(
        "convert_depth_to_point_cloud",
        &ScepterWorldCoordProcessor::convertDepthToPointCloud,
        this);
}

void ScepterWorldCoordProcessor::depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    latest_depth_info_ = *msg;
    has_depth_info_ = hasValidDepthIntrinsics();
}

bool ScepterWorldCoordProcessor::hasValidDepthIntrinsics() const
{
    return latest_depth_info_.K.size() >= 6 &&
           latest_depth_info_.K[0] > 0.0 &&
           latest_depth_info_.K[4] > 0.0;
}

cv::Vec3f ScepterWorldCoordProcessor::pixelDepthToWorldCoord(
    float pixel_x,
    float pixel_y,
    float depth_mm) const
{
    const float fx = static_cast<float>(latest_depth_info_.K[0]);
    const float fy = static_cast<float>(latest_depth_info_.K[4]);
    const float cx = static_cast<float>(latest_depth_info_.K[2]);
    const float cy = static_cast<float>(latest_depth_info_.K[5]);
    const float world_x = (pixel_x - cx) * depth_mm / fx;
    const float world_y = (pixel_y - cy) * depth_mm / fy;
    return cv::Vec3f(world_x, world_y, depth_mm);
}

void ScepterWorldCoordProcessor::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!has_depth_info_)
    {
        ROS_WARN_THROTTLE(2.0, "Scepter world coord processor is waiting for depth camera_info.");
        return;
    }

    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (const cv_bridge::Exception& ex)
    {
        ROS_ERROR_STREAM("Failed to decode depth image for world coord processing: " << ex.what());
        return;
    }

    const cv::Mat& depth_image = depth_ptr->image;
    cv::Mat raw_coord_image(depth_image.rows, depth_image.cols, CV_32FC3, cv::Scalar::all(0));
    cv::Mat filtered_coord_image(depth_image.rows, depth_image.cols, CV_32FC3, cv::Scalar::all(0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    valid_cloud->points.reserve(static_cast<size_t>(depth_image.rows * depth_image.cols));

    for (int row = 0; row < depth_image.rows; ++row)
    {
        for (int col = 0; col < depth_image.cols; ++col)
        {
            const float depth_mm = static_cast<float>(depth_image.at<uint16_t>(row, col));
            if (!isValidDepth(depth_mm))
            {
                continue;
            }

            const cv::Vec3f world_coord = pixelDepthToWorldCoord(static_cast<float>(col), static_cast<float>(row), depth_mm);
            raw_coord_image.at<cv::Vec3f>(row, col) = world_coord;
            valid_cloud->points.emplace_back(
                world_coord[0] / 1000.0f,
                world_coord[1] / 1000.0f,
                world_coord[2] / 1000.0f);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> non_plane_cloud;
    if (valid_cloud->points.size() >= 3)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(kRansacMaxIterations);
        seg.setDistanceThreshold(kRansacDistanceThresholdMeters);
        seg.setInputCloud(valid_cloud);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty() && inliers->indices.size() < valid_cloud->points.size())
        {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(valid_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(non_plane_cloud);
        }
        else
        {
            non_plane_cloud = *valid_cloud;
        }
    }
    else
    {
        non_plane_cloud = *valid_cloud;
    }

    for (const auto& point : non_plane_cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || point.z <= 0.0f)
        {
            continue;
        }

        const float pixel_x = point.x * static_cast<float>(latest_depth_info_.K[0]) / point.z +
                              static_cast<float>(latest_depth_info_.K[2]);
        const float pixel_y = point.y * static_cast<float>(latest_depth_info_.K[4]) / point.z +
                              static_cast<float>(latest_depth_info_.K[5]);
        const int col = static_cast<int>(std::lround(pixel_x));
        const int row = static_cast<int>(std::lround(pixel_y));
        if (row < 0 || row >= filtered_coord_image.rows || col < 0 || col >= filtered_coord_image.cols)
        {
            continue;
        }

        filtered_coord_image.at<cv::Vec3f>(row, col) = cv::Vec3f(
            point.x * 1000.0f,
            point.y * 1000.0f,
            point.z * 1000.0f);
    }

    sensor_msgs::ImagePtr filtered_msg =
        cv_bridge::CvImage(msg->header, "32FC3", filtered_coord_image).toImageMsg();
    filtered_msg->header.frame_id = camera_name_ + "_points_frame";
    world_coord_pub_.publish(filtered_msg);

    sensor_msgs::ImagePtr raw_msg =
        cv_bridge::CvImage(msg->header, "32FC3", raw_coord_image).toImageMsg();
    raw_msg->header.frame_id = camera_name_ + "raw_points_frame";
    raw_world_coord_pub_.publish(raw_msg);

    sensor_msgs::CameraInfo camera_info = latest_depth_info_;
    camera_info.header.stamp = msg->header.stamp;
    camera_info.header.frame_id = camera_name_ + "_points_frame";
    world_coord_camera_info_pub_.publish(camera_info);
}

bool ScepterWorldCoordProcessor::convertDepthToPointCloud(
    tie_robot_perception::ConvertDepthToPointCloud::Request& req,
    tie_robot_perception::ConvertDepthToPointCloud::Response& res)
{
    if (!has_depth_info_)
    {
        ROS_ERROR("convert_depth_to_point_cloud called before depth camera_info is ready.");
        return false;
    }

    if (!isValidDepth(req.depth))
    {
        ROS_ERROR_STREAM("convert_depth_to_point_cloud received invalid depth: " << req.depth);
        return false;
    }

    const cv::Vec3f world_coord = pixelDepthToWorldCoord(req.x, req.y, req.depth);
    res.world_x = world_coord[0];
    res.world_y = world_coord[1];
    res.world_z = world_coord[2];
    return true;
}
