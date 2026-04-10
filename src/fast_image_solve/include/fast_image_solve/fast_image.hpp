/*
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2025-06-01 18:39:40
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2025-06-01 20:10:10
 * @FilePath: /icc/src/fast_image_solve/include/fast_image_solve/fast_image.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef FAST_IMAGE_H
#define FAST_IMAGE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <omp.h>
#include <algorithm>
#include <mutex>

#include <fast_image_solve/image.h>

class FastImageSolve {
public:
    FastImageSolve(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~FastImageSolve();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::ServiceServer imagesolve_service;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher result_pub_;
    image_transport::Publisher binary_pub_;

    
    
    // 算法参数
    int min_Image_area_;
    double line_angle_threshold_;
    float cross_point_min_angle_;
    float grid_merge_size_;
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool imagesolveService( fast_image_solve::image::Request &req, 
                     fast_image_solve::image::Response &res);
    
    // 核心算法方法
    cv::Mat extractImage(const cv::Mat& input);
    std::vector<cv::Vec4i> detectLines(const cv::Mat& binary);
    std::vector<cv::Point2f> findCrossPoints(
        const std::vector<cv::Vec4i>& lines, 
        const cv::Size& img_size);

    std::vector<cv::Vec4i> mergeNearbyHorizontalLines(
    const std::vector<cv::Vec4i>& lines,
    double angle_thresh_deg,   // 角度阈值（度）
    double distance_thresh     // 中点距离阈值（像素）
);
double distanceBetweenLines(const cv::Vec4i& line1, const cv::Vec4i& line2);
    
    // 可视化方法
    cv::Mat visualizeResults(
        const cv::Mat& image,
        const std::vector<cv::Vec4i>& lines,
        const std::vector<cv::Point2f>& cross_points);
    
    public:
    cv::Mat input_img;
    sensor_msgs::ImageConstPtr msg_;
    std::mutex image_mutex_;
};

#endif // FAST_IMAGE_H