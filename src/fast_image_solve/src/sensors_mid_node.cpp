#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* BLUE */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

class PointCloudToImageConverter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher image_pub_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::string camera_pointcloud_topic;
    std::string self_image_topic;

    float plane_threshold_;

    
    int image_width_;
    int image_height_;
    
public:
    PointCloudToImageConverter(ros::NodeHandle& nh) : nh_(nh)
    {
        // 初始化相机参数
        initCameraParams(nh_);
        
        // 订阅点云话题
        pc_sub_ = nh_.subscribe(camera_pointcloud_topic, 1, &PointCloudToImageConverter::pointCloudCallback, this);
        
        // 发布图像话题
        image_pub_ = nh_.advertise<sensor_msgs::Image>(self_image_topic, 1);
        
        ROS_INFO("PointCloudToImageConverter initialized");
    }
    
private:
    void initCameraParams(ros::NodeHandle &nh)
    {
        // 初始化相机矩阵和畸变系数矩阵
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // 从参数服务器读取相机内参
        std::vector<double> camera_matrix_vec;
        if (nh.getParam("camera_matrix", camera_matrix_vec) && camera_matrix_vec.size() == 9) {
            camera_matrix_.at<double>(0, 0) = camera_matrix_vec[0];
            camera_matrix_.at<double>(0, 1) = camera_matrix_vec[1];
            camera_matrix_.at<double>(0, 2) = camera_matrix_vec[2];
            camera_matrix_.at<double>(1, 0) = camera_matrix_vec[3];
            camera_matrix_.at<double>(1, 1) = camera_matrix_vec[4];
            camera_matrix_.at<double>(1, 2) = camera_matrix_vec[5];
            camera_matrix_.at<double>(2, 0) = camera_matrix_vec[6];
            camera_matrix_.at<double>(2, 1) = camera_matrix_vec[7];
            camera_matrix_.at<double>(2, 2) = camera_matrix_vec[8];
        }

        // 从参数服务器读取畸变系数
        std::vector<double> dist_coeffs_vec;
        if (nh.getParam("distortion_coefficients", dist_coeffs_vec) && dist_coeffs_vec.size() >= 5) {
            dist_coeffs_.at<double>(0) = dist_coeffs_vec[0]; // k1
            dist_coeffs_.at<double>(1) = dist_coeffs_vec[1]; // k2
            dist_coeffs_.at<double>(2) = dist_coeffs_vec[2]; // p1
            dist_coeffs_.at<double>(3) = dist_coeffs_vec[3]; // p2
            dist_coeffs_.at<double>(4) = dist_coeffs_vec[4]; // k3
        }
        // 从参数服务器获取话题名称和图像尺寸
        nh.param("Topics/camera_pointcloud_topic", camera_pointcloud_topic, std::string("/Scepter/depthCloudPoint/cloud_points"));
        nh.param("Topics/self_image_topic", self_image_topic, std::string("/self_defination/image"));
        nh.param("image_width", image_width_, 640);
        nh.param("image_height", image_height_, 480);
        nh.param("plane_threshold", plane_threshold_, 0.02f);
        
        ROS_INFO("Camera parameters loaded from parameter server");
        ROS_INFO("Subscribing to: %s", camera_pointcloud_topic.c_str());
        ROS_INFO("Publishing to: %s", self_image_topic.c_str());
        ROS_INFO("Image size: %dx%d", image_width_, image_height_);
    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        try {
            // 转换PointCloud2到PCL格式
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
            
            // 创建32FC3图像
            cv::Mat image = cv::Mat::zeros(image_height_, image_width_, CV_32FC3);
            image.setTo(cv::Scalar(0.0f, 0.0f, 0.0f)); // 初始化为黑色(0,0,0)
            
            // 投影点云到图像
            projectPointCloudToImage(cloud, image);
            
            // 发布图像
            publishImage(image, cloud_msg);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in pointCloudCallback: %s", e.what());
        }
    }
    
    void projectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& image)
    {
        if (cloud->empty()) {
            ROS_WARN("Received empty point cloud");
            return;
        }


        // // 创建PCL点云用于RANSAC平面检测
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // for (const auto& point : cloud->points) {
        //     // 过滤无效点
        //     if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
        //         std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        //         continue;
        //     }
            
        //     pcl_cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
        // }

        // if (pcl_cloud->empty()) {
        //     return;
        // }


    //    // 创建分割对象
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // // 使用SAC分割的正确方法
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(plane_threshold_); // 
        // seg.setInputCloud(pcl_cloud);
        // seg.segment(*inliers, *coefficients);

        // 提取非平面点
        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        // extract.setInputCloud(pcl_cloud);
        // extract.setIndices(inliers);
        // extract.setNegative(true); // true表示提取非内点
        // extract.filter(*cloud_filtered);

    //     // 将非平面点转换为OpenCV点格式
        std::vector<cv::Point3f> points_3d(cloud->size());
        
        #pragma omp parallel for
        for (size_t i = 0; i < cloud->size(); i++) {
            const auto& point = cloud->points[i];
            points_3d[i] = cv::Point3f(point.x, point.y, point.z);
        }

        
        if (points_3d.empty()) {
            return;
        }
        // cloud_filtered->clear();
        
        // 准备图像点容器
        std::vector<cv::Point2f> points_2d;
        points_2d.resize(points_3d.size());
        
        // 使用OpenCV进行投影
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // 无旋转
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // 无平移
        
        try {
            cv::projectPoints(points_3d, rvec, tvec, camera_matrix_, dist_coeffs_, points_2d);
            
            // 将投影点绘制到图像上
            for (size_t i = 0; i < points_2d.size(); i++)
            {
                int u = static_cast<int>(std::round(points_2d[i].x)); // 列
                int v = static_cast<int>(std::round(points_2d[i].y)); // 行
                
                // 检查点是否在图像范围内
                if (u >= 0 && u <= image_width_ && v >= 0 && v <= image_height_) // 修复边界检查
                {
                    xyzImage(image, points_3d[i], u, v);
                }
            }
            
        } catch (const cv::Exception& e) {
            ROS_ERROR("OpenCV exception in projectPoints: %s", e.what());
        }
    }

    void xyzImage(cv::Mat &image, cv::Point3f points_3d, int u, int v)
    {
        // 将3D坐标直接存储到32FC3图像中
        double x = points_3d.x;
        double y = points_3d.y;
        double z = points_3d.z;

        if (u==320 && v==240)
        std::cout << z << RESET << std::endl;
        
        if (z > 0) {
            // 安全地设置像素值 (32FC3格式)
            cv::Vec3f& pixel = image.at<cv::Vec3f>(v, u);
            pixel[0] = x; // B -> x m -> mm 
            pixel[1] = y; // G -> y
            pixel[2] = z; // R -> z
        }
    }
    
    void publishImage(const cv::Mat& image, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        try {
            // 转换OpenCV图像到ROS消息
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp = cloud_msg->header.stamp;
            cv_image.header.frame_id = cloud_msg->header.frame_id;
            cv_image.encoding = "32FC3";  // 改为32FC3编码
            cv_image.image = image;
            
            // 发布图像
            image_pub_.publish(cv_image.toImageMsg());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception in publishImage: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensors_mid_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("Point Cloud to Image Converter started");
    
    try {
        PointCloudToImageConverter converter(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }
    
    return 0;
}