/*
    视觉合同部分代码，c++形式（待测试）
*/
#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <fast_image_solve/PointsArray.h>
#include <fast_image_solve/PointCoords.h>
// #include <chassis_ctrl/motion.h>
// #include <chassis_ctrl/linear_module_move.h>
#include <fast_image_solve/ProcessImage.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>  // 包含cv::partition
#include <opencv2/core.hpp>          // 基础矩阵操作
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <omp.h>

#define COLOR_RESET   "\033[0m"      // 重置所有样式
#define COLOR_BLACK   "\033[0;30m"   // 黑色
#define COLOR_RED     "\033[0;31m"   // 红色
#define COLOR_GREEN   "\033[0;32m"   // 绿色
#define COLOR_YELLOW  "\033[0;33m"   // 黄色
#define COLOR_BLUE    "\033[0;34m"   // 蓝色
#define COLOR_MAGENTA "\033[0;35m"   // 洋红/紫色
#define COLOR_CYAN    "\033[0;36m"   // 青色
#define COLOR_WHITE   "\033[0;37m"   // 白色

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~ImageProcessor() = default;

private:
    // ROS 相关成员
    ros::NodeHandle nh_, params_nh;
    ros::Subscriber image_sub_;
    ros::Subscriber color_image_sub_;
    ros::Subscriber infrared_image_sub_;
    ros::Subscriber max_depth_offset_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher image_pub_;
    ros::Publisher show_image_pub_;
    ros::Publisher coordinate_pub_;
    ros::ServiceServer process_image_srv_;
    ros::ServiceServer detect_and_save_pose_srv_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 图像处理相关成员
    cv_bridge::CvImagePtr bridge_;
    cv::Mat image_;
    cv::Mat x_channel, y_channel;
    cv::Mat vison_image_;
    cv::Mat Depth_image_Raw_ ,Depth_image_Range_, Depth_image_Raw_uni_ , Depth_image_Raw_binary_, closing_;
    cv::Mat skeleton_, line_image_, line_image_x;
    cv::Mat image_color_;
    cv::Mat image_color_copy_;
    cv::Mat image_infrared_;
    cv::Mat image_infrared_copy_;
    
    // 节点参数由参数服务器导入
    int threshold_;
    int minLineLength_;
    int maxLineGap_;
    int min_depth_, max_depth_;
    int max_depth_offset_;
    int offset_x_, offset_y_, offset_z_;
    int half_size_;
    int non_shuiguan_count_, shuiguan_count_;
    int displacement_x_, displacement_y_, displacement_z_;
    int region_x_, region_y_, region_z_;
    int x1_, y1_, x2_, y2_, x3_,y3_, x4_, y4_; 
    int frame_count_; 
    float fps_;
    float ground_h_;
    int pixel_clsuter_;
    int medianBlur_, thinning_size_, iterations_;
    bool use_nineregion_;
    std::string pose_matrix_path;
    std::string nineregion_data_path;
    std::string fullregion_data_path;
    std::string self_defination_image_topic;
    std::string color_image_topic;
    std::string ir_image_topic;
    std::string camera_info_topic;
    

    Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity(); // 相机虎口和auroc玛外参
    Eigen::Matrix4f T_camera_ee = Eigen::Matrix4f::Identity();
    Eigen::Affine3f T_scepter_gripper = Eigen::Affine3f::Identity();

    ros::Time start_time_;
    std::vector<std::pair<std::pair<cv::Point2f, cv::Point2f>, float>> angle_view;
    
public:
    std::mutex _channel_image;
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    inline bool fileExists(const std::string& path) {
        std::ifstream f(path.c_str());
        return f.good();
    }  
    
    inline Eigen::Affine3d transformToEigen(const geometry_msgs::Transform& msg) {
        Eigen::Affine3d T;
        
        // 设置平移
        T.translation() = Eigen::Vector3d(
            msg.translation.x,
            msg.translation.y,
            msg.translation.z
        );
        
        // 设置旋转
        Eigen::Quaterniond q(
            msg.rotation.w,
            msg.rotation.x,
            msg.rotation.y,
            msg.rotation.z
        );
        T.linear() = q.toRotationMatrix();
        
        return T;
    }

    inline bool saveMatrixToTxt(const Eigen::Matrix4f& pose_matrix, const std::string& file_path) {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_path.c_str());
            return false;
        }

        // 设置输出格式：固定小数位，保留4位小数
        file << std::fixed << std::setprecision(6);

        // 按行写入矩阵
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                file << std::setw(12) << pose_matrix(i, j);  // 固定宽度对齐
                if (j < 3) file << " ";  // 列间空格分隔
            }
            file << "\n";  // 行尾换行
        }

        file.close();
        std::cout << "Get Extrinsic Matrix: \n";
        std::cout << pose_matrix(0,0) << " " << pose_matrix(0,1) << " " << pose_matrix(0,2) << " " << pose_matrix(0,3) << std::endl; 
        std::cout << pose_matrix(1,0) << " " << pose_matrix(1,1) << " " << pose_matrix(1,2) << " " << pose_matrix(1,3) << std::endl;
        std::cout << pose_matrix(2,0) << " " << pose_matrix(2,1) << " " << pose_matrix(2,2) << " " << pose_matrix(2,3) << std::endl;
        std::cout << pose_matrix(3,0) << " " << pose_matrix(3,1) << " " << pose_matrix(3,2) << " " << pose_matrix(3,3) << std::endl;
        return true;
    }

    inline bool loadMatrixFromTxt(const std::string& file_path, Eigen::Matrix4f& output_matrix) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open matrix file: %s", file_path.c_str());
            return false;
        }

        output_matrix.setIdentity(); // 初始化为单位矩阵
        std::string line;
        int row = 0;

        while (std::getline(file, line) && row < 4) {
            // 跳过空行和注释行（以#开头的行）
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            for (int col = 0; col < 4; ++col) {
                if (!(iss >> output_matrix(row, col))) {
                    ROS_ERROR("Format error at row %d, col %d", row+1, col+1);
                    return false;
                }
            }
            ++row;
        }

        if (row != 4) {
            ROS_ERROR("Incomplete matrix data, only got %d rows", row);
            return false;
        }

        std::cout << "Extrinsic Matrix: \n";
        std::cout << output_matrix(0,0) << " " << output_matrix(0,1) << " " << output_matrix(0,2) << " " << output_matrix(0,3) << std::endl; 
        std::cout << output_matrix(1,0) << " " << output_matrix(1,1) << " " << output_matrix(1,2) << " " << output_matrix(1,3) << std::endl;
        std::cout << output_matrix(2,0) << " " << output_matrix(2,1) << " " << output_matrix(2,2) << " " << output_matrix(2,3) << std::endl;
        std::cout << output_matrix(3,0) << " " << output_matrix(3,1) << " " << output_matrix(3,2) << " " << output_matrix(3,3) << std::endl;
        return true;
    }

    inline void draw9GridRegions(cv::Mat& display_image) {
        if (display_image.empty()) {
            std::cerr << "Error: Empty image!" << std::endl;
            return;
        }
        
        int img_height = display_image.rows;
        int img_width = display_image.cols;
        
        // Calculate grid dimensions (3x3 grid)
        int grid_rows = 3;
        int grid_cols = 3;
        int cell_height = img_height / grid_rows;
        int cell_width = img_width / grid_cols;
        
        // Purple color (BGR format: Blue, Green, Red)
        cv::Scalar purple_color(255, 0, 255);  // Purple in BGR
        int line_thickness = 1;
        
        // Draw horizontal lines
        for (int i = 1; i < grid_rows; i++) {

            int y = i * cell_height;
            cv::line(display_image, 
                    cv::Point(0, y), 
                    cv::Point(img_width, y), 
                    purple_color, 
                    line_thickness);
        }
        
        // Draw vertical lines
        for (int i = 1; i < grid_cols; i++) {
            int x = i * cell_width;
            cv::line(display_image, 
                    cv::Point(x, 0), 
                    cv::Point(x, img_height), 
                    purple_color, 
                    line_thickness);
        }
        
        // Optional: Add grid labels (1-9)
        cv::Scalar text_color(255, 255, 255);  // White text
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.8;
        int thickness = 2;
        

        for (int row = 0; row < grid_rows; row++) {
            for (int col = 0; col < grid_cols; col++) {
                int grid_number = row * 3 + col + 1;
                int x_pos = col * cell_width + cell_width / 2 - 10;
                int y_pos = row * cell_height + cell_height / 2 + 10;
                
                cv::putText(display_image, 
                        std::to_string(grid_number),
                        cv::Point(x_pos, y_pos),
                        font_face,
                        font_scale,
                        text_color,
                        thickness);
            }
        }
    }

    inline void debug_show_images(cv::Mat &show_image, cv::Mat &mat1, cv::Mat &mat2,cv::Mat &mat3,cv::Mat &mat4,cv::Mat &mat5, cv::Mat &mat6)
    {

        // 将mat1,mat2,mat3黑白颜色颠倒
        cv::bitwise_not(mat1, mat1);
        cv::bitwise_not(mat2, mat2);
        cv::bitwise_not(mat3, mat3);
        // 将mat4和mat6的背景替换成白色
        cv::bitwise_not(mat4, mat4);
        cv::bitwise_not(mat6, mat6);
        // Define positions for each image (2 rows x 3 columns)
        int img_width = mat1.cols;
        int img_height = mat1.rows;

        std::vector<cv::Rect> rois = {
            cv::Rect(0, 0, img_width, img_height),              // Position 1: Top-left
            cv::Rect(img_width, 0, img_width, img_height),      // Position 2: Top-middle
            cv::Rect(img_width * 2, 0, img_width, img_height),  // Position 3: Top-right
            cv::Rect(0, img_height, img_width, img_height),     // Position 4: Bottom-left
            cv::Rect(img_width, img_height, img_width, img_height), // Position 5: Bottom-middle
            cv::Rect(img_width * 2, img_height, img_width, img_height)
            // Position 6 (bottom-right) can be left empty or used for additional info
        };

        // Images and their labels
        std::vector<std::pair<cv::Mat, std::string>> image_data = {
            {mat1, "Binary"},
            {mat2, "Closing"},
            {mat3, "Skeleton"},
            {mat4, "Lines"},
            {mat5, "Debug"},
            {mat6, "lines_x_direction"}
        };

        // Function to prepare image with label
        auto prepareImageWithLabel = [](const cv::Mat& src, const std::string& label) -> cv::Mat {
            if (src.empty()) return cv::Mat();
            
            cv::Mat result;
            
            // Convert to 3-channel if needed
            if (src.channels() == 1) {
                cv::cvtColor(src, result, cv::COLOR_GRAY2BGR);
            } else {
                result = src.clone();
            }
            
            // Add label
            cv::putText(result, label, 
                        cv::Point(10, 25), 
                        cv::FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        cv::Scalar(0, 255, 0), // Yellow color
                        2);
            
            return result;
        };

        // Copy each labeled image to its position
        for (size_t i = 0; i < image_data.size() && i < rois.size(); ++i) {
            cv::Mat labeled_img = prepareImageWithLabel(image_data[i].first, image_data[i].second);
            if (!labeled_img.empty()) {
                // Resize if necessary to fit the ROI
                cv::Mat resized_img;
                if (labeled_img.rows != img_height || labeled_img.cols != img_width) {
                    cv::resize(labeled_img, resized_img, cv::Size(img_width, img_height));
                } else {
                    resized_img = labeled_img;
                }
                resized_img.copyTo(show_image(rois[i]));
            }
        }

        // Display the combined image
       
        // cv::imshow("All_Processing_Steps", show_image);
        // cv::waitKey(1);
    }

    inline float angleBetween(const cv::Vec4i& line1, const cv::Vec4i& line2) {
        cv::Point2f v1(line1[2] - line1[0], line1[3] - line1[1]);  // line1 方向向量
        cv::Point2f v2(line2[2] - line2[0], line2[3] - line2[1]);  // line2 方向向量
        // 计算点积
        float dot = v1.x * v2.x + v1.y * v2.y;
        // 计算向量的模
        float len1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        float len2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
        // 避免除零
        if (len1 < 1e-6 || len2 < 1e-6) {
            return 0.0f;  // 无效的直线方向
        }
        // 计算 cos(θ)
        float cos_theta = dot / (len1 * len2);
        // 限制在 [-1, 1] 范围内，避免浮点误差
        cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
        // 计算夹角（弧度）
        float angle_rad = std::acos(cos_theta);
        // 转为角度 [0°, 180°]
        float angle_deg = angle_rad * 180.0f / CV_PI;
        
        if (angle_deg < 30 || angle_deg > 150) return angle_deg;
        // 保证夹角始终是在两直线相交的右下角
        if (fabs(v1.x) > fabs(v1.y)) // l2是横向钢筋的情况
        {
            if ((v1.x > 0 && v2.y > 0)) return angle_deg;
            if ((v1.x < 0 && v2.y > 0)) return 180 - angle_deg;
            if ((v1.x > 0 && v2.y < 0)) return 180 - angle_deg;
            if ((v1.x < 0 && v2.y < 0)) return angle_deg;  
        }
        else if (fabs(v2.x) > fabs(v2.y)) // l1是横向钢筋的情况
        {
            if ((v2.x > 0 && v1.y > 0)) return angle_deg;
            if ((v2.x < 0 && v1.y > 0)) return 180 - angle_deg;
            if ((v2.x > 0 && v1.y < 0)) return 180 - angle_deg;
            if ((v2.x < 0 && v1.y < 0)) return angle_deg; 
        }
        return 45;
    }

    inline void pubCamGripperTf(Eigen::Matrix4f T){
        // 可视化验证
        static tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transform;
        // 1. 填写 header
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "Scepter_color_frame";    // 父坐标系，例如 "aruco_frame"
        transform.child_frame_id = "gripper_frame";      // 子坐标系，例如 "gripper_frame"

        // 2. 填写平移 [单位：米]
        transform.transform.translation.x = T(0,3); // m
        transform.transform.translation.y = T(1,3);
        transform.transform.translation.z = T(2,3);

        // 3. 填写旋转：绕 Y 轴旋转 yaw_degrees（默认 90°）
        // double radians = 180 * M_PI / 180.0;
        // tf2::Quaternion q;
        // q.setRPY(0.0, radians, 0.0);  // 绕 Y 轴旋转
        Eigen::Matrix3f rotation_matrix = T.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation_matrix);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        broadcaster.sendTransform(transform);
    }

    inline void printCurrentTime() {
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();

        // 转换为 time_t（秒级，UTC）
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

        // 转换为本地时间
        std::tm local_time;
    #if defined(_WIN32) || defined(_WIN64)
        localtime_s(&local_time, &now_time_t);  // Windows 安全版本
    #else
        localtime_r(&now_time_t, &local_time);  // POSIX 标准，线程安全
    #endif

        // 获取毫秒部分
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        // 格式化日期时间（本地时间），精确到秒
        char time_str[20];  // 足够存放 "2024-06-01 14:03:45" + '\0'
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_time);

        // 打印完整时间（含毫秒）与分隔符
        printf("%s.%03d - \n", time_str, static_cast<int>(ms.count()));
    }
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageColorCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageInfraredCallback(const sensor_msgs::ImageConstPtr& msg);
    void maxDepthOffsetCallback(const std_msgs::Int32ConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg); // 自动标定
    
    // 服务处理函数
    bool handleProcessImage(fast_image_solve::ProcessImage::Request& req,
                          fast_image_solve::ProcessImage::Response& res);
    // bool detectAndSavePose(std_srvs::Trigger::Request& req,
    //                       std_srvs::Trigger::Response& res);
    
    // 工具函数
    std::vector<std::pair<cv::Point2f, float>> clusterPointsWithValues(
        const std::vector<std::pair<cv::Point2f, float>>& points, float distance_threshold, const std::string& float_agg_method); 
    std::vector<std::pair<cv::Point2f, float>> calculateIntersections(const std::vector<cv::Vec4i>& lines, float dx_thr, float dy_thr);
    void drawTextWithBackground(cv::Mat& image, const std::string& text, const cv::Point& position,
                               int font = cv::FONT_HERSHEY_SIMPLEX, double font_scale = 0.23,
                               const cv::Scalar& text_color = cv::Scalar(255, 255, 255),
                               const cv::Scalar& bg_color = cv::Scalar(0, 0, 0), int thickness = 1);
    
    // void mouseCallback(int event, int x, int y, int flags, void* param);
    // void testCallback();
    // void callLinearModuleMoveService(int pos_x, int pos_y, int pos_z);
    bool saveImage(int x, int y, int z_value); // 检测水管
    std::pair<Eigen::Vector3f, Eigen::Matrix3f> transformToEndEffector(float x_obj, float y_obj, float z_obj, float theta_obj);
    Eigen::Matrix4f createRotationMatrix(float theta);
    std::vector<std::pair<cv::Point2f, float>> snakeSortByColumn(const std::vector<std::pair<cv::Point2f, float>>& points,float row_threshold);
    std::vector<cv::Point3f> snakeSort(const std::vector<cv::Point3f>& centers, float row_threshold = 50.0f);
    void publishTfTransform();
    void publishGripperTfTransform();
    Eigen::Vector3f getGripperRelativeTranslation();
    Eigen::Vector3f transformToGripperFrame(float x, float y, float z, int idx);
    Eigen::Affine3f getScepterGripper(Eigen::Matrix4f & pose_base2target);
    
    // 图像处理主函数
    std::vector<std::pair<cv::Point2f, float>> UseFullRegionToGetPoint(cv::Mat &in_img, cv::Mat &closing_, cv::Mat &line_image_, cv::Mat &line_image_x, cv::Mat &skeleton_,
                                                                       int &thinning_size_, int &minLineLength_, int &maxLineGap_ , int &threshold_, int &iterations_);
    std::vector<std::pair<cv::Point2f, float>> UseNineRegionToGetPoint(int rows, int cols, cv::Mat &in_img, cv::Mat &closing_, cv::Mat &line_image_, cv::Mat &line_image_x, cv::Mat &skeleton_,
                                                                       int &thinning_size_, int &minLineLength_, int &maxLineGap_ , int &threshold_, int &iterations_);
    fast_image_solve::PointsArray preImg(cv::Mat &three_channel_image);
    fast_image_solve::PointsArray result;
};

#endif // IMAGE_PROCESSOR_HPP