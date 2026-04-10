/*
    功能：针对钢筋面的处理
    流程：高斯滤波 -> 直方图增强 -> 大律法二值化 -> 边缘直线提取 -> 保留平行度阈值和距离阈值内的直线作为钢筋线
*/

#include "fast_image_solve/fast_image.hpp"

FastImageSolve::FastImageSolve(ros::NodeHandle& nh, ros::NodeHandle& private_nh) 
    : nh_(nh), it_(nh){
    // 初始化参数
    private_nh.param("min_Image_area", min_Image_area_, 50);
    private_nh.param("line_angle_threshold", line_angle_threshold_, 10.0);
    private_nh.param("cross_point_min_angle", cross_point_min_angle_, 10.0f);
    // private_nh.param("grid_merge_size", grid_merge_size_, 10.0f);
    

    image_sub_ = it_.subscribe("input_image", 1, &FastImageSolve::imageCallback, this);
    imagesolve_service = nh_.advertiseService("image_solve", &FastImageSolve::imagesolveService, this);

    result_pub_ = it_.advertise("Image_detection_result", 1);
    binary_pub_ = it_.advertise("binary_detection_result", 1);
    
    ROS_INFO("Image Detector initialized");
}

FastImageSolve::~FastImageSolve() {
    ROS_INFO("Shutting down Image Detector");
}

void FastImageSolve::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        std::lock_guard<std::mutex> lock(image_mutex_);
        msg_ = msg;
        // 转换ROS图像消息为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        input_img = cv_ptr->image;
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool FastImageSolve::imagesolveService( fast_image_solve::image::Request &req, 
                     fast_image_solve::image::Response &res){
    
    
    if (req.action)
    {               
        std::cout <<  req.action << std::endl;
        try {

            std::lock_guard<std::mutex> lock(image_mutex_);
            cv::Mat input_image = input_img.clone();
            
        // 执行检测流程
            std::cout << "###############################\n";
            float t0 = omp_get_wtime();
            cv::Mat binary = extractImage(input_image);
            float t1 = omp_get_wtime();
            std::cout << "Binary Cost Time : " << (t1 - t0) * 1000 << " ms\n";

            float t2 = omp_get_wtime();
            std::vector<cv::Vec4i> lines = detectLines(binary);
            float t3 = omp_get_wtime();
            std::cout << "DetctLines Cost Time : " << (t3 - t2) * 1000 << " ms\n";

            float t4 = omp_get_wtime();
            std::vector<cv::Point2f> cross_points = findCrossPoints(lines, input_image.size());
            float t5 = omp_get_wtime();
            std::cout << "FindCrossPoints Cost Time : " << (t5 - t4) * 1000 << " ms\n";
            std::cout << "###############################\n\n";
            
            // 可视化结果
            cv::Mat result = visualizeResults(input_image, lines, cross_points);

            // 发布结果图像
            cv_bridge::CvImage binary_msg;
            binary_msg.header = msg_->header;
            binary_msg.encoding = sensor_msgs::image_encodings::MONO8;
            binary_msg.image = binary;
            binary_pub_.publish(binary_msg.toImageMsg());
            
            // 发布结果图像
            cv_bridge::CvImage out_msg;
            out_msg.header = msg_->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = result;
            result_pub_.publish(out_msg.toImageMsg());

            res.solve_or = true;
            return res.solve_or;

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
    }
}

cv::Mat FastImageSolve::extractImage(const cv::Mat& input) {
    cv::Mat gray, enhanced, binary;

    assert(input.type() == CV_8UC3);  // 确保输入是 BGR 格式
    // 1. 灰度化
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.0);
    // cv::equalizeHist(blurred, blurred); 
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->setTilesGridSize(cv::Size(8,8));
    enhanced = blurred.clone();
    clahe->apply(blurred, enhanced);

    // 2. 大津法二值化
    cv::threshold(enhanced, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // 3. 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    
    // 4. 连通域过滤
    cv::Mat labels, stats, centroids;
    int num_objects = cv::connectedComponentsWithStats(binary, labels, stats, centroids);
    
    cv::Mat filtered_binary = cv::Mat::zeros(binary.size(), CV_8UC1);
    for (int i = 1; i < num_objects; i++) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) > min_Image_area_) {
            cv::Mat mask = (labels == i);
            filtered_binary.setTo(255, mask);
        }
    }
    
    return filtered_binary;
}

std::vector<cv::Vec4i> FastImageSolve::detectLines(const cv::Mat& binary) {
    // 边缘检测
    cv::Mat edges;
    cv::Canny(binary, edges, 0, 0, 3, true);
    
    // 概率霍夫变换
    std::vector<cv::Vec4i> lines;
    int threshold = std::max(50, static_cast<int>(binary.cols * 0.08));
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, threshold, 
                   binary.rows*0.1, binary.rows*0.02);
    
    // 过滤并合并
    double angle_thresh = line_angle_threshold_ * CV_PI/180;
    std::vector<cv::Vec4i> filtered_lines = mergeNearbyHorizontalLines(lines, angle_thresh, 20.0);
    
    return filtered_lines;
}

std::vector<cv::Point2f> FastImageSolve::findCrossPoints(
    const std::vector<cv::Vec4i>& lines, 
    const cv::Size& img_size) 
{
    std::vector<cv::Point2f> cross_points;
    float min_angle_rad = cross_point_min_angle_ * CV_PI / 180.0f;
    
    // 转换为参数形式
    std::vector<cv::Vec2f> param_lines;
    for (const auto& line : lines) {
        float dx = line[2] - line[0];
        float dy = line[3] - line[1];
        float length = std::sqrt(dx*dx + dy*dy);
        
        if (length < 1e-5) continue;
        
        float nx = -dy / length;
        float ny = dx / length;
        float rho = line[0] * nx + line[1] * ny;
        float theta = std::atan2(ny, nx);
        if (theta < 0) theta += CV_PI;
        
        param_lines.emplace_back(rho, theta);
    }
    
    // 计算交点
    for (size_t i = 0; i < param_lines.size(); ++i) {
        for (size_t j = i + 1; j < param_lines.size(); ++j) {
            float rho1 = param_lines[i][0], theta1 = param_lines[i][1];
            float rho2 = param_lines[j][0], theta2 = param_lines[j][1];
            
            float angle_diff = std::abs(theta1 - theta2);
            angle_diff = std::min(angle_diff, static_cast<float>(CV_PI - angle_diff));
            if (angle_diff < min_angle_rad) continue;
            
            float det = std::cos(theta1) * std::sin(theta2) - 
                       std::sin(theta1) * std::cos(theta2);
            if (std::abs(det) < 1e-5) continue;
            
            float x = (std::sin(theta2) * rho1 - std::sin(theta1) * rho2) / det;
            float y = (std::cos(theta1) * rho2 - std::cos(theta2) * rho1) / det;
            
            if (x >= 0 && x < img_size.width && y >= 0 && y < img_size.height) {
                cross_points.emplace_back(x, y);
            }
        }
    }
    
    return cross_points;
}

// 计算两条线段的中点距离
double FastImageSolve::distanceBetweenLines(const cv::Vec4i& line1, const cv::Vec4i& line2) {
    cv::Point2f mid1((line1[0] + line1[2]) * 0.5f);
    cv::Point2f mid2((line2[0] + line2[2]) * 0.5f);
    return cv::norm(mid1 - mid2);
}

// 主函数：过滤水平线段并合并邻近线段
std::vector<cv::Vec4i> FastImageSolve::mergeNearbyHorizontalLines(
    const std::vector<cv::Vec4i>& lines,
    double angle_thresh_deg,   // 角度阈值（度）
    double distance_thresh     // 中点距离阈值（像素）
) {
    std::vector<cv::Vec4i> filtered_lines;
    double angle_thresh = angle_thresh_deg * CV_PI / 180.0;

    // 1. 过滤出接近水平的线段（角度小于阈值）
    for (const auto& line : lines) {
        double dx = line[2] - line[0];
        double dy = line[3] - line[1];
        double angle = std::atan2(std::abs(dy), std::abs(dx));
        
        if (angle < angle_thresh) {  // 仅保留接近水平的线段
            filtered_lines.push_back(line);
        }
    }

    // 2. 合并中点距离小于阈值的线段
    std::vector<cv::Vec4i> merged_lines;
    std::vector<bool> merged(filtered_lines.size(), false);

    for (size_t i = 0; i < filtered_lines.size(); ++i) {
        if (merged[i]) continue;

        std::vector<cv::Vec4i> cluster;
        cluster.push_back(filtered_lines[i]);

        // 查找邻近线段
        for (size_t j = i + 1; j < filtered_lines.size(); ++j) {
            if (merged[j]) continue;

            if (distanceBetweenLines(filtered_lines[i], filtered_lines[j]) < distance_thresh) {
                cluster.push_back(filtered_lines[j]);
                merged[j] = true;
            }
        }

        // 3. 生成中位线（取起点和终点的平均值）
        if (!cluster.empty()) {
            float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            for (const auto& line : cluster) {
                x1 += line[0];
                y1 += line[1];
                x2 += line[2];
                y2 += line[3];
            }
            x1 /= cluster.size();
            y1 /= cluster.size();
            x2 /= cluster.size();
            y2 /= cluster.size();
            merged_lines.emplace_back(x1, y1, x2, y2);
        }
    }

    return merged_lines;
}

cv::Mat FastImageSolve::visualizeResults(
    const cv::Mat& image,
    const std::vector<cv::Vec4i>& lines,
    const std::vector<cv::Point2f>& cross_points) 
{
    cv::Mat result = image.clone();
    
    // 绘制检测到的线段
    for (const auto& line : lines) {
        cv::line(result, 
                cv::Point(line[0], line[1]),
                cv::Point(line[2], line[3]), 
                cv::Scalar(0, 255, 0), 2);
    }
    
    // 绘制交叉点
    for (const auto& pt : cross_points) {
        cv::circle(result, pt, 8, cv::Scalar(0, 0, 255), -1);
    }
    
    return result;
}