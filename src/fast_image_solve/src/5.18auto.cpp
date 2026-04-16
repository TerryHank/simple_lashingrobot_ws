#include "fast_image_solve/5.18auto.hpp"

ImageProcessor::ImageProcessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : 
    nh_(nh), params_nh(private_nh),
    tf_listener_(tf_buffer_),
    threshold_(50),
    minLineLength_(60),
    maxLineGap_(350),
    min_depth_(910), max_depth_(1060),
    max_depth_offset_(90),
    offset_x_(0), offset_y_(0), offset_z_(0),
    half_size_(30),
    non_shuiguan_count_(0), shuiguan_count_(0),
    displacement_x_(0), displacement_y_(0), displacement_z_(0),
    region_x_(0), region_y_(0), region_z_(0),
    x1_(0), y1_(0), x2_(640), y2_(140),
    x3_(520), y3_(140), x4_(640), y4_(480),
    frame_count_(0), fps_(0) 
    {
    
    std::cout << COLOR_GREEN << "########Image Solve Node started########" << COLOR_RESET << std::endl;
    params_nh.param("Preprocess/threshold", threshold_, 50);
    params_nh.param("Preprocess/minLineLength", minLineLength_, 60);
    params_nh.param("Preprocess/maxLineGap_", maxLineGap_, 350);
    params_nh.param("Preprocess/min_depth", min_depth_, 910);
    params_nh.param("Preprocess/max_depth", max_depth_, 1060);
    params_nh.param("Preprocess/max_depth_offset", max_depth_offset_, 90);
    params_nh.param("Preprocess/offset_x", offset_x_, 0);
    params_nh.param("Preprocess/offset_y", offset_y_, 2);
    params_nh.param("Preprocess/offset_y", offset_z_, 2);
    params_nh.param("Preprocess/half_size", half_size_, 30);
    params_nh.param("Preprocess/non_shuiguan_count", non_shuiguan_count_, 0);
    params_nh.param("Preprocess/shuiguan_count", shuiguan_count_, 0);
    params_nh.param("Preprocess/displacement_x", displacement_x_,200);
    params_nh.param("Preprocess/displacement_y", displacement_y_, 0);
    params_nh.param("Preprocess/displacement_z", displacement_z_, 130);
    params_nh.param("Preprocess/region_x", region_x_, 130);
    params_nh.param("Preprocess/region_y", region_y_, 130);
    params_nh.param("Preprocess/region_z", region_z_, 130);
    params_nh.param("Preprocess/x1", x1_, 0);
    params_nh.param("Preprocess/y1", y1_, 0);
    params_nh.param("Preprocess/x2", x2_, 640);
    params_nh.param("Preprocess/y2", y2_, 140);
    params_nh.param("Preprocess/x3", x3_, 520);
    params_nh.param("Preprocess/y3", y3_, 140);
    params_nh.param("Preprocess/x4", x4_, 640);
    params_nh.param("Preprocess/y4", y4_, 480);
    params_nh.param("Preprocess/frame_count", frame_count_, 0);
    params_nh.param("Preprocess/fps", fps_, 0.f);
    params_nh.param("Preprocess/ground_h", ground_h_, 15.0f);
    params_nh.param("Preprocess/pixel_clsuter", pixel_clsuter_, 12);
    params_nh.param("Preprocess/medianBlur", medianBlur_, 5);
    params_nh.param("Preprocess/thinning_size", thinning_size_, 5);
    params_nh.param("Preprocess/iterations", iterations_, 1);
    params_nh.param("Preprocess/use_nineregion", use_nineregion_, true);
    params_nh.param("Preprocess/pose_matrix_path", pose_matrix_path,std::string("/home/hyq/free_walking-bind-robot/src/fast_image_solve/config/pose_matrix.txt"));
    params_nh.param("Preprocess/nineregion_data_path", nineregion_data_path, std::string("/home/hyq/hyq-ros-wrapper/free_walking-bind-robot/src/fast_image_solve/data/usenine.jpg"));
    params_nh.param("Preprocess/fullregion_data_path", fullregion_data_path, std::string("/home/hyq/hyq-ros-wrapper/free_walking-bind-robot/src/fast_image_solve/data/usefull.jpg"));
    params_nh.param("Topics/world_image_topic", self_defination_image_topic,std::string("/main_image_topic"));
    params_nh.param("Topics/color_image_topic", color_image_topic,std::string("/color_image_topic"));
    params_nh.param("Topics/ir_image_topic", ir_image_topic,std::string("/depth_image_topic"));
    params_nh.param("Topics/camera_info_topic", camera_info_topic,std::string("/camera_info_topic"));
    
    // 初始化订阅者
    image_sub_ = nh_.subscribe(self_defination_image_topic, 5, &ImageProcessor::imageCallback, this);
    color_image_sub_ = nh_.subscribe(color_image_topic, 5, &ImageProcessor::imageColorCallback, this);
    infrared_image_sub_ = nh_.subscribe(ir_image_topic, 5, &ImageProcessor::imageInfraredCallback, this);
    max_depth_offset_sub_ = nh_.subscribe("/max_depth_offset", 5, &ImageProcessor::maxDepthOffsetCallback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic, 5, &ImageProcessor::cameraInfoCallback, this);
    
    // 初始化发布者
    image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/image_result/show", 10);
    coordinate_pub_ = nh_.advertise<fast_image_solve::PointsArray>("/image_coordinate_points", 10);
    show_image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/fast_image_node/image_result", 10);
    
    // 初始化服务
    process_image_srv_ = nh_.advertiseService("/Moduan/process_image", &ImageProcessor::handleProcessImage, this);
    // 自动标定
    // detect_and_save_pose_srv_ = nh_.advertiseService("detect_and_save_pose", &ImageProcessor::detectAndSavePose, this);
    
    // 设置默认相机内参
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 640, 0, 320, 0, 640, 240, 0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_32F);
    
    start_time_ = ros::Time::now();

    loadMatrixFromTxt(pose_matrix_path,pose_matrix); // get camera2aruco
    T_camera_ee = pose_matrix;
    // T_scepter_gripper = getScepterGripper(pose_matrix);
}
// 图像回调函数实现
void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,"32FC3");
        {
            std::lock_guard<std::mutex> lock_3channel_image(_channel_image);
            image_ = cv_ptr->image.clone();
            vison_image_ = image_.clone();
        }
        // 归一化处理
        cv::normalize(vison_image_, vison_image_, 0, 255, cv::NORM_MINMAX);
        vison_image_.convertTo(vison_image_, CV_8UC3);
        cv::applyColorMap(vison_image_, vison_image_, cv::COLORMAP_JET);

        // cv::imshow("JET image", vison_image_);
        // cv::waitKey(1);
        
}

// 其他回调函数实现
void ImageProcessor::imageColorCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_color_copy_ = cv_ptr->image.clone();//image_color_.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageProcessor::imageInfraredCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "8UC1");
        image_infrared_copy_ = cv_ptr->image.clone();//image_infrared_.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageProcessor::maxDepthOffsetCallback(const std_msgs::Int32ConstPtr& msg) {
    max_depth_offset_ = msg->data;
}

//加载相机内参
void ImageProcessor::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    static int load_cnt = 0;
    if (load_cnt == 0)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_32F, const_cast<double*>(msg->K.data())).clone();
        dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_32F, const_cast<double*>(msg->D.data())).clone();
        ROS_INFO("Load camera information successfully!\n");
        load_cnt++;
    }
}

// 服务处理函数实现
bool ImageProcessor::handleProcessImage(fast_image_solve::ProcessImage::Request& req,
                                      fast_image_solve::ProcessImage::Response& res){
    cv::Mat three_channel_image;
    {
        std::lock_guard<std::mutex> lock_3channel_image(_channel_image);
        three_channel_image = image_.clone();
    }                               
    if (three_channel_image.empty()) {
        printCurrentTime();
        printf(COLOR_RED "当前服务未收到自定义编码图像\n" COLOR_RESET);
        res.count = 0;
        return false;
    }
    result = preImg(image_);
    res.count = result.count;
    res.PointCoordinatesArray = result.PointCoordinatesArray;
    // if (result.count > 0) {
    //         coordinate_pub_.publish(result);
    // }
    return true;
}

// 核心图像处理函数
fast_image_solve::PointsArray ImageProcessor::preImg(cv::Mat &three_channel_image) {
    static int image_count = 0;
    float t0 = omp_get_wtime();
    fast_image_solve::PointsArray result;
    // 普通图像测试
    // cv::Mat display_image = image_color_copy_.clone();
    cv::Mat display_image = image_infrared_copy_.clone();
    
    // 通道分离
    std::vector<cv::Mat> channels;
    cv::split(three_channel_image, channels);
    x_channel = channels[0];
    y_channel = channels[1];
    Depth_image_Raw_ = channels[2];

    // ROI处理
    Depth_image_Raw_(cv::Rect(x1_, y1_, x2_ - x1_, y2_ - y1_)).setTo(0);
    Depth_image_Raw_(cv::Rect(x3_, y3_, x4_ - x3_, y4_ - y3_)).setTo(0);
    Depth_image_Raw_(cv::Rect(x1_, y3_, x4_ - x3_ + 30, y4_ - y3_)).setTo(0);
    Depth_image_Raw_(cv::Rect(x4_ - x3_ + 30, y4_ - (x4_ - x3_), x4_ - 2 * (x4_ - x3_) - 30, x4_ - x3_)).setTo(0);
    // 计算最大深度值并减去地面阈值
    double minVal, maxVal;
    cv::minMaxLoc(Depth_image_Raw_, &minVal, &maxVal);
    int max_depth = static_cast<int>(maxVal) - ground_h_; //进一步过滤残留模板面点
    // 创建深度范围图像
    cv::inRange(Depth_image_Raw_, 10, max_depth, Depth_image_Range_);
    // 深度图像处理
    cv::normalize(Depth_image_Range_, Depth_image_Raw_uni_, 0, 255, cv::NORM_MINMAX); // 归一化
    cv::threshold(Depth_image_Raw_uni_, Depth_image_Raw_binary_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // cv::threshold(Depth_image_Raw_binary_, Depth_image_Raw_binary_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::medianBlur(Depth_image_Raw_binary_, Depth_image_Raw_binary_, medianBlur_);

    std::vector<std::pair<cv::Point2f, float>> results;
    if (use_nineregion_)
    {
                                                                                       
         results = UseNineRegionToGetPoint(3, 3, Depth_image_Raw_binary_, closing_, line_image_, line_image_x, skeleton_,
                                                                                    thinning_size_, minLineLength_, maxLineGap_, threshold_, iterations_);
    }else{
         results = UseFullRegionToGetPoint(Depth_image_Raw_binary_, closing_, line_image_, line_image_x, skeleton_,
                                                                                    thinning_size_, minLineLength_, maxLineGap_, threshold_, iterations_);
    }
   int results_size = results.size();
    printCurrentTime();
    printf(COLOR_CYAN "RESULTS SIZE: %d\n" COLOR_RESET, results_size);
    
    //  world_pts恢复到末端执行系
    if (!results.empty()) {
        for (size_t i = 0; i < results.size(); ++i)
        {
            // pixel_pts坐标查询获取world_pts世界坐标属性
            int pix_x = std::round(results[i].first.x); // pixel_x
            int pix_y = std::round(results[i].first.y); // pixel_y
            float world_x = x_channel.at<float>(pix_y, pix_x); // mm
            float world_y = y_channel.at<float>(pix_y, pix_x);
            float world_z = Depth_image_Raw_.at<float>(pix_y, pix_x);
            ROS_WARN("z: %f\n", Depth_image_Raw_.at<float>(pix_y, pix_x));
            fast_image_solve::PointCoords point;
            point.idx = i;
            point.Pix_coord = {pix_x, pix_y}; // 列行
            // 转换到gripper坐标系
            Eigen::Vector3f gripper_coords = transformToGripperFrame(world_x, world_y, world_z, point.idx);
            point.World_coord = {gripper_coords[0], gripper_coords[1], gripper_coords[2]}; // 负号是转换图像X轴到末端+x
            // 判断是否为水管
            point.is_shuiguan = false; //saveImage(centers[i].x, centers[i].y, centers[i].z) // 使用的yolo推理;

            // 由于绑扎枪把影响最佳旋转作以下限制：保证把始终朝向识别中心 
            if (point.World_coord[0] > 0)
            {
                point.Angle = -(180 - results[i].second/2);
                if (point.World_coord[1] > region_y_ - 50) // 50 -> y方向极限绑扎差值
                    point.Angle = 90 + results[i].second/2;    
            }
            else
            {
                point.Angle = -(180 - results[i].second)/2;
                if (point.World_coord[1] > region_y_ - 50)
                    point.Angle = results[i].second/2;
            }
            result.PointCoordinatesArray.push_back(point);
        }
        result.count = result.PointCoordinatesArray.size();
    }
    // 帧率计算
    float t1 = omp_get_wtime(); 
    fps_ = 1 / (t1 - t0);
    ROS_INFO("Precess image : %f ms\n", (t1 - t0) * 1000);
    drawTextWithBackground(display_image, "FPS: " + std::to_string(fps_), cv::Point(10, 10));
    // display pts in current image_infrared_copy_
     if (!result.PointCoordinatesArray.empty()) {
            for (const auto& point : result.PointCoordinatesArray) {
                // std::string text = std::to_string(point.idx) + ", " + 
                //                   std::to_string(point.World_coord[0]) + ", " +
                //                   std::to_string(point.World_coord[1]) + ", " +
                //                   (point.is_shuiguan ? "Yes" : "No");
                // drawTextWithBackground(display_image, text, 
                //                      cv::Point(point.Pix_coord[0] - 10, point.Pix_coord[1] - 10));
                // 3. OpenCV 的 ellipse() 函数中，角度是以度为单位，0 度指向正右侧（X 轴正方向），逆时针为负
                cv::ellipse(
                    display_image,
                    cv::Point(point.Pix_coord[0], point.Pix_coord[1]),          // 圆心
                    cv::Size(20, 20),               // 椭圆的长宽（这里设为相等，即为圆）
                    0,                                      // 旋转角度（通常为 0，表示不旋转椭圆）
                    0,                        // 起始角度（单位：度）
                    point.Angle,                          // 终止角度（单位：度）
                    cv::Scalar(0, 0, 255),                  // 颜色：红色 (BGR)
                    2                                       // 线宽
                );
                std::string angle_text = "Angle/2: " + std::to_string(point.Angle) + "°";
                drawTextWithBackground(display_image, angle_text, 
                                     cv::Point(point.Pix_coord[0] - 40, point.Pix_coord[1] - 20));
                if (point.World_coord[0] >= -80 && point.World_coord[0] <= region_x_ 
                    && point.World_coord[1] >= 0 && point.World_coord[1] <= region_y_)
                {
                    // 符合执行范围的点标记黑色
                    cv::circle(display_image, 
                        cv::Point(point.Pix_coord[0], point.Pix_coord[1]), 
                        3, cv::Scalar(0, 0, 0), -1);
                }
                else
                {
                    // 不符合执行范围的点标记白色
                    cv::circle(display_image, 
                        cv::Point(point.Pix_coord[0], point.Pix_coord[1]), 
                        3, cv::Scalar(255, 255, 255), -1);
                }
            }
    }
    // 发布识别结果
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format = "jpeg";
    std::vector<uchar> buf;
    cv::imencode(".jpg", display_image, buf);
    img_msg.data = std::vector<uchar>(buf.begin(), buf.end());
    image_pub_.publish(img_msg);
    cv::Mat show_image = cv::Mat::zeros(Depth_image_Raw_binary_.rows * 2, Depth_image_Raw_binary_.cols * 3, CV_8UC3);
    // 分块处理的可视化
    if (!display_image.empty() && !Depth_image_Raw_binary_.empty() 
        && !closing_.empty() && !skeleton_.empty() && !line_image_.empty())
    {
        if (use_nineregion_)
        {
            draw9GridRegions(Depth_image_Raw_binary_);
            draw9GridRegions(closing_);
            draw9GridRegions(skeleton_);
            draw9GridRegions(line_image_);
            // draw9GridRegions(display_image);
        }
        debug_show_images(show_image, Depth_image_Raw_binary_,closing_,skeleton_,line_image_,display_image, line_image_x);
        if (use_nineregion_)
            cv::imwrite(nineregion_data_path, show_image);
        else
            cv::imwrite(fullregion_data_path, show_image);
    }
    // 发布show_image
    sensor_msgs::CompressedImage show_img_msg;
    show_img_msg.header.stamp = ros::Time::now();
    show_img_msg.format = "jpeg";
    std::vector<uchar> show_buf;
    cv::imencode(".jpg", show_image, show_buf);
    show_img_msg.data = std::vector<uchar>(show_buf.begin(), show_buf.end());
    show_image_pub_.publish(show_img_msg);

    image_count++;
    std::cout << COLOR_GREEN << "No." << image_count << " image: "<< COLOR_RESET << std::endl;
    ROS_WARN("z: %f\n", Depth_image_Raw_.at<float>(240, 320));
    for (auto & pt :  result.PointCoordinatesArray)
    {
        std::cout << COLOR_GREEN << "[" << pt.idx << ", "<< pt.World_coord[0] << ", " <<  pt.World_coord[1] << ", " << pt.World_coord[2] << ", " << pt.Angle << "]" << COLOR_RESET << std::endl;
    }
    // 符合执行范围的点
    // for (auto & pt :  result.PointCoordinatesArray)
    // {
    //     if (pt.World_coord[0] < -80 || pt.World_coord[0] > region_x_ || pt.World_coord[1] < 0 || pt.World_coord[1] > region_y_ || pt.World_coord[2] < 0 || pt.World_coord[2] > region_z_) 
    //          continue;
    //     std::cout << COLOR_GREEN << "[" << pt.idx << ", "<< pt.World_coord[0] << ", " <<  pt.World_coord[1] << ", " << pt.World_coord[2] << ", " << pt.Angle << "]" << COLOR_RESET << std::endl;
    // }

    // // 测试代码
    // if(count == 8)
    // {   
    //     if (!result.PointCoordinatesArray.empty()) 
    //     {
    //         // 创建并打开txt文件
    //         std::ofstream outFile("/home/hyq/free_walking-bind-robot/src/fast_image_solve/data/NineRegioncoordinates.txt");
    //         if (!outFile.is_open()) {
    //             ROS_ERROR("Failed to open coordinates.txt for writing!");
    //         } else {
    //             // 写入表头
    //             outFile << "ID,Pixel_X,Pixel_Y,World_X,World_Y,World_Z,Is_Shuiguan\n";
    //         }
            
    //         for (const auto& point : result.PointCoordinatesArray) 
    //         {
    //             std::string text = std::to_string(point.idx) + ", " + 
    //                             std::to_string(point.World_coord[0]) + ", " +
    //                             std::to_string(point.World_coord[1]) + ", " +
    //                             std::to_string(point.World_coord[2]) + ", " +
    //                             (point.is_shuiguan ? "Yes" : "No");
                
    //             drawTextWithBackground(display_image, text, 
    //                                 cv::Point(point.Pix_coord[0] - 10, point.Pix_coord[1] - 10));
    //             cv::circle(display_image, 
    //                     cv::Point(point.Pix_coord[0], point.Pix_coord[1]), 
    //                     2, cv::Scalar(255, 255, 255), -1);
                
    //             // 写入坐标数据到文件
    //             if (outFile.is_open()) {
    //                 outFile << point.idx << ","
    //                         << point.Pix_coord[0] << ","
    //                         << point.Pix_coord[1] << ","
    //                         << point.World_coord[0] << ","
    //                         << point.World_coord[1] << ","
    //                         << point.World_coord[2] << ","
    //                         << (point.is_shuiguan ? "Yes" : "No") << "\n";
    //             }
    //         }
            
    //         // 关闭文件
    //         if (outFile.is_open()) {
    //             outFile.close();
    //             ROS_INFO("Coordinates saved to coordinates.txt");
    //         }
    //     }
        
    //         cv::imwrite("/home/hyq/free_walking-bind-robot/src/fast_image_solve/data/NineRegionimage.jpg", display_image);
    //         ros::shutdown();
    // }
    return result;
}

// 使用全尺寸图像计算交点
std::vector<std::pair<cv::Point2f, float>>  ImageProcessor::UseFullRegionToGetPoint(cv::Mat &in_img, cv::Mat &closing_, cv::Mat &line_image_, cv::Mat &line_image_x, cv::Mat &skeleton_,
                                                                    int &thinning_size_, int &minLineLength_, int &maxLineGap_ , int &threshold_, int &iterations_)
{
    // 十字交叉的形态学处理 cv::MORPH_CROSS； 全向形态学处理： cv::MORPH_RECT
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(thinning_size_, thinning_size_));
    cv::morphologyEx(in_img, closing_, cv::MORPH_OPEN, kernel, cv::Point(1,1), iterations_);
    // 细化处理
    cv::ximgproc::thinning(closing_, skeleton_, cv::ximgproc::THINNING_ZHANGSUEN);
    
    // 直线检测
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skeleton_, lines, 1, CV_PI/180, threshold_, minLineLength_, maxLineGap_);
    
    // 角度过滤
    if (!lines.empty()) {
        std::vector<cv::Vec4i> filtered_lines;
        for (const auto& line : lines) {
            float dx = line[2] - line[0];
            float dy = line[3] - line[1];
            float angle = (dx == 0) ? 90.0f : std::abs(std::atan(dy / dx) * 180.0f / CV_PI);
            float angle_diff = std::min(std::fmod(angle, 90), 90 - std::fmod(angle, 90));
            
            if (angle_diff <= 30) {
                filtered_lines.push_back(line);
            }
        }
        lines = filtered_lines;
    }
    // 计算交点
    std::vector<std::pair<cv::Point2f, float>> results = calculateIntersections(lines, 320.0, 50.0);
    // 像素点聚类
    auto cluster_results_ = clusterPointsWithValues(results, pixel_clsuter_, "avg");
    auto cluster_results = snakeSortByColumn(cluster_results_, 15.f);

    //绘制检测到的直线
    line_image_ = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC3);
    line_image_x = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC3);
    if (!lines.empty())
    {
        for (const auto& line : lines) {
            float dx = line[2] - line[0];
            float dy = line[3] - line[1];
            if (fabs(dy) < fabs(dx)) 
            {   
                cv::line(line_image_x, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 0), 2);
                cv::line(line_image_x, cv::Point(line[0] + fabs(dx)/2 - 15, line[1]), cv::Point(line[0] + fabs(dx)/2 + 15, line[1]), cv::Scalar(0, 0, 255), 2.5);
                cv::line(line_image_x, cv::Point(line[0] + fabs(dx)/2, line[1] - 15), cv::Point(line[0] + fabs(dx)/2, line[1] + 15), cv::Scalar(0, 0, 255), 2.5);
            }
            cv::line(line_image_, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
        }
    }
    ROS_WARN("Before cluster pix_pts number: %d\n", (int)results.size());
    ROS_WARN("After cluster pix_pts number: %d\n", (int)cluster_results.size());
    return cluster_results;
}

// 使用9宫格划分区域计算交点
std::vector<std::pair<cv::Point2f, float>> ImageProcessor::UseNineRegionToGetPoint(int rows, int cols, cv::Mat &in_img, cv::Mat &closing_, cv::Mat &line_image_, cv::Mat &line_image_x_, cv::Mat &skeleton_,
                                                                       int &thinning_size_, int &minLineLength_, int &maxLineGap_ , int &threshold_, int &iterations_)
{
        closing_ = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC1);
        skeleton_ = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC1);
        line_image_ = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC3);
        line_image_x = cv::Mat::zeros(in_img.rows, in_img.cols, CV_8UC3);
        // 将图像划分为3x3网格（9宫格）
        int grid_rows = rows;
        int grid_cols = cols;
        int img_height = in_img.rows;
        int img_width = in_img.cols;

        int cell_height = img_height / grid_rows;
        int cell_width = img_width / grid_cols;

        std::vector<std::pair<cv::Point2f, float>> all_results;
        std::vector<std::pair<cv::Point2f, float>> before_culster_results;
        int region_cnt_ = 1;
        // 遍历每个网格区域
        for (int row = 0; row < grid_rows; ++row) {
            for (int col = 0; col < grid_cols; ++col) {
                // 计算当前网格的边界
                int start_row = row * cell_height;
                int end_row = (row == grid_rows - 1) ? img_height : (row + 1) * cell_height;
                int start_col = col * cell_width;
                int end_col = (col == grid_cols - 1) ? img_width : (col + 1) * cell_width;
                
                // 提取当前网格区域
                cv::Rect roi(start_col, start_row, end_col - start_col, end_row - start_row);
                cv::Mat grid_region = in_img(roi);
                
                if (grid_region.empty() || cv::countNonZero(grid_region) == 0) {
                    continue; // 跳过空区域
                }
                
                // 对当前网格进行形态学处理
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(thinning_size_, thinning_size_));
                cv::Mat grid_closing;
                cv::morphologyEx(grid_region, grid_closing, cv::MORPH_OPEN, kernel, cv::Point(1,1), iterations_);
                
                // 细化处理
                cv::Mat grid_skeleton;
                cv::ximgproc::thinning(grid_closing, grid_skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
                
                // 用于区域遍历完显示
                grid_closing.copyTo(closing_(roi));
                grid_skeleton.copyTo(skeleton_(roi));
                
                // 直线检测
                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP(grid_skeleton, lines, 1, CV_PI/180, threshold_, minLineLength_, maxLineGap_);
                
                // 轴线角度过滤 
                if (!lines.empty()) {
                    std::vector<cv::Vec4i> filtered_lines;
                    for (const auto& line : lines) {
                        float dx = line[2] - line[0];
                        float dy = line[3] - line[1];
                        float angle = (dx == 0) ? 90.0f : std::abs(std::atan(dy / dx) * 180.0f / CV_PI);
                        float angle_diff = std::min(std::fmod(angle, 90), 90 - std::fmod(angle, 90));
                        
                        if (angle_diff <= 30) {
                            filtered_lines.push_back(line);
                        }
                    }
                    lines.clear();
                    lines = filtered_lines;
                }

                // 计算交点（需要将坐标转换回原图坐标系）
               
                printCurrentTime();
                printf(COLOR_CYAN "################### REGION %d #####################\n", region_cnt_);
                std::vector<std::pair<cv::Point2f, float>> grid_results =  calculateIntersections(lines, 150.0, 50.0);
                printf(COLOR_CYAN "###################################################\n");
                region_cnt_++;
                // 转换交点坐标到原图坐标系
                for (auto& result : grid_results) {
                    result.first.x += start_col;
                    result.first.y += start_row;
                }
                before_culster_results.insert(before_culster_results.end(), grid_results.begin(), grid_results.end());
                // 像素点聚类
                auto cluster_results = clusterPointsWithValues(grid_results, pixel_clsuter_, "avg");
                all_results.insert(all_results.end(), cluster_results.begin(), cluster_results.end());
                
                // 绘制检测到的直线（需要转换回原图坐标系）
                if (!lines.empty()) {
                    std::vector<cv::Vec4i> filtered_lines_x;
                    cv::Mat grid_line_image = cv::Mat::zeros(grid_skeleton.size(), CV_8UC3);
                    cv::Mat grid_line_image_x = cv::Mat::zeros(grid_skeleton.size(), CV_8UC3);
                    for (const auto& line : lines) {
                        float dx = line[2] - line[0];
                        float dy = line[3] - line[1];
                        if (fabs(dy) < fabs(dx)) 
                        {
                            filtered_lines_x.push_back(line);
                            cv::line(grid_line_image_x, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 255), 2);
                            cv::line(grid_line_image_x, cv::Point(line[0] + fabs(dx)/2 - 15, line[1]), cv::Point(line[0] + fabs(dx)/2 + 15, line[1]), cv::Scalar(0, 0, 255), 2.5);
                            cv::line(grid_line_image_x, cv::Point(line[0] + fabs(dx)/2, line[1] - 15), cv::Point(line[0] + fabs(dx)/2, line[1] + 15), cv::Scalar(0, 0, 255), 2.5);
                        }
                        cv::line(grid_line_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
                    }
                    grid_line_image.copyTo(line_image_(roi));
                    grid_line_image_x.copyTo(line_image_x_(roi));                
                }      
            }
        }
        auto cluster_results_ = snakeSortByColumn(all_results, 15.f);
        ROS_WARN("Before cluster pix_pts number: %d\n", (int)before_culster_results.size());
        ROS_WARN("After cluster pix_pts number: %d\n", (int)cluster_results_.size());
        // 使用所有网格的交点进行后续处理
        return cluster_results_;
}

// 交点计算实现
std::vector<std::pair<cv::Point2f, float>> ImageProcessor::calculateIntersections(const std::vector<cv::Vec4i> &lines, float dx_thr, float dy_thr) {
    std::vector<std::pair<cv::Point2f, float>> intersections;
    
    angle_view.clear();
    for (size_t i = 0; i < lines.size(); ++i) {
        for (size_t j = i + 1; j < lines.size(); ++j) {
            // l1
            cv::Point2f p1(lines[i][0], lines[i][1]); // (x1, y1)
            cv::Point2f p2(lines[i][2], lines[i][3]); // (x2, y2)
            // l2
            cv::Point2f p3(lines[j][0], lines[j][1]); // (x1,y1)
            cv::Point2f p4(lines[j][2], lines[j][3]); // (x2,y2)
            
            // 计算两条直线的夹角
            float angle = angleBetween(lines[i], lines[j]);
            if (angle >= 30 && angle <= 150) {
                // 判断x方向直线的斜率，根据电机零点位置做角度差补
                float dx1_abs = fabs(p2.x - p1.x);
                float dx2_abs = fabs(p4.x - p3.x);
                float dy1_abs = fabs(p2.y - p1.y);
                float dy2_abs = fabs(p4.y - p3.y);
                // 判断哪条直线趋于平行
                if (dy1_abs < dx1_abs || dy2_abs < dx2_abs)
                {
                    if (dy1_abs < dx1_abs) // l1是横向钢筋
                    {
                        float angle_offset_1 = std::abs(std::atan(dy1_abs / dx1_abs) * 180.0f / CV_PI);
                        // printCurrentTime();
                        // printf(COLOR_CYAN "1_line_x_direction: %f\n" COLOR_RESET, angle_offset_1);
                        if ((p2.y - p1.y) > 0 && (p2.x - p1.x) > 0) angle += angle_offset_1;
                        else if ((p2.y - p1.y) < 0 && (p2.x - p1.x) > 0) angle -= angle_offset_1;
                        else if ((p2.y - p1.y) < 0 && (p2.x - p1.x) < 0) angle += angle_offset_1;
                        else if ((p2.y - p1.y) > 0 && (p2.x - p1.x) < 0) angle -= angle_offset_1;
                    }
                    else if (dy2_abs < dx2_abs) // l2是横向钢筋
                    {
                        float angle_offset_2 = std::abs(std::atan(dy2_abs / dx2_abs) * 180.0f / CV_PI);
                        // printCurrentTime();
                        // printf(COLOR_CYAN "2_line_x_direction: %f\n" COLOR_RESET, angle_offset_2);
                        if ((p4.y - p3.y) > 0 && (p4.x - p3.x) > 0) angle += angle_offset_2;
                        else if ((p4.y - p3.y) < 0 && (p4.x - p3.x) > 0) angle -= angle_offset_2;
                        else if ((p4.y - p3.y) < 0 && (p4.x - p3.x) < 0) angle += angle_offset_2;
                        else if ((p4.y - p3.y) > 0 && (p4.x - p3.x) < 0) angle -= angle_offset_2;
                    }

                }

                // 计算交点
                float denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
                if (denom != 0) {
                    float px = ((p1.x*p2.y - p1.y*p2.x)*(p3.x - p4.x) - (p1.x - p2.x)*(p3.x*p4.y - p3.y*p4.x)) / denom;
                    float py = ((p1.x*p2.y - p1.y*p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x*p4.y - p3.y*p4.x)) / denom;
                    
                    // 检查交点是否在线段上
                    if (px >= std::min(p1.x, p2.x) && px <= std::max(p1.x, p2.x) &&
                        py >= std::min(p1.y, p2.y) && py <= std::max(p1.y, p2.y) &&
                        px >= std::min(p3.x, p4.x) && px <= std::max(p3.x, p4.x) &&
                        py >= std::min(p3.y, p4.y) && py <= std::max(p3.y, p4.y)) {
                        intersections.emplace_back(cv::Point2f(px, py), angle);
                    }
                }
            }
        }
    }
    
    return intersections;
}

std::vector<std::pair<cv::Point2f, float>> ImageProcessor::clusterPointsWithValues(
    const std::vector<std::pair<cv::Point2f, float>>& points,
    float distance_threshold,
    const std::string& float_agg_method) 
{
    const size_t n = points.size();
    if (n == 0) return {};

    // Step 1: 每个点分配一个 cluster label
    std::vector<int> labels(n, -1);
    int current_label = 0;

    for (size_t i = 0; i < n; ++i) {
        if (labels[i] != -1) continue; // 已经分簇

        labels[i] = current_label;

        for (size_t j = i + 1; j < n; ++j) {
            if (labels[j] != -1) continue;

            float dist = cv::norm(points[i].first - points[j].first);
            if (dist < distance_threshold) {
                labels[j] = current_label;
            }
        }

        current_label++;
    }

    // Step 2: 对每个簇，计算中心点 + float 值的聚合值
    struct ClusterData {
        cv::Point2f sum{0, 0};
        float float_sum = 0.0f;
        int count = 0;
        std::vector<float> float_values; // 可选，如果需要更复杂的聚合
    };

    std::vector<ClusterData> clusters(current_label);

    for (size_t i = 0; i < n; ++i) {
        int label = labels[i];
        const auto& p = points[i];

        clusters[label].sum += p.first;
        clusters[label].float_sum += p.second;
        clusters[label].count++;
        clusters[label].float_values.push_back(p.second); // 可选
    }

    // Step 3: 构造输出结果
    std::vector<std::pair<cv::Point2f, float>> result;

    for (int i = 0; i < current_label; ++i) {
        if (clusters[i].count == 0) continue;

        cv::Point2f center = clusters[i].sum / clusters[i].count;

        float agg_float_value = 0.0f;

        if (float_agg_method == "avg") {
            agg_float_value = clusters[i].float_sum / clusters[i].count;
        }
        else if (float_agg_method == "first") {
            agg_float_value = clusters[i].float_values.front(); // 第一个
        }
        else if (float_agg_method == "max") {
            agg_float_value = *std::max_element(clusters[i].float_values.begin(), clusters[i].float_values.end());
        }
        else if (float_agg_method == "min") {
            agg_float_value = *std::min_element(clusters[i].float_values.begin(), clusters[i].float_values.end());
        }
        else {
            // 默认使用平均值
            agg_float_value = clusters[i].float_sum / clusters[i].count;
        }

        result.emplace_back(center, agg_float_value);
    }

    return result;
}

 std::vector<std::pair<cv::Point2f, float>> ImageProcessor::snakeSortByColumn(
        const std::vector<std::pair<cv::Point2f, float>>& points,
        float row_threshold) {
        if (points.empty()) {
            return {};
        }

        // 1. 按 y 坐标排序（上下排序，便于分组为行）
        std::vector<std::pair<cv::Point2f, float>> sorted_by_y = points;
        std::sort(sorted_by_y.begin(), sorted_by_y.end(),
            [](const std::pair<cv::Point2f, float>& a, const std::pair<cv::Point2f, float>& b) {
                return a.first.y < b.first.y;
            });

        // 2. 按 y 分组为多个 "行"（group by y，相近的为一行）
        std::vector<std::vector<std::pair<cv::Point2f, float>>> rows;
        std::vector<std::pair<cv::Point2f, float>> current_row = {sorted_by_y[0]};
        float last_y = sorted_by_y[0].first.y;

        for (size_t i = 1; i < sorted_by_y.size(); ++i) {
            if (std::abs(sorted_by_y[i].first.y - last_y) <= row_threshold) {
                current_row.push_back(sorted_by_y[i]);
            } else {
                rows.push_back(current_row);
                current_row = {sorted_by_y[i]};
                last_y = sorted_by_y[i].first.y;
            }
        }

        if (!current_row.empty()) {
            rows.push_back(current_row);
        }

        // 3. 对每一行内部按 x 坐标排序（从左到右）
        // 4. 对奇数行做 reverse（蛇形排序：奇数行从右到左）
        std::vector<std::pair<cv::Point2f, float>> result;
        for (size_t i = 0; i < rows.size(); ++i) {
            auto& row = rows[i];

            // 按 x 坐标排序
            std::sort(row.begin(), row.end(),
                [](const std::pair<cv::Point2f, float>& a, const std::pair<cv::Point2f, float>& b) {
                    return a.first.x < b.first.x;
                });

            // 奇数行：反转（蛇形：从右到左）
            if (i % 2 == 1) {
                std::reverse(row.begin(), row.end());
            }

            // 添加到结果
            result.insert(result.end(), row.begin(), row.end());
        }

        return result;
}

std::vector<cv::Point3f> ImageProcessor::snakeSort(const std::vector<cv::Point3f>& centers, float row_threshold) {
    if (centers.empty()) {
        return centers;
    }
    
    // 按y坐标排序
    std::vector<cv::Point3f> sorted_centers = centers;
    std::sort(sorted_centers.begin(), sorted_centers.end(), 
             [](const cv::Point3f& a, const cv::Point3f& b) { return a.y < b.y; });
    
    // 分组为行
    std::vector<std::vector<cv::Point3f>> rows;
    std::vector<cv::Point3f> current_row = {sorted_centers[0]};
    float last_y = sorted_centers[0].y;
    
    for (size_t i = 1; i < sorted_centers.size(); ++i) {
        if (std::abs(sorted_centers[i].y - last_y) <= row_threshold) {
            current_row.push_back(sorted_centers[i]);
        } else {
            rows.push_back(current_row);
            current_row = {sorted_centers[i]};
            last_y = sorted_centers[i].y;
        }
    }
    
    if (!current_row.empty()) {
        rows.push_back(current_row);
    }
    
    // 蛇形排序
    std::vector<cv::Point3f> result;
    for (size_t i = 0; i < rows.size(); ++i) {
        auto& row = rows[i];
        std::sort(row.begin(), row.end(), 
                 [](const cv::Point3f& a, const cv::Point3f& b) { return a.x < b.x; });
        
        if (i % 2 == 1) {
            std::reverse(row.begin(), row.end());
        }
        
        result.insert(result.end(), row.begin(), row.end());
    }
    
    return result;
}

void ImageProcessor::drawTextWithBackground(cv::Mat& image, const std::string& text, 
                                          const cv::Point& position, int font, 
                                          double font_scale, const cv::Scalar& text_color,
                                          const cv::Scalar& bg_color, int thickness) {
    cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, nullptr);
    cv::rectangle(image, 
                 cv::Point(position.x, position.y - text_size.height),
                 cv::Point(position.x + text_size.width, position.y),
                 bg_color, cv::FILLED);
    cv::putText(image, text, position, font, font_scale, text_color, thickness, cv::LINE_AA);
}

// 判断水管(yolo推理 -> 可以改为单独写一个python服务，方便使用yolo)
bool ImageProcessor::saveImage(int x, int y, int z_value) {
    // 计算ROI边界
    int x_start = std::max(0, x - half_size_);
    int x_end = std::min(image_infrared_.cols, x + half_size_);
    int y_start = std::max(0, y - half_size_);
    int y_end = std::min(image_infrared_.rows, y + half_size_);
    
    // 提取ROI
    cv::Mat roi_depth = Depth_image_Raw_(cv::Range(y_start, y_end), cv::Range(x_start, x_end));
    cv::Mat roi_infrared = image_infrared_(cv::Range(y_start, y_end), cv::Range(x_start, x_end));
    
    // 创建掩码
    cv::Mat mask = (roi_depth >= (z_value - max_depth_offset_)) & 
                  (roi_depth <= (z_value + max_depth_offset_));
    
    // 应用掩码
    cv::Mat sub_image = cv::Mat::zeros(roi_infrared.size(), roi_infrared.type());
    roi_infrared.copyTo(sub_image, mask);
    
    // 调整大小
    cv::Mat resized_image;
    cv::resize(sub_image, resized_image, cv::Size(128, 128));
    
    // 归一化
    cv::Mat normalized_image;
    cv::normalize(resized_image, normalized_image, 0, 255, cv::NORM_MINMAX);
    normalized_image.convertTo(normalized_image, CV_8U);
    
    // CLAHE处理
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(normalized_image, normalized_image);
    
    // 这里应该有YOLO推理，但为了简化示例我们省略
    // 假设我们随机返回true或false
    return (rand() % 2) == 0;
}

Eigen::Matrix4f ImageProcessor::createRotationMatrix(float theta) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<2, 2>(0, 0) = Eigen::Rotation2Df(theta).toRotationMatrix();
    return matrix;
}

std::pair<Eigen::Vector3f, Eigen::Matrix3f> ImageProcessor::transformToEndEffector(float x_obj, float y_obj, float z_obj, float theta_obj) {
    // 创建物体在相机坐标系中的变换矩阵
    Eigen::Matrix4f T_camera_obj = Eigen::Matrix4f::Identity();
    T_camera_obj.block<3, 3>(0, 0) = createRotationMatrix(theta_obj).block<3, 3>(0, 0);
    T_camera_obj(0, 3) = x_obj;
    T_camera_obj(1, 3) = y_obj;
    T_camera_obj(2, 3) = z_obj;
    
    // 计算物体在末端执行器坐标系中的位姿
    Eigen::Matrix4f T_ee_obj = T_camera_ee * T_camera_obj;
    
    return {
        T_ee_obj.block<3, 1>(0, 3),
        T_ee_obj.block<3, 3>(0, 0)
    };
}

// camera2gripper
Eigen::Affine3f ImageProcessor::getScepterGripper(Eigen::Matrix4f & pose_1)
{
    Eigen::Matrix4f gripper2aruco = Eigen::Matrix4f::Identity();
    // 1. 创建两个旋转矩阵
    Eigen::Matrix3f rotX = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();   // 绕 X 轴转 180°
    Eigen::Matrix3f rotZ = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).toRotationMatrix(); // 绕 Z 轴转 -90°

    // 2. 将两个旋转组合（先绕 X，再绕 Z）→ 顺序：rotZ * rotX
    Eigen::Matrix3f combined_rotation = rotZ * rotX;

    // 3. 将组合后的旋转矩阵赋值给 gripper2aruco 的旋转部分
    gripper2aruco.block<3,3>(0,0) = combined_rotation;
    gripper2aruco.block<3,1>(0,3) = Eigen::Vector3f(offset_x_ / 1000, offset_y_ / 1000, offset_z_/ 1000); // m
    Eigen::Matrix4f T = gripper2aruco.inverse() * pose_1.inverse();
    return Eigen::Affine3f(T);

}
Eigen::Vector3f ImageProcessor::transformToGripperFrame(float x, float y, float z, int idx) {
    try {
        
        // 转换点坐标至gripper系
        Eigen::Vector3f point_camera(x / 1000, y / 1000 , z / 1000); // mm ->m
        Eigen::Vector3f point_gripper = T_scepter_gripper * point_camera; // m

        point_gripper[0] = point_gripper[0] * 1000 + offset_x_; // (偏心补偿)-37.2529; // m -> mm
        point_gripper[1] = point_gripper[1] * 1000 + offset_y_; // （偏心补偿）-16.715; 
        point_gripper[2] = point_gripper[2] * 1000 + offset_z_;
        return point_gripper;
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform point: %s", ex.what());
        return Eigen::Vector3f(x, y, z);
    }
}

Eigen::Vector3f ImageProcessor::getGripperRelativeTranslation() {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "gripper_frame", "Scepter_depth_frame", ros::Time(0), ros::Duration(1.0));
        return Eigen::Vector3f(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("Failed to get transform: %s", ex.what());
        return Eigen::Vector3f::Zero();
    }
}

// void ImageProcessor::callLinearModuleMoveService(int pos_x, int pos_y, int pos_z) {
//     ros::ServiceClient client = nh_.serviceClient<chassis_ctrl::linear_module_move>("linear_module_move");
//     chassis_ctrl::linear_module_move srv;
//     srv.request.pos_x = pos_x;
//     srv.request.pos_y = pos_y;
//     srv.request.pos_z = pos_z;
    
//     if (client.call(srv)) {
//         ROS_INFO("Service call succeeded: %s", srv.response.message.c_str());
//     } else {
//         ROS_ERROR("Failed to call service linear_module_move");
//     }
// }

// // aruco码自动标定
// bool ImageProcessor::detectAndSavePose(std_srvs::Trigger::Request& req,
//                                      std_srvs::Trigger::Response& res) {
//     callLinearModuleMoveService(displacement_x_, displacement_y_, 0.);
//     ros::Duration(2).sleep();
    
//     bool detected = false;
//     bool saved = false;
//     cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//     cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    
//     ros::Time start_time = ros::Time::now();
//     while ((ros::Time::now() - start_time).toSec() < 5) {
//         if (!image_color_copy_.empty() && !camera_matrix_.empty() && !dist_coeffs_.empty()) {
//             std::vector<int> ids;
//             std::vector<std::vector<cv::Point2f>> corners;
//             cv::aruco::detectMarkers(image_color_copy_, aruco_dict, corners, ids, parameters);
            
//             if (!ids.empty()) {
//                 detected = true;
//                 std::vector<cv::Vec3d> rvecs, tvecs;
//                 cv::aruco::estimatePoseSingleMarkers(corners, 0.06, camera_matrix_, dist_coeffs_, rvecs, tvecs);
                
//                 cv::Mat rotation_matrix;
//                 cv::Rodrigues(rvecs[0], rotation_matrix);
                
                
//                 for (int i = 0; i < 3; ++i) {
//                     for (int j = 0; j < 3; ++j) {
//                         pose_matrix(i, j) = rotation_matrix.at<double>(i, j);
//                     }
//                     pose_matrix(i, 3) = tvecs[0][i];
//                 }
                
//                 // 调整位置偏移
//                 pose_matrix(0, 3) -= displacement_y_ / 1000.0f;
//                 pose_matrix(1, 3) -= displacement_x_ / 1000.0f;
//                 pose_matrix(2, 3) -= displacement_z_ / 1000.0f;
                
//                 try {
                    
//                     // 使用Eigen的Map来直接保存矩阵数据
//                     // Eigen::Matrix<float, 4, 4, Eigen::RowMajor> pose_matrix_row_major = pose_matrix;
//                     // std::ofstream file(pose_matrix_path, std::ios::binary);
//                     // if (file.is_open()) {
//                     //     file.write(reinterpret_cast<const char*>(pose_matrix_row_major.data()), 
//                     //               sizeof(float) * 16);
//                     //     file.close();
//                     //     saved = true;
//                     //     break;
//                     // }
//                     if (saveMatrixToTxt(pose_matrix,pose_matrix_path)){
//                         ROS_INFO("Matrix saved successfully");
//                         saved = true;
//                         break;
//                     } else {
//                         ROS_ERROR("Failed to save matrix");
//                     }
//                 } catch (...) {
//                     saved = false;
//                 }
//             }
//         }
//         ros::Duration(0.1).sleep();
//     }
    
//     callLinearModuleMoveService(0, 0, 0);
//     res.success = saved;
//     res.message = "Detected: " + std::to_string(detected) + ", Saved: " + std::to_string(saved);
//     return true;
// }
