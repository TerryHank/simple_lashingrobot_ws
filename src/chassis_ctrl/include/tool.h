// tool.h
#ifndef TOOL_H
#define TOOL_H
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <imuTool.h>

// 角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 使用ZYX顺序，imu左手系，roll取反
void calculateCorrectedTarget(
    double pitch1, double roll1, double yaw1,    // 识别时姿态
    double x1, double y1, double z1,             // 识别时坐标
    double pitch2, double roll2, double yaw2,    // 当前姿态
    double &x_out, double &y_out, double &z_out, // 输出坐标
    bool degrees = true)
{
    // 角度转弧度
    pitch1 = deg2rad(pitch1);
    roll1 = deg2rad(roll1);
    yaw1 = deg2rad(yaw1);

    pitch2 = deg2rad(pitch2);
    roll2 = deg2rad(roll2);
    yaw2 = deg2rad(yaw2);

    // roll取负值
    roll1 = -roll1;
    roll2 = -roll2;

    // 创建旋转矩阵 R1
    Eigen::Matrix3d R1;
    R1(0, 0) = cos(yaw1) * cos(roll1);
    R1(0, 1) = cos(yaw1) * sin(roll1) * sin(pitch1) - sin(yaw1) * cos(pitch1);
    R1(0, 2) = cos(yaw1) * sin(roll1) * cos(pitch1) + sin(yaw1) * sin(pitch1);

    R1(1, 0) = sin(yaw1) * cos(roll1);
    R1(1, 1) = sin(yaw1) * sin(roll1) * sin(pitch1) + cos(yaw1) * cos(pitch1);
    R1(1, 2) = sin(yaw1) * sin(roll1) * cos(pitch1) - cos(yaw1) * sin(pitch1);

    R1(2, 0) = -sin(roll1);
    R1(2, 1) = cos(roll1) * sin(pitch1);
    R1(2, 2) = cos(roll1) * cos(pitch1);

    // 创建旋转矩阵 R2
    Eigen::Matrix3d R2;
    R2(0, 0) = cos(yaw2) * cos(roll2);
    R2(0, 1) = cos(yaw2) * sin(roll2) * sin(pitch2) - sin(yaw2) * cos(pitch2);
    R2(0, 2) = cos(yaw2) * sin(roll2) * cos(pitch2) + sin(yaw2) * sin(pitch2);

    R2(1, 0) = sin(yaw2) * cos(roll2);
    R2(1, 1) = sin(yaw2) * sin(roll2) * sin(pitch2) + cos(yaw2) * cos(pitch2);
    R2(1, 2) = sin(yaw2) * sin(roll2) * cos(pitch2) - cos(yaw2) * sin(pitch2);

    R2(2, 0) = -sin(roll2);
    R2(2, 1) = cos(roll2) * sin(pitch2);
    R2(2, 2) = cos(roll2) * cos(pitch2);
    // 补偿数据
    double tx = -14;
    double ty = 25;
    double tz = 338;

    Eigen::Vector3d target_point = Eigen::Vector3d(x1 + tx, y1 + ty, z1 + tz);
    // 计算校正位置
    Eigen::Vector3d target_in_camera = R1 * target_point;
    Eigen::Vector3d corrected_pos = R2.transpose() * target_in_camera;
    // 输出结果
    double d = 0.8;
    x_out = (corrected_pos(0) - tx - x1)*d + x1;
    y_out = (corrected_pos(1) - ty - y1)*d + y1;
    z_out = (corrected_pos(2) - tz - z1)*d + z1;
}

#endif // TOOL_H
