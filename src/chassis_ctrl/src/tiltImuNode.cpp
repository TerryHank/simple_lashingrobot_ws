#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "imuTool.h"

namespace
{
geometry_msgs::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiltImuNode");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string serial_port;
    std::string frame_id;
    std::string topic_name;
    double publish_rate_hz;
    double reader_period_ms;
    double yaw_deg;
    bool invert_roll;

    pnh.param("port", serial_port, std::string("/dev/ttyUSB0"));
    pnh.param("frame_id", frame_id, std::string("imu_link"));
    pnh.param("topic_name", topic_name, std::string("/imu/data"));
    pnh.param("publish_rate_hz", publish_rate_hz, 50.0);
    pnh.param("reader_period_ms", reader_period_ms, 40.0);
    pnh.param("yaw_deg", yaw_deg, 0.0);
    pnh.param("invert_roll", invert_roll, true);

    SensorReader reader;
    if (!reader.init(reader_period_ms, serial_port))
    {
        ROS_ERROR_STREAM("Failed to open tilt IMU on " << serial_port);
        return 1;
    }

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(topic_name, 20);
    ros::Rate loop_rate(std::max(1.0, publish_rate_hz));

    ROS_INFO_STREAM("Publishing tilt IMU on " << topic_name
                    << " from " << serial_port
                    << " with frame_id=" << frame_id);

    const double yaw_rad = yaw_deg * M_PI / 180.0;
    bool has_last_sample = false;
    double last_roll = 0.0;
    double last_pitch = 0.0;
    ros::Time last_stamp;

    ros::Duration(0.2).sleep();

    while (ros::ok())
    {
        double roll_deg_raw = 0.0;
        double pitch_deg = 0.0;
        reader.getDataBufferAvg(roll_deg_raw, pitch_deg);

        const double roll_deg = invert_roll ? -roll_deg_raw : roll_deg_raw;
        const double roll = roll_deg * M_PI / 180.0;
        const double pitch = pitch_deg * M_PI / 180.0;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id;
        imu_msg.orientation = quaternionFromRPY(roll, pitch, yaw_rad);

        imu_msg.orientation_covariance[0] = 0.02;
        imu_msg.orientation_covariance[4] = 0.02;
        imu_msg.orientation_covariance[8] = 1e6; // Yaw is unknown on this tilt sensor.

        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        imu_msg.angular_velocity_covariance[0] = 0.1;
        imu_msg.angular_velocity_covariance[4] = 0.1;
        imu_msg.angular_velocity_covariance[8] = 1e6;

        if (has_last_sample)
        {
            const double dt = (imu_msg.header.stamp - last_stamp).toSec();
            if (dt > 1e-4)
            {
                imu_msg.angular_velocity.x = (roll - last_roll) / dt;
                imu_msg.angular_velocity.y = (pitch - last_pitch) / dt;
            }
        }

        imu_msg.linear_acceleration_covariance[0] = -1.0;

        imu_pub.publish(imu_msg);

        last_roll = roll;
        last_pitch = pitch;
        last_stamp = imu_msg.header.stamp;
        has_last_sample = true;

        ros::spinOnce();
        loop_rate.sleep();
    }

    reader.close();
    return 0;
}
