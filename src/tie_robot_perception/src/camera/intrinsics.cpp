#include "tie_robot_perception/camera/scepter_manager.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
//define call back function
void ScepterManager::set_sensor_intrinsics()
{
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame"),
                points_frame(this->camera_name_ + "_points_frame"),
                depth2colorpoints_frame(this->camera_name_ + "_depth2colorpoints_frame");

    // Get camera parameters (extrinsic)
    checkScStatus(scGetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_),
                      "Could not get extrinsics!");

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Time now = ros::Time::now();

    // PsCameraExtrinsicParameters to ROS transform
    tf::Transform transform;
    tf::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Publish static TFs
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = now;
    msg.transform.rotation.w = 1.0;

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = depth2colorpoints_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster.sendTransform(msg);
    
    msg.header.frame_id = depth_frame;
    msg.child_frame_id = alignedcolor_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = orientation;
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_TOF_SENSOR, &this->depth_intrinsics_),
                      "Could not get depth intrinsics!");
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_),
                      "Could not get color intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2color_point_cloud_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth_frame;
    info_msg.D = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.K = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_->setCameraInfo(info_msg);
    info_msg.header.frame_id = ir_frame;
    ir_info_->setCameraInfo(info_msg);
    alignedColor_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = points_frame;
    depth_point_cloud_info_->setCameraInfo(info_msg);

    ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index_);

    checkScStatus(scSetTransformColorImgToDepthSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformColorImgToDepthSensorEnabled!");
    checkScStatus(scSetTransformDepthImgToColorSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformDepthImgToColorSensorEnabled!");
}


void ScepterManager::updateColorIntrinsicParameters()
{
    std::string color_frame(this->camera_name_ + "_color_frame"),
                depth2colorpoints_frame(this->camera_name_ + "_depth2colorpoints_frame");
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_),
                    "Could not get color intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2color_point_cloud_info_->setCameraInfo(info_msg);

    cameraInfo_Ary[2].reset();
    cameraInfo_Ary[2] = boost::make_shared<sensor_msgs::CameraInfo>(color_info_->getCameraInfo());
    cameraInfo_Ary[3].reset();
    cameraInfo_Ary[3] = boost::make_shared<sensor_msgs::CameraInfo>(alignedDepth_info_->getCameraInfo());
    cameraInfo_Ary[6].reset();
    cameraInfo_Ary[6] = boost::make_shared<sensor_msgs::CameraInfo>(depth2color_point_cloud_info_->getCameraInfo());
}
