#include "tie_robot_perception/camera/scepter_manager.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
//define call back function
void ScepterManager::publishCloudPoint(const ros::Time& time, const ScFrame& srcFrame, ros::Publisher& pub, sensor_msgs::CameraInfoPtr& cameraInfoPtr, ScFrame* frameArr)
{
    ScFrameType type = srcFrame.frameType;
    if (!(type == SC_DEPTH_FRAME || type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME))
    {
        ROS_INFO_STREAM( "invaid frame type : " << type );
        return;
    }
    const int len = srcFrame.width * srcFrame.height;
    ScVector3f* worldV = new ScVector3f[len];
    scConvertDepthFrameToPointCloudVector(deviceHandle_, &srcFrame, worldV);

    sensor_msgs::PointCloud2 output_msg;
    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(len);
    output_msg.header.frame_id = this->camera_name_ + "_points_frame";
    if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        output_msg.header.frame_id = this->camera_name_ + "_depth2colorpoints_frame";
    }
    output_msg.header.stamp = time;

    sensor_msgs::PointCloud2Iterator<float> iter_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output_msg, "b");

    if (type == SC_DEPTH_FRAME)
    {
        cv::Mat mat = cv::Mat(frameArr->height, frameArr->width, CV_8UC1, frameArr->pFrameData);
        for (int i = 0; i < len; i++)
        {
            int row = (i / mat.cols);
            int col = (i % mat.cols);
            const uchar gray_value = mat.at<uchar>(row, col);
            if (0 != worldV[i].z && worldV[i].z != 65535)
            {
                *iter_x = worldV[i].x / 1000.0f;
                *iter_y = worldV[i].y / 1000.0f;
                *iter_z = worldV[i].z / 1000.0f;
                *iter_r = gray_value;
                *iter_g = gray_value;
                *iter_b = gray_value;
            }
            else
            {
                *iter_x = 0.0f;
                *iter_y = 0.0f;
                *iter_z = 0.0f;
                *iter_r = 0;
                *iter_g = 0;
                *iter_b = 0;
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
    }
    else if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        cv::Mat mat = cv::Mat((*(frameArr + 2)).height, (*(frameArr + 2)).width, CV_8UC3, (*(frameArr + 2)).pFrameData);
        for (int i = 0; i < len; i++)
        {
            int row = (i / mat.cols);
            int col = (i % mat.cols);
            uchar* data = mat.ptr<uchar>(row);
            if (0 != worldV[i].z && worldV[i].z != 65535)
            {
                *iter_x = worldV[i].x / 1000.0f;
                *iter_y = worldV[i].y / 1000.0f;
                *iter_z = worldV[i].z / 1000.0f;
                *iter_r = data[3 * col + 2];
                *iter_g = data[3 * col + 1];
                *iter_b = data[3 * col];
            }
            else
            {
                *iter_x = 0.0f;
                *iter_y = 0.0f;
                *iter_z = 0.0f;
                *iter_r = 0;
                *iter_g = 0;
                *iter_b = 0;
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
    }

    cameraInfoPtr->height = srcFrame.height;
    cameraInfoPtr->width = srcFrame.width;
    cameraInfoPtr->header.stamp = time;
    pub.publish(output_msg);

    delete [] worldV;

    if (type == SC_DEPTH_FRAME)
    {
        this->depthCloudPointCameraInfoPub_.publish(cameraInfoPtr);
    }
    else if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        this->depth2colorCloudPointCameraPub_.publish(cameraInfoPtr);
    }
}


bool ScepterManager::fillImagePtr(const ros::Time& time, const ScFrameType type, ScFrame& frame,sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr)
{
    bool ret = false;

    if (frame.pFrameData != NULL)
    {
        int cvMatType = CV_16UC1;
        std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
        switch (type)
        {
        case SC_IR_FRAME:
            cvMatType = CV_8UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
            break;
        case SC_DEPTH_FRAME:
        case SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME:
            cvMatType = CV_16UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case SC_COLOR_FRAME:
 	case SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME:
            cvMatType = CV_8UC3;
            imageEncodeType = sensor_msgs::image_encodings::BGR8;
            break;
        default:
            return ret;
        }

        cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
        cameraInfoPtr->height = frame.height;
        cameraInfoPtr->width = frame.width;
        cameraInfoPtr->header.stamp = time;
        imagePtr = cv_bridge::CvImage(cameraInfoPtr->header, imageEncodeType, mat).toImageMsg();
        ret = true;
    }

    return ret;
}

