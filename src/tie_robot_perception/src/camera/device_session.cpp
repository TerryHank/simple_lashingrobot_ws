#include "tie_robot_perception/camera/scepter_manager.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {

constexpr int kMaxConsecutiveFrameReadyFailures = 30;

}  // namespace

//define call back function
void ScepterManager::paramCallback(scepter_param::Sceptertof_roscppConfig& config,uint32_t level)
{           

    // ROS_INFO("Request: %d %d %d %d %d %d %d %d %d %d %d %d",
    //             config.FrameRate,
    //             config.IRGMMGain,
    //             config.ColorResloution,
    //             config.FlyingPixelenable,
    //             config.FlyingPixelvalue,
    //             config.Confidenceenable,
    //             config.Confidencevalue,
    //             config.TimeFilterenable,
    //             config.TimeFiltervalue,
    //             config.IRGMMCorrectionenable,
    //             config.IRGMMCorrectionvalue,
    //             config.ToFManual,
    //             config.ToFExposureTime,
    //             config.ColorManual,
    //             config.ColorExposureTime,
    //             config.WorkMode,
    //             config.SoftwareTrigger,
    //             config.XDRMode,
    //             config.DepthCloudPoint,
    //             config.Depth2ColorCloudPoint);

    if(config_.XDRMode != config.XDRMode)
    {
        config_.XDRMode = config.XDRMode;
        ScStatus status = ScStatus::SC_OK;
        switch (config_.XDRMode)
        {
        case 0:
            hdrEnabled = false;
            wdrEnabled = false;
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            break;
        case 1:
            hdrEnabled = true;
            wdrEnabled = false;
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            break;
        case 2:
            hdrEnabled = false;
            wdrEnabled = true;
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            break;
        }
    }
    if(config_.IRGMMGain != config.IRGMMGain)
    {
        config_.IRGMMGain = config.IRGMMGain;
        ScStatus status= scSetIRGMMGain(deviceHandle_,config_.IRGMMGain);
        ROS_INFO_STREAM( "SetIRGMMGain status: " << status);
    }
    if( FlyingPixelFilterParams.threshold !=config.FlyingPixelvalue)
    {

        FlyingPixelFilterParams.enable = config.FlyingPixelenable;
        FlyingPixelFilterParams.threshold = config.FlyingPixelvalue;
        ScStatus status= scSetFlyingPixelFilterParams(deviceHandle_, FlyingPixelFilterParams);
        ROS_INFO_STREAM( "SetFlyingPixelFilter status: " << status);
    }
    if( ConfidenceFilterParams.threshold != config.Confidencevalue)
    {
       
        ConfidenceFilterParams.enable = config.Confidenceenable;
        ConfidenceFilterParams.threshold = config.Confidencevalue;
        ScStatus status = scSetConfidenceFilterParams(deviceHandle_, ConfidenceFilterParams);
        ROS_INFO_STREAM("SetConfidenceFilter status: " << status);
    }
    if( TimeFilterParams.threshold != config.TimeFiltervalue)
    {
        
        TimeFilterParams.enable = config.TimeFilterenable;
        TimeFilterParams.threshold = config.TimeFiltervalue;
        ScStatus status = scSetTimeFilterParams(deviceHandle_, TimeFilterParams);
        ROS_INFO_STREAM("SetTimeFilterParams status: " << status);
    }

    if(IRGMMCorrectionParams.threshold != config.IRGMMCorrectionvalue)
    {
        
        IRGMMCorrectionParams.enable = config.IRGMMCorrectionenable;
        IRGMMCorrectionParams.threshold = config.IRGMMCorrectionvalue;
        ScStatus status = scSetIRGMMCorrection(deviceHandle_, IRGMMCorrectionParams);
        ROS_INFO_STREAM("SetIRGMMCorrection status: " << status);
    }

    if(config_.SpatialFilterEnabled != config.SpatialFilterEnabled)
    {
        config_.SpatialFilterEnabled = config.SpatialFilterEnabled;
        ScStatus status = scSetSpatialFilterEnabled(deviceHandle_, config_.SpatialFilterEnabled);
        ROS_INFO_STREAM("SetSpatialFilterEnabled status: " << status);
    }

    if(config_.FillHoleFilterEnabled != config.FillHoleFilterEnabled)
    {
        config_.FillHoleFilterEnabled = config.FillHoleFilterEnabled;
        ScStatus status = scSetFillHoleFilterEnabled(deviceHandle_, config_.FillHoleFilterEnabled);
        ROS_INFO_STREAM("SetFillHoleFilterEnabled status: " << status);
    }

    if(config_.FrameRate != config.FrameRate)
    {
        config_.FrameRate = config.FrameRate;
        ScStatus status= scSetFrameRate(deviceHandle_,config_.FrameRate);
        ROS_INFO_STREAM( "SetFrameRate status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "FrameRate number invaild with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "FrameRate number invaild with WDR enabled");
            }
        }
    }
    if(config_.ColorResloution != config.ColorResloution)
    {
        config_.ColorResloution = config.ColorResloution;
        int w = 640;
        int h = 480;
        switch (config_.ColorResloution)
        {
        case 0:
            w = 1600;
            h = 1200;
            break;
        case 1:
            w = 800;
            h = 600;
            break;
        case 2:
            w = 640;
            h = 480;
            break;
        }
        ScStatus status= scSetColorResolution(deviceHandle_,w,h);
        ROS_INFO_STREAM( "SetColorResolution status: " << status);
        if(status == SC_OK)
        {
            ROS_INFO_STREAM( "updateColorIntrinsicParameters as color resloution changed");
            updateColorIntrinsicParameters();
        }
    }
    
    if(config_.ToFManual != config.ToFManual)
    {
        config_.ToFManual = config.ToFManual;
        ScStatus status= scSetExposureControlMode(deviceHandle_,SC_TOF_SENSOR,(ScExposureControlMode)config_.ToFManual);
        ROS_INFO_STREAM( "SetExposureControlMode tof status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureControlMode with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureControlMode with WDR enabled");
            }
        }
        if(config_.ToFManual != 1)
        {
          config_.ToFExposureTime = 0;  
        }
    }
    if(config_.ToFExposureTime != config.ToFExposureTime&&config_.ToFManual==1)
    {
        config_.ToFExposureTime = config.ToFExposureTime;
        int exposureTime = 0;
		int retry = 0;
		while (0 == exposureTime && retry < 3)
        {
			scGetMaxExposureTime(deviceHandle_, SC_TOF_SENSOR, &exposureTime);
            retry++;
        }
        if (0 != exposureTime && config_.ToFExposureTime <= exposureTime)
        {
			exposureTime = config_.ToFExposureTime;
		}
        else
        {
            ROS_INFO_STREAM( "SetExposureTime tof Max Value: " << exposureTime);
        }
        ScStatus status= scSetExposureTime(deviceHandle_,SC_TOF_SENSOR,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime tof status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureTime with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureTime with WDR enabled");
            }
        }
    }
    if(config_.ColorManual != config.ColorManual)
    {
        config_.ColorManual = config.ColorManual;
        ScStatus status= scSetExposureControlMode(deviceHandle_,SC_COLOR_SENSOR,(ScExposureControlMode)config_.ColorManual);
        ROS_INFO_STREAM( "SetExposureControlMode color status: " << status);
        if(config_.ColorManual != 1)
        {
          config_.ColorExposureTime = 0;  
        }
    }
    if(config_.ColorExposureTime != config.ColorExposureTime&&config_.ColorManual==1)
    {
		config_.ColorExposureTime = config.ColorExposureTime;
        int exposureTime = 0;
		int retry = 0;
		while (0 == exposureTime && retry < 3)
        {
			scGetMaxExposureTime(deviceHandle_, SC_COLOR_SENSOR, &exposureTime);
            retry++;
        }
        if (0 != exposureTime && config_.ColorExposureTime <= exposureTime)
        {
			exposureTime = config_.ColorExposureTime;
		}
        else
        {
            ROS_INFO_STREAM( "SetExposureTime color Max Value: " << exposureTime);
        }
        ScStatus status= scSetExposureTime(deviceHandle_,SC_COLOR_SENSOR,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime color status: " << status);
    }
    if(config_.WorkMode != config.WorkMode)
    {
        config_.WorkMode = config.WorkMode;
        ScStatus status= scSetWorkMode(deviceHandle_,(ScWorkMode)config_.WorkMode);
        ROS_INFO_STREAM( "SetWorkMode status: " << status);
    }

    config_.SoftwareTrigger = config.SoftwareTrigger;

    if(config_.DepthCloudPoint != config.DepthCloudPoint)
    {   
        config_.DepthCloudPoint = config.DepthCloudPoint;
    }
    if(config_.Depth2ColorCloudPoint != config.Depth2ColorCloudPoint)
    {   
        config_.Depth2ColorCloudPoint = config.Depth2ColorCloudPoint;
    }
}


void ScepterManager::run() 
{
    // Initialise ROS nodes
    set_sensor_intrinsics();

    cameraInfo_Ary[0] = boost::make_shared<sensor_msgs::CameraInfo>(depth_info_->getCameraInfo());
    cameraInfo_Ary[1] = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_->getCameraInfo());
    cameraInfo_Ary[2] = boost::make_shared<sensor_msgs::CameraInfo>(color_info_->getCameraInfo());
    cameraInfo_Ary[3] = boost::make_shared<sensor_msgs::CameraInfo>(alignedDepth_info_->getCameraInfo());
    cameraInfo_Ary[4] = boost::make_shared<sensor_msgs::CameraInfo>(alignedColor_info_->getCameraInfo());
    cameraInfo_Ary[5] = boost::make_shared<sensor_msgs::CameraInfo>(depth_point_cloud_info_->getCameraInfo());
    cameraInfo_Ary[6] = boost::make_shared<sensor_msgs::CameraInfo>(depth2color_point_cloud_info_->getCameraInfo());

    // CameraPublisher 	advertiseCamera (const std::string &base_topic, uint32_t queue_size, bool latch=false)
    this->color_pub_ = this->color_it_->advertiseCamera("image_raw", 30);
    this->depth_pub_ = this->depth_it_->advertiseCamera("image_raw", 30);
    this->ir_pub_ = this->ir_it_->advertiseCamera("image_raw", 30);
    this->alignedDepth_pub_ = this->alignedDepth_it_->advertiseCamera("image_raw", 30);
    this->alignedColor_pub_ = this->alignedColor_it_->advertiseCamera("image_raw", 30);
    this->depthCloudPointPub_ = depthCloudPoint_nh_.advertise<sensor_msgs::PointCloud2>("cloud_points",30);
    this->depth2colorCloudPointPub_ = depth2colorCloudPoint_nh_.advertise<sensor_msgs::PointCloud2>("cloud_points",30);
    this->depthCloudPointCameraInfoPub_ = depthCloudPoint_nh_.advertise<sensor_msgs::CameraInfo>("camera_info",30);
    this->depth2colorCloudPointCameraPub_ = depth2colorCloudPoint_nh_.advertise<sensor_msgs::CameraInfo>("camera_info",30);
    
    // Containers for frames
    ScStatus status;
    ros::Time now = ros::Time::now();
    int missed_frames = 0;

    ScFrame frameArr[5];
    memset(frameArr, 0,sizeof(ScFrame) * 5);

    sensor_msgs::ImagePtr msg_Ary[5]; // depth_msg,ir_msg,color_msg,alignedDetph_msg,alignedColor_msg
    image_transport::CameraPublisher pub_Ary[5] = {this->depth_pub_,this->ir_pub_,this->color_pub_,this->alignedDepth_pub_,this->alignedColor_pub_};
    while (ros::ok()) {
        ros::spinOnce();
        // Get next frame set
        if(config_.WorkMode == SC_SOFTWARE_TRIGGER_MODE && config_.SoftwareTrigger ==1)
        {
            scSoftwareTriggerOnce(deviceHandle_);
        }
        ScFrameReady psReadFrame = {0};
        ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
        if (status != SC_OK)
        {
            ++missed_frames;
            if (missed_frames >= kMaxConsecutiveFrameReadyFailures)
            {
                ROS_ERROR(
                    "相机连续取帧失败 %d 次，主动退出等待 roslaunch/systemd 守护重启。",
                    missed_frames
                );
                ros::shutdown();
                break;
            }
            continue;
        }
        missed_frames = 0;

        now = ros::Time::now();
        ScFrame frame = {0};
        if (psReadFrame.depth == 1)
        {
            scGetFrame(deviceHandle_, SC_DEPTH_FRAME, &frame);
            memcpy(&frameArr[0], &frame, sizeof(ScFrame));   
        }
        if (psReadFrame.ir == 1)
        {
            scGetFrame(deviceHandle_, SC_IR_FRAME, &frame);
            memcpy(&frameArr[1], &frame, sizeof(ScFrame));  
        }
        if (psReadFrame.color == 1)
        {
            scGetFrame(deviceHandle_, SC_COLOR_FRAME, &frame);
            memcpy(&frameArr[2], &frame, sizeof(ScFrame));  
        }
        if (psReadFrame.transformedDepth == 1)
        {
            scGetFrame(deviceHandle_, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &frame);
            memcpy(&frameArr[3], &frame, sizeof(ScFrame));  
        }
        if (psReadFrame.transformedColor == 1)
        {
            scGetFrame(deviceHandle_, SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME, &frame);
            memcpy(&frameArr[4], &frame, sizeof(ScFrame));  
        }

        bool ret = false;
        for(int ind = 0; ind  < 5; ind++)
        {
            frame = frameArr[ind];
            ScFrameType type = frame.frameType;
            if (frame.pFrameData != NULL)
            {
                ret = fillImagePtr(now, type, frame, cameraInfo_Ary[ind], msg_Ary[ind]);
                if(ret)
                {
                    pub_Ary[ind].publish(msg_Ary[ind],cameraInfo_Ary[ind]);
                    if (config_.DepthCloudPoint == true && type == SC_DEPTH_FRAME)
                    {
                        publishCloudPoint(now, frame, depthCloudPointPub_, cameraInfo_Ary[5], &frameArr[1]);
                    }
                    if (config_.Depth2ColorCloudPoint == true && type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
                    {
                        publishCloudPoint(now, frame, depth2colorCloudPointPub_, cameraInfo_Ary[6], frameArr);
                    }
                }
                else
                {
                    ROS_INFO_STREAM( "fill image failed for type : " << type );
                }
            }
        }
    }

    status = scStopStream(deviceHandle_);
    ROS_INFO_STREAM( "Stop Depth Frame status: " << status);
    status = scCloseDevice(&deviceHandle_);
    ROS_INFO_STREAM( "CloseDevice status: " << status);
    status = scShutdown();
    ROS_INFO_STREAM( "Shutdown status: " << status );
}


void ScepterManager::sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    ROS_ERROR("Segmentation fault, stopping camera driver (%d).", sig);    
    ros::shutdown();
}


void ScepterManager::checkScStatus(ScStatus status, const std::string &message_on_fail)
{
    if (status == ScStatus::SC_OK)
        return;
    ROS_ERROR("%s", message_on_fail.c_str());
}
