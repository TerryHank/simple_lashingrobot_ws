#include "tie_robot_perception/camera/scepter_manager.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
//define call back function
ScepterManager::ScepterManager(int32_t device_index, const string &camera_name) :
        color_nh_(camera_name + "/color"),
        depth_nh_(camera_name + "/depth"),
        ir_nh_(camera_name + "/ir"),
        alignedDepth_nh_(camera_name + "/transformedDepth"),        
        alignedColor_nh_(camera_name + "/transformedColor"),
        depthCloudPoint_nh_(camera_name + "/depthCloudPoint"),
        depth2colorCloudPoint_nh_(camera_name + "/depth2colorCloudPoint"),

        camera_name_(camera_name),
        color_info_(new camera_info_manager::CameraInfoManager(color_nh_)),
        depth_info_(new camera_info_manager::CameraInfoManager(depth_nh_)),
        ir_info_(new camera_info_manager::CameraInfoManager(ir_nh_)),
        alignedDepth_info_(new camera_info_manager::CameraInfoManager(alignedDepth_nh_)),
        alignedColor_info_(new camera_info_manager::CameraInfoManager(alignedColor_nh_)),
        depth_point_cloud_info_(new camera_info_manager::CameraInfoManager(depthCloudPoint_nh_)),
        depth2color_point_cloud_info_(new camera_info_manager::CameraInfoManager(depth2colorCloudPoint_nh_)),
        color_it_(new image_transport::ImageTransport(color_nh_)),
        depth_it_(new image_transport::ImageTransport(depth_nh_)),
        ir_it_(new image_transport::ImageTransport(ir_nh_)),
        alignedDepth_it_(new image_transport::ImageTransport(alignedDepth_nh_)),
        alignedColor_it_(new image_transport::ImageTransport(alignedColor_nh_)),
        color_width_(-1),
        color_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        sessionIndex_(0),
        hdrEnabled(false),
        wdrEnabled(false)
{
    signal(SIGSEGV, ScepterManager::sigsegv_handler);

    // Initialise the API
    checkScStatus(scInitialize(), "Initialize failed!");

    // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
	ScStatus status = scGetDeviceCount(&device_count, 3000);
	if (status != ScStatus::SC_OK || device_count < 1)
	{
        ROS_INFO("check device cost:%d second.", checkDeviceSec++);
        ros::Duration(1).sleep();
		goto GET;	
	}
    ROS_INFO("Get device count: %d", device_count);

    // Verify device index selection
    this->device_index_ = device_index;
    if (this->device_index_ < 0
        || this->device_index_ >= device_count)
        {
        throw std::runtime_error(
                "Device index outside of available devices range 0-" + std::to_string(device_count));
        }
       
	ScDeviceInfo* pPsDeviceInfoList = new ScDeviceInfo[device_count];
	status = scGetDeviceInfoList(device_count, pPsDeviceInfoList);
	if (status != ScStatus::SC_OK)
	{
		ROS_INFO("scGetDeviceInfoList failed! %d", status);
		delete[] pPsDeviceInfoList;
		pPsDeviceInfoList = NULL;
		goto GET;
	}

	ScDeviceInfo* pPsDeviceInfo = &pPsDeviceInfoList[0];

    // Attempt to open the device
    checkScStatus(scOpenDeviceBySN(pPsDeviceInfo->serialNumber, &deviceHandle_), "OpenDevice failed!");

    ROS_INFO("Successfully connected to device %d", this->device_index_);

    status= scStartStream(deviceHandle_);
    ROS_INFO_STREAM( "Start Depth Frame status: " << status);
    /* add user define api call start*/
    // such as call the scSetSpatialFilterEnabled
   
    /*
    status= scSetSpatialFilterEnabled(deviceHandle_,true);
    ROS_INFO_STREAM( "SetSpatialFilterEnabled status: " << status);
    */

    // such as call the scSetParamsByJson     
   
    /*
    char buffer[2048];
    getcwd(buffer, sizeof(buffer));
    string path(buffer);
    path = path + "/parameter.json";
    status = scSetParamsByJson(deviceHandle_, const_cast<char*>(path.c_str()));
    if (status != ScStatus::SC_OK)
    {
        ROS_INFO("scSetParamsByJson ret %d Please create json file at %s if you need it", status, const_cast<char*>(path.c_str()));
    }
    else
    {
        ROS_INFO("Successfully load json %s", const_cast<char*>(path.c_str()));
    }
    */
   
    /* add user define api call end*/
    // ScFlyingPixelFilterParams FlyingPixelFilterParams;
    // FlyingPixelFilterParams.enable = false;
    // FlyingPixelFilterParams.threshold = 1;
    // scSetFlyingPixelFilterParams(deviceHandle_, FlyingPixelFilterParams);
    // scSetSpatialFilterEnabled(deviceHandle_,true);

    // ScConfidenceFilterParams ConfidenceFilterParams;
    // ConfidenceFilterParams.enable = true;
    // ConfidenceFilterParams.threshold = 100; // 假设此字段用于设置置信度阈值
    // scSetConfidenceFilterParams(deviceHandle_, ConfidenceFilterParams);

    // scSetSpatialFilterEnabled(deviceHandle_, true);

    // scSetFillHoleFilterEnabled(deviceHandle_,true);

    // ScTimeFilterParams TimeFilterParams;
    // TimeFilterParams.enable = true;
    // TimeFilterParams.threshold = 3; 
    // scSetTimeFilterParams(deviceHandle_, TimeFilterParams);

    // ScIRGMMCorrectionParams IRGMMCorrectionParams;
    // IRGMMCorrectionParams.enable = false;
    // IRGMMCorrectionParams.threshold = 50;
    // scSetIRGMMCorrection(deviceHandle_, IRGMMCorrectionParams);

    // int rate=30;
    // scSetWorkMode(deviceHandle_, SC_ACTIVE_MODE);
    // status= scGetFrameRate(deviceHandle_,&rate);
    // config_.FrameRate=rate;
    // ROS_INFO_STREAM( "GetFrameRate status: " << status);

    // uint8_t gmmgain=50;
    // status= scGetIRGMMGain(deviceHandle_,&gmmgain);
    // config_.IRGMMGain = gmmgain;
    // ROS_INFO_STREAM( "GetIRGMMGain status: " << status);
   
    // ScWorkMode mode;
    // status= scGetWorkMode(deviceHandle_,&mode);
    // config_.WorkMode= mode;
    // ROS_INFO_STREAM( "GetWorkMode status: " << status);
    
    {
        ScExposureControlMode pControlMode;
        status= scGetExposureControlMode(deviceHandle_,SC_TOF_SENSOR,&pControlMode);
        config_.ToFManual = pControlMode;       
        ROS_INFO_STREAM( "GetExposureControlMode tof status: " << status);

        if(config_.ToFManual == SC_EXPOSURE_CONTROL_MODE_MANUAL)
        {
            int nExposureTime = 0;
            status= scGetExposureTime(deviceHandle_,SC_TOF_SENSOR,&nExposureTime);
            config_.ToFExposureTime = nExposureTime;
            ROS_INFO_STREAM( "GetExposureTime ToF status: " << status);
        }
    }
    {
        ScExposureControlMode pControlMode;
        status= scGetExposureControlMode(deviceHandle_,SC_COLOR_SENSOR,&pControlMode);
        config_.ColorManual = pControlMode;
        ROS_INFO_STREAM( "GetExposureControlMode color status: " << status);
        if(pControlMode == SC_EXPOSURE_CONTROL_MODE_MANUAL)
        {
            int nExposureTime = 3000;
            status= scGetExposureTime(deviceHandle_,SC_COLOR_SENSOR,&nExposureTime);
            config_.ColorExposureTime= nExposureTime;
            ROS_INFO_STREAM( "GetExposureTime color status: " << status);
        }
    }

    bool hdrEnable = false;
    status = scGetHDRModeEnabled(deviceHandle_, &hdrEnable);
    ROS_INFO_STREAM( "hdrEnable status: " << status << " hdrEnable " << hdrEnable);
    bool wdrEnable = false;
    status = scGetWDRModeEnabled(deviceHandle_, &wdrEnable);
    ROS_INFO_STREAM( "wdrEnable status: " << status << " wdrEnable " << wdrEnable);
    if (hdrEnable == false && wdrEnable == false)
    {
        config_.XDRMode = 0;
    }
    else if (hdrEnable == true && wdrEnable == false)
    {
        config_.XDRMode = 1;
    }
    else if (hdrEnable == false && wdrEnable == true)
    {
        config_.XDRMode = 2;
    }
    else
    {
        config_.XDRMode = -1;
        ROS_ERROR("XDRMode init error");
    }
    config_.DepthCloudPoint = true;
    config_.Depth2ColorCloudPoint = false;
    // ROS_INFO("ctl: %d %d %d %d %d %d %d %d %d %d %d",
    //             config_.FrameRate,
    //             config_.IRGMMGain,
    //             FlyingPixelFilterParams.enable,
    //             FlyingPixelFilterParams.threshold,
    //             config_.Confidenceenable,
    //             config_.Confidencevalue,
    //             config_.TimeFilterenable,
    //             config_.TimeFiltervalue,
    //             config_.IRGMMCorrectionenable,
    //             config_.IRGMMCorrectionvalue,
    //             config_.SpatialFilterEnabled,
    //             config_.FillHoleFilterEnabled,
    //             config_.ColorResloution,
    //             config_.ToFManual,
    //             config_.ToFExposureTime,
    //             config_.ColorManual,
    //             config_.ColorExposureTime,
    //             config_.WorkMode,
    //             config_.XDRMode,
    //             config_.DepthCloudPoint,
    //             config_.Depth2ColorCloudPoint);
}
 
