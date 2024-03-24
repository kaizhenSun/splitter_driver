#include "daheng.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <iostream>
// for delay function.
#include <chrono>
#include <thread>
#include <map>
#include <std_msgs/Time.h>

// for signal handling
#include <signal.h>
//#include <dv_ros_msgs/EventArray.h>
//#include <dv_ros_msgs/Trigger.h>
//#include <JetsonGPIO.h>

using namespace std;
using namespace cv;

mutex q_mutex;
mutex time_mutex;
queue<Mat> framesQueue;
string video_stream_provider_type;
double set_camera_fps;
double set_exposure_time;
std::string camera_info_url;
string frame_id;
int frame_id_global = 0;

std_msgs::Header header;
sensor_msgs::ImagePtr msg;
sensor_msgs::CameraInfo cam_info_msg;
string camera_name;
queue<ros::Time> timeQueue;
image_transport::CameraPublisher pub;
std::map<uint64_t,ros::Time> time_map;
std::vector<double> time_buffer;
std::map<uint64_t,ros::Time>::iterator time_map_it;
ros::Time start_ros_time;
double exposure_time;

void syncCallback(const std_msgs::TimeConstPtr& msg)
{
  time_buffer.push_back(msg->data.toSec());
  //frame_id_global++;
  //std::cout << "sync: " << std::to_string(msg->data.toSec()) << std::endl; 
}

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    //ros::Time time_now = ros::Time::now();
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    if(pFrame->status != GX_FRAME_STATUS_SUCCESS)
    {
        printf("<Abnormal Acquisition: Exception code: %d>\n", pFrame->status);
    }
    else
    {

        
        //std::cout << "now: " << std::to_string(time_now.toSec()) << std::endl; 
        //std::cout << "raw time: : " << pFrame->nTimestamp << std::endl; 
        //printf("<Successful acquisition: Width: %d Height: %d FrameID: %lu Timestamp: %lu>\n", 
        //           pFrame->nWidth, pFrame->nHeight, pFrame->nFrameID, pFrame->nTimestamp); 

        int nRet = PixelFormatConvert(pFrame);
        Mat frame(pFrame->nHeight, pFrame->nWidth, CV_8UC3, g_pRGBImageBuf);
            sensor_msgs::ImagePtr msg_thread = cv_bridge::CvImage(header, "rgb8", frame).toImageMsg();



                //std::string save_img_file = "/media/nail-agx/SSD/temporal_calibration/image_raw/" + std::to_string(time_buffer.back()) + ".png";
                //std::cout << save_img_file << std::endl; 
                //imwrite(save_img_file ,frame);

            if(time_buffer.empty()){
                printf("Error can't receive sync msg");
                return;
            }
               
            pub.publish(*msg_thread, cam_info_msg, ros::Time(time_buffer.back()));
                
            
    }
        
    //printf("<Acquisition Exit!>\n");

}
/*
void TriggerCallback(const dv_ros_msgs::Trigger::ConstPtr& msg)
{

     //for (auto it=time_map.begin(); it!=time_map.end(); ++it){
     //   cout << msg->secs << msg->nsecs;
     //}

    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Trigger Info: type: %d", 
             msg->type);
}
*/
int main(int argc, char** argv)
{


    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("camera_name", camera_name, std::string("camera"));
    ROS_INFO_STREAM("Camera name: " << camera_name);
    nh_private.param("frame_id", frame_id, std::string("camera"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);
    nh_private.param("exposure_time", exposure_time, 15000.0);
    ROS_INFO_STREAM("exposure_time: " << exposure_time);
    nh_private.param("camera_info_url", camera_info_url, std::string(""));
    ROS_INFO_STREAM("Provided camera_info_url: '" << camera_info_url << "'");

    image_transport::ImageTransport it(nh);
    pub = it.advertiseCamera("camera", 1);
    header.frame_id = frame_id;

    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;
    //ros::Subscriber person_info_sub = nh.subscribe("/trigger", 10, TriggerCallback);
    ros::Subscriber sync_sub = nh.subscribe("/sync", 10, syncCallback);




    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    uint32_t ui32DeviceNum = 0;

    //Initialize libary
    emStatus = GXInitLib(); 
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    //Get device enumerated number
    emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(emStatus != GX_STATUS_SUCCESS)
    { 
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }

    //If no device found, app exit
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    //Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;           
    }

    //Get Device Info
    printf("***********************************************\n");
    //Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    //Get string length of Vendor name
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Vendor name
    char *pszVendorName = new char[nSize];
    //Get Vendor name
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Vendor Name : %s>\n", pszVendorName);
    //Release memory for Vendor name
    delete[] pszVendorName;
    pszVendorName = NULL;

    //Get string length of Model name
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Model name
    char *pszModelName = new char[nSize];
    //Get Model name
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Model Name : %s>\n", pszModelName);
    //Release memory for Model name
    delete[] pszModelName;
    pszModelName = NULL;

    //Get string length of Serial number
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Serial number
    char *pszSerialNumber = new char[nSize];
    //Get Serial Number
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Serial Number : %s>\n", pszSerialNumber);
    //Release memory for Serial number
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    //Get string length of Device version
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_EXIT(emStatus);
    char *pszDeviceVersion = new char[nSize];
    //Get Device Version
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    //Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    printf("***********************************************\n");

    //Get the type of Bayer conversion. whether is a color camera.
    emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    GX_VERIFY_EXIT(emStatus);

    if (g_bColorFilter)
    {
        emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        GX_VERIFY_EXIT(emStatus);
    }

    emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    GX_VERIFY(emStatus);

    //Set acquisition mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(emStatus);





    // --------------------------- importatnt ---------------------------
    //Set trigger mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY_EXIT(emStatus);



    
    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    //GX_VERIFY_EXIT(emStatus);

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE ,GX_ACQUISITION_FRAME_RATE_MODE_ON);
    status = GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 30.0);


    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    //status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 25000.0);
    status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time);


    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    //status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    
    //Reverse image in X axis
    status = GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, true);
    // status = GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, false);


    
    //status = GXSendCommand(g_hDevice, GX_COMMAND_TIMESTAMP_LATCH_RESET);
    //status = GXSendCommand(g_hDevice, GX_COMMAND_TIMESTAMP_RESET);
    //status = GXSendCommand(g_hDevice, GX_COMMAND_TIMESTAMP_LATCH);
    //status = GXSendCommand(g_hDevice, GX_COMMAND_TIMESTAMP_LATCH_RESET);
    //status = GXSendCommand(g_hDevice, GX_COMMAND_TIMESTAMP_RESET);
    
    /*
    int64_t nBinningH = 2;
    int64_t nBinningV = 2;
    int64_t nDecimationH= 2;
    int64_t nDecimationV= 2;
    //设置水平和垂直 Binning 模式为 Sum 模式
    status = GXSetEnum(g_hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,
    GX_BINNING_HORIZONTAL_MODE_SUM);
    status = GXSetEnum(g_hDevice,GX_ENUM_BINNING_VERTICAL_MODE,
    GX_BINNING_VERTICAL_MODE_SUM);
    status = GXSetInt(g_hDevice, GX_INT_BINNING_HORIZONTAL, nBinningH);
    status = GXSetInt(g_hDevice, GX_INT_BINNING_VERTICAL, nBinningV);
    status = GXSetInt(g_hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
    status = GXSetInt(g_hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);
    */
    // --------------------------- importatnt ---------------------------



    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
    GX_VERIFY_EXIT(emStatus);

    bool bStreamTransferSize = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(emStatus);
    }

    bool bStreamTransferNumberUrb = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(emStatus);
    }
    
    //Prepare for Acquisition, alloc memory for image pixel format tansform
    PreForAcquisition();

    //Device start acquisition
    //emStatus = GXStreamOn(g_hDevice);


    //Start acquisition thread, if thread create failed, exit this app
    //int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, NULL);

    /*

    // Pin Definitions
    int output_pin = 22; // BOARD pin 12, BCM pin 18

    // Pin Setup.
    GPIO::setmode(GPIO::BOARD);
    // set pin as an output pin with optional initial state of HIGH
    GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);

    int curr_value = GPIO::HIGH;

    int frame_id = 0;
    ros::Rate r(30);
    while (nh.ok()) {

        ros::spinOnce();
        emStatus = GXSendCommand(g_hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
        GPIO::output(output_pin, curr_value);
        //  curr_value ^= GPIO::HIGH;
        time_map[frame_id] = ros::Time::now();
        frame_id++;
        r.sleep();
    }



    GPIO::cleanup();

    */

    status = GXSetEnum(g_hDevice,GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
    status = GXSetEnum(g_hDevice,GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);


    //设置触发激活方式为上升沿
    status = GXSetEnum(g_hDevice,GX_ENUM_TRIGGER_ACTIVATION,
    GX_TRIGGER_ACTIVATION_RISINGEDGE);
    //注册图像处理回调函数
    status = GXRegisterCaptureCallback(g_hDevice, NULL, OnFrameCallbackFun);

    //发送开采命令
    status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);


    ros::spin();

    status = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);
    //注销采集回调
    status = GXUnregisterCaptureCallback(g_hDevice);

    //Stop Acquisition thread
    g_bAcquisitionFlag = false;
    pthread_join(g_nAcquisitonThreadID, NULL);
    
    //Device stop acquisition
    emStatus = GXStreamOff(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    //Release the memory allocated
    UnPreForAcquisition();

    //Close device
    emStatus = GXCloseDevice(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    //Release libary
    emStatus = GXCloseLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    printf("<App exit!>\n");
    return 0;
}


