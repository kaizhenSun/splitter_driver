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
std::map<uint64_t,ros::Time>::iterator time_map_it;


void *ProcGetImage(void* pParam)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    while(g_bAcquisitionFlag)
    {
        // Get a frame from Queue
        emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                GetErrorString(emStatus);
                break;
            }
        }
        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
        else
        {
            //std::cout << "now: " << std::to_string((ros::Time::now()).toSec()) << std::endl; 
            //time_map_it = time_map.find(pFrameBuffer->nFrameID);
            //std::cout << "record: " << std::to_string(time_map_it->second.toSec()) << std::endl; 

            /* time_mutex.lock();
            ros::Time cur_time = timeQueue.front();
            time_mutex.unlock();
            printf("cur_time %.9f \n", cur_time.toSec());
            printf("(ros::Time::now()).toSec() %.9f \n", (ros::Time::now()).toSec());
            // cout << cur_time.toSec() << std::endl;
            // cout << (ros::Time::now()).toSec() << std::endl;
            printf("<Successful acquisition: Width: %d Height: %d FrameID: %lu Timestamp: %lu>\n", 
                   pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID, pFrameBuffer->nTimestamp); */
            
            int nRet = PixelFormatConvert(pFrameBuffer);
            Mat frame(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3, g_pRGBImageBuf);
            sensor_msgs::ImagePtr msg_thread = cv_bridge::CvImage(header, "rgb8", frame).toImageMsg();
            //if (cam_info_msg.distortion_model == ""){
            //    cam_info_msg = get_default_camera_info_from_image(msg_thread);
                //cam_info_manager.setCameraInfo(cam_info_msg);
            //}
            time_map_it = time_map.find(pFrameBuffer->nFrameID);
            //cout << time_map.size() << endl;
            //cout << "frameid: " << pFrameBuffer->nFrameID << endl;
            if (time_map_it != time_map.end()){
                pub.publish(*msg_thread, cam_info_msg, time_map_it->second);

                std::string save_img_file = "/media/nail-agx/SSD/temporal_calibration/image_raw/" + std::to_string(time_map_it->second.toSec()) + ".png";
                //std::cout << save_img_file << std::endl; 
                //imwrite(save_img_file ,frame);

                time_map.erase(time_map_it);
                //cout << time_map.size() << endl;

            }
                
            
        }
        
        emStatus = GXQBuf(g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }  
    }
    printf("<Acquisition thread Exit!>\n");

    return 0;
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
    ros::Publisher sync_pub = nh.advertise<std_msgs::Time>("/sync", 10);




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

    //Set trigger mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY_EXIT(emStatus);



    // --------------------------- importatnt ---------------------------
    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    //GX_VERIFY_EXIT(emStatus);

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE ,GX_ACQUISITION_FRAME_RATE_MODE_ON);
    status = GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 20.0);


    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    //status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 25000.0);
    status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 50000.0);


    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    //status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    
    status = GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, true);


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
    emStatus = GXStreamOn(g_hDevice);


    //Start acquisition thread, if thread create failed, exit this app
    int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, NULL);



    int frame_id = 0;
    ros::Rate r(18);
    while (nh.ok()) {

        ros::Time time_now = ros::Time::now();
        emStatus = GXSendCommand(g_hDevice, GX_COMMAND_TRIGGER_SOFTWARE);

        time_map[frame_id] = time_now;
        std_msgs::Time msg;
        msg.data = time_now;
        // sync_pub.publish(msg);

        frame_id++;
        r.sleep();
        sync_pub.publish(msg);
    }





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


