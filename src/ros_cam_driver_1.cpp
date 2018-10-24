/******************************************************************************/
/* GrabStream_FreerunUsingCallback.cpp                                        */
/*									                                          */
/* TeliCamSDK sample console application.                                     */
/* Access camera feature register using TeliCamApi default functions          */
/*																			  */
/* GenApi modules are used inside TeliCamApi                                  */
/*									                                          */
/******************************************************************************/

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
//#include "ros/"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#if defined (_WIN32)
    #include <windows.h>
    #include <tchar.h>
    #include <conio.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "/opt/TeliCamSDK/include/TeliCamApi.h"
//#include "/opt/TeliCamSDK/include/TeliCamUtl.h"

#include "TeliCamApi.h"
#include "TeliCamUtl.h"

using namespace Teli;
using namespace std;

/******************************************************************************/
/* Prototype declares                                                         */
/******************************************************************************/
uint32_t Initialize();
void Terminate();

uint32_t SetFeature();
uint32_t SetTriggerMode();
uint32_t OpenStream();
uint32_t CloseStream();

uint32_t StartAcquisition();
uint32_t StopAcquisition();

uint32_t ProcessLoop();
uint32_t DispFeature();
void CALLBACK CallbackImageAcquired(CAM_HANDLE hRcvCam, CAM_STRM_HANDLE hRcvStrm, CAM_IMAGE_INFO *psImageInfo, uint32_t uiBufferIndex, void *pvContext);
void CALLBACK CallbackImageError(CAM_HANDLE hRcvCam, CAM_STRM_HANDLE	hRcvStrm, uint32_t uiBufferIndex, CAM_API_STATUS iErrorStatus, void *pvContext);
void CALLBACK CallbackBufferBusy(CAM_HANDLE hRcvCam, CAM_STRM_HANDLE	hRcvStrm, uint32_t uiBufferIndex, void *pvContext);
uint32_t ImageHandling();
uint32_t SetExposureTime();
uint32_t SetGain();

#if !defined (_WIN32)
uint32_t _kbhit();
#endif

void clearkey();

/*****************************************************************************/
/* Static Values                                                             */
/*****************************************************************************/

//
static CAM_API_STATUS	s_uiStatus = CAM_API_STS_SUCCESS;

static bool8_t			s_bU3vCamera;

static uint32_t			s_uiImgBufSize	= 0;		// Size of image buffer.
static double			s_dMultDspExp	= 0.001;	// Coefficient for converting micro second data to mili second data.
static bool8_t			s_bStartStream	= false;

// Handles
static CAM_HANDLE		s_hCam		= (CAM_HANDLE)NULL;		// Camera handle
static CAM_STRM_HANDLE	s_hStrm		= (CAM_STRM_HANDLE)NULL;		// Stream handle

/*****************************************************************************/
/* Functions                                                                 */
/*****************************************************************************/

ros::Publisher imagePublisher;
ros::Subscriber imuListener;

// init index/time lists
ros::Time imuTime;
//int imuCnt = 1;
//int imgCnt = 1;
//map<int, ros::Time> timeList;

int camera_index;
string message_name;
string image_frame_name;
//string ros_node_name;

ros::Time cameraTime;

// features
bool ifTrigger;
int exposure_value;
int gain_value;

// listen to the time of IMU sensor
void imuCallBack(const sensor_msgs::Imu::ConstPtr& imu)
{
    imuTime = imu->header.stamp;
//    timeList.insert(make_pair(imuCnt, imuTime));
//    imuCnt++;
}

#if defined (_WIN32)
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char **argv)
#endif
{
    // must be after the init?
//    ros::param::get("ros_node_name", ros_node_name);

    ros::init(argc, argv, "ros_cam_driver_1");

    ros::NodeHandle n;

    n.getParam("/ros_cam_driver_1/camera_index", camera_index);
    n.getParam("/ros_cam_driver_1/message_name", message_name);
    n.getParam("/ros_cam_driver_1/image_frame_name", image_frame_name);


    n.getParam("/ros_cam_driver_1/exposure_value", exposure_value);
    n.getParam("/ros_cam_driver_1/gain_value", gain_value);
    n.getParam("/ros_cam_driver_1/ifTrigger", ifTrigger);

    imagePublisher = n.advertise<sensor_msgs::Image>(message_name, 1);

    imuListener = n.subscribe("/mti/sensor/imu", 100, imuCallBack);

    ros::AsyncSpinner spinner(1); // Use another thread
    spinner.start();

    uint32_t  uiStatus = 0;

    printf("GrabStreamSimple sample application for TeliCamSDK.\n\n");
    printf("Copyrights (C) 2015 - 2017 TOSHIBA TELI CORPORATION. All rights reserved.\n\n");
    printf(" Access camera feature registers using TeliCamApi default functions.\n");
    printf(" GenApi modules are used inside TeliCamApi.\n\n");

    do
    {
        uiStatus = Initialize();
        if (uiStatus != 0)
        {
            printf("Failed Initialize(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        // Set features.
        uiStatus = SetFeature();
        if (uiStatus != 0)
        {
            printf("Failed SetFeature(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        // Open stream.
        uiStatus = OpenStream();
        if (uiStatus != 0)
        {
            printf("Failed OpenStream(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        // Set callback functions.
        uiStatus = Strm_SetCallbackImageAcquired(s_hStrm, NULL, CallbackImageAcquired);
        if (uiStatus != CAM_API_STS_SUCCESS)
        {
            printf("Failed Strm_SetCallbackImageAcquired(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        uiStatus = Strm_SetCallbackImageError(s_hStrm, NULL, CallbackImageError);
        if (uiStatus != CAM_API_STS_SUCCESS)
        {
            printf("Failed Strm_SetCallbackImageError(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        uiStatus = Strm_SetCallbackBufferBusy(s_hStrm, NULL, CallbackBufferBusy);
        if (uiStatus != CAM_API_STS_SUCCESS)
        {
            printf("Failed Strm_SetCallbackBufferBusy(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }

        // Trigger in set sfeature
        /*
        // Set trigger mode.(freerun mode)
        uiStatus = SetTriggerMode();
        if (uiStatus != 0)
        {
            printf("Failed SetTriggerMode(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
            break;
        }
        */

        // Receive images.
        uiStatus = ProcessLoop();
        if (uiStatus != 0)
        {
            printf("Failed ProcessLoop(), Location = %d, Status = 0x%08X.\n", uiStatus, s_uiStatus);
        }
    } while(0);

    ros::waitForShutdown();

    // Close stream.
    CloseStream();

    // Terminate.
    Terminate();

    printf("Finish!\n\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t Initialize()
{
    uint32_t  uiNum;
    uint32_t  uiMax;


    printf("\n");
    printf("Initialize() started!\n");

    // API initialization.
    s_uiStatus = Sys_Initialize();
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 1;
    }

    // Get SDK system information.
    CAM_SYSTEM_INFO sSysInfo;
    s_uiStatus = Sys_GetInformation(&sSysInfo);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 2;
    }
    printf(" <System information>\n");
    printf("  TeliU3vDriver  version : %s\n", sSysInfo.sU3vInfo.szDriverVersion);
    printf("  TeliU3vApi2    version : %s\n", sSysInfo.sU3vInfo.szDllVersion);
    printf("  TeliU3vCamApi  version : %s\n", sSysInfo.sU3vInfo.szDllExVersion);
    printf("  TeliGevApi2    version : %s\n", sSysInfo.sGevInfo.szDllVersion);
    printf("  TeliGevCamApi  version : %s\n", sSysInfo.sGevInfo.szDllExVersion);
    printf("  TeliCamApi     version : %s\n", sSysInfo.szDllVersion);

    // Get uiNumber of camera.
    s_uiStatus = Sys_GetNumOfCameras(&uiNum);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 3;
    }
    if (uiNum == 0)
    {
        printf(" No cameras found.\n");
        return 4;
    }
    printf(" %d camera(s) found.\n", uiNum);

    // Get camera information.
    CAM_INFO		sCamInfo;
    U3V_CAM_INFO	*psU3vCamInfo;
    GEV_CAM_INFO	*psGevCamInfo;

    // print one camera,

    memset((void*)&sCamInfo, 0, sizeof(CAM_INFO));
    // Get information of a camera.
    s_uiStatus = Cam_GetInformation((CAM_HANDLE)NULL, camera_index, &sCamInfo);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 5;
    }

    printf("\n<Camera%d information>\n", camera_index);
    if (sCamInfo.eCamType == CAM_TYPE_U3V)
        printf(" Type                               : USB3 Vision camera\n");
    else if (sCamInfo.eCamType == CAM_TYPE_GEV)
        printf(" Type                               : GigE Vision camera\n");

    printf(" Manufacturer                       : %s\n", sCamInfo.szManufacturer);
    printf(" Model name                         : %s\n", sCamInfo.szModelName);
    printf(" Serial number                      : %s\n", sCamInfo.szSerialNumber);
    printf(" User defined name                  : %s\n", sCamInfo.szUserDefinedName);

    if (sCamInfo.eCamType == CAM_TYPE_U3V) {
        psU3vCamInfo = &sCamInfo.sU3vCamInfo;

        printf(" U3v family name                    : %s\n", psU3vCamInfo->szFamilyName);
        printf(" U3v device version                 : %s\n", psU3vCamInfo->szDeviceVersion);
        printf(" U3v manufacturer information       : %s\n", psU3vCamInfo->szManufacturerInfo);
        printf(" U3v adapter vendor ID              : 0x%04X\n", psU3vCamInfo->uiAdapterVendorId);
        printf(" U3v adapter device ID              : 0x%04X\n", psU3vCamInfo->uiAdapterDeviceId);
        printf(" U3v Adapter default MaxPacketSize  : %d\n", psU3vCamInfo->uiAdapterDfltMaxPacketSize);
    } else if (sCamInfo.eCamType == CAM_TYPE_GEV) {
        psGevCamInfo = &sCamInfo.sGevCamInfo;

        printf(" Gev display name                   : %s\n", psGevCamInfo->szDisplayName);
        printf(" Gev MAC address                    : %02X-%02X-%02X-%02X-%02X-%02X\n",
            psGevCamInfo->aucMACAddress[0],
            psGevCamInfo->aucMACAddress[1],
            psGevCamInfo->aucMACAddress[2],
            psGevCamInfo->aucMACAddress[3],
            psGevCamInfo->aucMACAddress[4],
            psGevCamInfo->aucMACAddress[5]);
        printf(" Gev support IP LLA                 : %d\n", psGevCamInfo->cSupportIP_LLA);
        printf(" Gev support IP DHCP                : %d\n", psGevCamInfo->cSupportIP_DHCP);
        printf(" Gev Support IP Persistent-IP       : %d\n", psGevCamInfo->cSupportIP_Persistent);
        printf(" Gev current IP LLA                 : %d\n", psGevCamInfo->cCurrentIP_LLA);
        printf(" Gev current IP DHCP                : %d\n", psGevCamInfo->cCurrentIP_DHCP);
        printf(" Gev current IP Persistent-IP       : %d\n", psGevCamInfo->cCurrentIP_Persistent);
        printf(" Gev IP Address                     : %d.%d.%d.%d\n",
            psGevCamInfo->aucIPAddress[0],
            psGevCamInfo->aucIPAddress[1],
            psGevCamInfo->aucIPAddress[2],
            psGevCamInfo->aucIPAddress[3]);
        printf(" Gev subnet mask                    : %d.%d.%d.%d\n",
            psGevCamInfo->aucSubnet[0],
            psGevCamInfo->aucSubnet[1],
            psGevCamInfo->aucSubnet[2],
            psGevCamInfo->aucSubnet[3]);
        printf(" Gev default gateway                : %d.%d.%d.%d\n",
            psGevCamInfo->aucGateway[0],
            psGevCamInfo->aucGateway[1],
            psGevCamInfo->aucGateway[2],
            psGevCamInfo->aucGateway[3]);
        printf(" Gev adapter MAC address            : %02X-%02X-%02X-%02X-%02X-%02X\n",
            psGevCamInfo->aucAdapterMACAddress[0],
            psGevCamInfo->aucAdapterMACAddress[1],
            psGevCamInfo->aucAdapterMACAddress[2],
            psGevCamInfo->aucAdapterMACAddress[3],
            psGevCamInfo->aucAdapterMACAddress[4],
            psGevCamInfo->aucAdapterMACAddress[5]);
        printf(" Gev adapter IP address             : %d.%d.%d.%d\n",
            psGevCamInfo->aucAdapterIPAddress[0],
            psGevCamInfo->aucAdapterIPAddress[1],
            psGevCamInfo->aucAdapterIPAddress[2],
            psGevCamInfo->aucAdapterIPAddress[3]);
        printf(" Gev adapter subnet mask            : %d.%d.%d.%d\n",
            psGevCamInfo->aucAdapterSubnet[0],
            psGevCamInfo->aucAdapterSubnet[1],
            psGevCamInfo->aucAdapterSubnet[2],
            psGevCamInfo->aucAdapterSubnet[3]);
        printf(" Gev adapter default gateway        : %d.%d.%d.%d\n",
            psGevCamInfo->aucAdapterGateway[0],
            psGevCamInfo->aucAdapterGateway[1],
            psGevCamInfo->aucAdapterGateway[2],
            psGevCamInfo->aucAdapterGateway[3]);
        printf(" Gev adapter display name           : %s\n", psGevCamInfo->szAdapterDisplayName);
    }

    // Open camera that is detected first, in this sample code.
    uint32_t iCamNo = (uint32_t)camera_index;
    s_uiStatus = Cam_Open(iCamNo, &s_hCam);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 6;
    }
    printf(" Camera opened.\n");

    // Check camera interface type.
    CAM_TYPE eType;
    s_uiStatus = GetCamTypeFromCamHandle(s_hCam, &eType);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 7;
    }

    if (eType == CAM_TYPE_U3V)
    {
        s_bU3vCamera = true;
    } else
    {
        s_bU3vCamera = false;
    }

    // Get sensor width.
    s_uiStatus = GetCamSensorWidth(s_hCam, &uiMax);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 8;
    }
    printf(" Pixel Width                        : %4d\n",uiMax);

    // Get sensor height.
    s_uiStatus = GetCamSensorHeight(s_hCam, &uiMax);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 9;
    }
    printf(" Pixel Height                       : %4d\n",uiMax);

    printf("Initialize() successful.\n");

    return 0;
}

/////////////////////////////////////////////////////////
//
void Terminate()
{
    printf("\n");
    printf("Terminate() started!\n");

    // Close camera.
    if (s_hCam != (CAM_HANDLE)NULL)
    {
        Cam_Close(s_hCam);
        s_hCam = (CAM_HANDLE)NULL;
    }

    // Terminate system.
    Sys_Terminate();

    printf("Terminate() successful.\n");
}

/////////////////////////////////////////////////////////
//
uint32_t SetFeature()
{
    printf("\n");
    printf("SetFeature() started!\n");

    /**  uncommnt for the projetct, re-write these features    **/
    /*
    // ExposureTime / ExposureMode.
    {
        if (s_bU3vCamera == true)
        {
            // Set ExposureTimeControl "Manual", in this sample code.
            s_uiStatus = SetCamExposureTimeControl(s_hCam, CAM_EXPOSURE_TIME_CONTROL_MANUAL);
            if (s_uiStatus != CAM_API_STS_SUCCESS)
            {
                return 20;
            }
        } else
        {
            // Set ExposureTimeControl "Manual", in this sample code. (ExposureAuto = Off)
            s_uiStatus = SetCamExposureTimeControl(s_hCam, CAM_EXPOSURE_TIME_CONTROL_MANUAL);
            if (s_uiStatus != CAM_API_STS_SUCCESS)
            {
                return 22;
            }
        }
    }
*/
    // GainAuto.
    {
        if (s_bU3vCamera == true)
        {
            // Do nothing.
        } else
        {
            // Set GainAuto off.
            s_uiStatus = SetCamGainAuto(s_hCam, CAM_GAIN_AUTO_OFF);
            if (s_uiStatus != CAM_API_STS_SUCCESS)
            {
                return 27;
            }
        }
    }

    // AcquisitionFrameRateControl.
    {
        if (s_bU3vCamera == true)
        {
            // Set AcquisitionFramerRateControl NoSpecify.
            s_uiStatus = SetCamAcquisitionFrameRateControl(s_hCam, CAM_ACQ_FRAME_RATE_CTRL_NO_SPECIFY);
            if (s_uiStatus != CAM_API_STS_SUCCESS)
            {
                return 29;
            }
        }
    }


    // add more features for cameras, ritten by yh 2018.1024

    // Trigger mode
    {
        SetCamTriggerMode(s_hCam, ifTrigger);
    }

    // exposure
    {
        SetCamExposureTimeControl(s_hCam, CAM_EXPOSURE_TIME_CONTROL_MANUAL);
        SetCamExposureTime(s_hCam, (float64_t)exposure_value);
    }

    // white balance
    {
        s_uiStatus = SetCamBalanceWhiteAuto(s_hCam, CAM_BALANCE_WHITE_AUTO_ONCE);
        if (s_uiStatus != CAM_API_STS_SUCCESS)
        {
            cout<<"!!!!!!"<<endl;
        }
    }

    // Gain
    {
        s_uiStatus= SetCamGain(s_hCam, (float64_t)gain_value);
    }


    printf("SetFeature() successful.\n");
    return 0;
}

/////////////////////////////////////////////////////////
// Trigger mode value should be set before calling Strm_StartingAcquisition().
//
uint32_t SetTriggerMode()
{
// This application uses freerun mode.
//   TriggerMode : Off.
    printf("\n");
    printf("SetTriggerMode() started!\n");

    // Set TriggerMode false, in this sample code.
    s_uiStatus = SetCamTriggerMode(s_hCam, false);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 40;
    }

    printf("SetTriggerMode() successful.\n");
    return 0;
}


/////////////////////////////////////////////////////////
//
uint32_t OpenStream()
{
    printf("\n");
    printf("OpenStream() started!\n");

    // Open stream channel.
    // Use default MaxPacketSize(U3v : 65536 or 524288 , GEV : 1500).
    // Payload size will be written to uiPyld,
    s_uiStatus = Strm_OpenSimple(s_hCam, &s_hStrm, &s_uiImgBufSize, NULL);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 81;
    }
    printf(" Payload size : %d[byte]\n", s_uiImgBufSize);

    printf("OpenStream() successful.\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t CloseStream()
{
    printf("\n");
    printf("CloseStream() started!\n");

    // Close stream.
    if (s_hStrm != (CAM_STRM_HANDLE)NULL)
    {
        Strm_Close(s_hStrm);
        s_hStrm = (CAM_STRM_HANDLE)NULL;
    }

    printf("CloseStream() successful.\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t StartAcquisition()
{
    printf("\n");
    printf("StartAcquisition() started!\n");

    s_uiStatus = Strm_Start(s_hStrm);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 100;
    }

    s_bStartStream = true;

    printf("StartAcquisition() successful.\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t StopAcquisition()
{
    printf("\n");
    printf("StopAcquisition() started!\n");

    s_bStartStream = false;

    s_uiStatus = Strm_Stop(s_hStrm);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 120;
    }

    printf("StopAcquisition() successful.\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t SetExposureTime()
{
    float64_t    dExp, dExpMin, dExpMax;


    printf("\n");
    printf("SetExposureTime() started!\n");

    // Get Exposure time minimum and maximum values.
    s_uiStatus = GetCamExposureTimeMinMax(s_hCam, &dExpMin, &dExpMax);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 141;
    }

    printf(" Exposure time min. : %.4lf, max. : %.4lf[ms] >>> ", dExpMin * s_dMultDspExp, dExpMax * s_dMultDspExp);
#if defined (_WIN32)
    scanf_s("%lf", &dExp);
#else
    if (scanf("%lf", &dExp) == EOF)
    {
        return 143;
    }
#endif
    dExp /= s_dMultDspExp;

    // Set ExposureTime.
    s_uiStatus = SetCamExposureTime(s_hCam, dExp);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 144;
    }

    // Get current ExposureTime value.
    s_uiStatus =  GetCamExposureTime(s_hCam, &dExp);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 145;
    }
    printf(" Current exposure time : %.4lf[ms].\n", dExp * s_dMultDspExp);

    printf("SetExposureTime() finished!\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t SetGain()
{
    float64_t    dGain, dGainMin, dGainMax;


    printf("\n");
    printf("SetGain() started!\n");

    // Get Gain minimum and maximum values.
    s_uiStatus = GetCamGainMinMax(s_hCam, &dGainMin, & dGainMax);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 161;
    }

    printf(" Gain min. :%.4lf, max. :%.4lf [db] >>> ", dGainMin, dGainMax);
#if defined (_WIN32)
    scanf_s("%lf", &dGain);
#else
    if (scanf("%lf", &dGain) == EOF)
    {
        return 164;
    }
#endif
    // Set Gain.
    s_uiStatus = SetCamGain(s_hCam, dGain);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 165;
    }

    // Get current Gain value.
    s_uiStatus = GetCamGain(s_hCam, &dGain);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 166;
    }
    printf(" Current gain : %.4lf[db].\n", dGain);

    printf("SetGain() finished!\n");
    return 0;
}

/////////////////////////////////////////////////////////
//
#if !defined (_WIN32)
uint32_t _kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}
#endif

void clearkey()
{
    while(1)
    {
        if (!_kbhit())
        {
            break;
        }
#if defined (_WIN32)
        _getch();
#else
        getchar();
#endif
    }
}


/////////////////////////////////////////////////////////
//

void CALLBACK CallbackImageAcquired(
    CAM_HANDLE		hRcvCam,
    CAM_STRM_HANDLE	hRcvStrm,
    CAM_IMAGE_INFO	*psImageInfo,
    uint32_t		uiBufferIndex,
    void			*pvContext)
{
    uint8_t			*pImageBuf = NULL;

    pImageBuf = (uint8_t*)psImageInfo->pvBuf;

    // now Time
    cameraTime = imuTime;

//#if defined (_WIN32)
//        printf("Received ImageAcquired event.  Block id : %I64u\n", (uint64_t)psImageInfo->ullBlockId);
//#else
//        printf("Received ImageAcquired event.  Block id : %lld\n", (long long uint64_t)psImageInfo->ullBlockId);
//#endif

//    // Display the first 8 pixel values.
//    printf("  ImageBuffer :");
//    for (int i = 0; i < 8; i++)
//    {
//        printf(" %02X ", pImageBuf[i]);
//    }
//    printf("...\n");

    // no display, transform and publish, yh

    cv::Mat des_image;
    cv::Mat src_image(psImageInfo->uiSizeY, psImageInfo->uiSizeX, CV_8UC1, pImageBuf);

//    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE ); // Create a window for display.
//    cv::waitKey(100);

    cv::cvtColor(src_image, des_image, 48, 0);

//    cv::imshow( "Display window", des_image );                // Show our image inside it.

//    ros::Time timeStamp(psImageInfo->ullTimestamp/1000000000, psImageInfo->ullTimestamp);
//    ros::Time timeStamp = ros::Time::now();
//    cout<<timeStamp.toSec()<<endl;

    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = image_frame_name;
    out_msg.header.seq = (uint32_t)psImageInfo->ullBlockId;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
    out_msg.image    = des_image; // Your cv::Mat
    out_msg.header.stamp   = cameraTime; // a little time delay

    imagePublisher.publish(out_msg.toImageMsg());

//    imgCnt++;

}

/////////////////////////////////////////////////////////
//
void CALLBACK CallbackImageError(
    CAM_HANDLE		hRcvCam,
    CAM_STRM_HANDLE	hRcvStrm,
    CAM_API_STATUS	iErrorStatus,
    uint32_t		uiBufferIndex,
    void			*pvContext)
{
    // Ignore if the stream interface has been stopped.
    // When you call the Strm_Stop() function in the stream output, CAM_API_STS_PACKET_STATUS_ERROR may occur. (not abnormal.)
    if (!s_bStartStream)
    {
        return;
    }
    printf("Image Error! iErrorStatus:(%d)\n", iErrorStatus);
}

/////////////////////////////////////////////////////////
//
void CALLBACK CallbackBufferBusy(
    CAM_HANDLE		hRcvCam,
    CAM_STRM_HANDLE	hRcvStrm,
    uint32_t		uiBufferIndex,
    void			*pvContext)
{
    printf("Buffer Busy!\n");
}

/////////////////////////////////////////////////////////
//
uint32_t ImageHandling()
{
#if !defined (_WIN32)
    clearkey();
#endif

    // Start acquisition
    s_uiStatus = StartAcquisition();
    if (s_uiStatus != 0)
    {
        printf("Failed StartAcquisition(), Status = 0x%08X.\n", s_uiStatus);
        return 180;
    }

    while(1)
    {
        if (_kbhit())
        {
            break;
        }
#if defined (_WIN32)
        Sleep(1);
#else
        usleep(1000);
#endif
    }

    clearkey();

    // Stop acquisition.
    s_uiStatus = StopAcquisition();
    if(s_uiStatus != 0)
    {
        printf("Failed StopAcquisition(), Status = 0x%08X.\n", s_uiStatus);
        return 184;
    }

#if defined (_WIN32)
        Sleep(100);
#else
        usleep(100000);
#endif

    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t DispFeature()
{
    float64_t dExp, dGain, dFRate;


    // Exposure
    s_uiStatus = GetCamExposureTime(s_hCam, &dExp);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 201;
    }
    printf(" Exposure time : %.4lf[ms]\n", dExp * s_dMultDspExp);

    // Gain
    s_uiStatus = GetCamGain(s_hCam, &dGain);
    if (s_uiStatus != CAM_API_STS_SUCCESS)
    {
        return 204;
    }
    printf(" Gain          : %.4lf[db]\n", dGain);

    // Framerate
    {
        // Get AcquisitionFrameRate
        s_uiStatus = GetCamAcquisitionFrameRate(s_hCam, &dFRate);
        if (s_uiStatus != CAM_API_STS_SUCCESS)
        {
            return 207;
        }
        printf(" Frame rate    : %.4lf[fps]\n", dFRate);
    }

    return 0;
}

/////////////////////////////////////////////////////////
//
uint32_t ProcessLoop()
{
    int       iKey = 0;
    uint32_t  uiStatus = 0;
//    bool8_t   bDisp = true;


    printf("\n");
    printf("ProcessLoop() started!\n");

    while (1)
    {
//        if (bDisp)
//        {
//            printf("\n");
//            DispFeature();

//            printf("---------------------------------------------------------------------\n");
//            printf("Press '0' key to grab frames.\n");
//            printf("Press '1' key to set exposure time\n");
//            printf("Press '2' key to set gain\n");
//            printf("Press '9' key to quit application.\n");
//            printf("---------------------------------------------------------------------\n");

//            bDisp = false;
//        }

//#if defined (_WIN32)
//        iKey = _getch();
//#else
//        clearkey();
//        iKey = getchar();
//#endif

//        if (iKey == '0')
//        {
//            //
            uiStatus = ImageHandling();
            if (uiStatus != CAM_API_STS_SUCCESS)
            {
                break;
            }
//            bDisp = true;
//        } else if (iKey == '1')
//        {
//            //
//            uiStatus = SetExposureTime();
//            if (uiStatus != CAM_API_STS_SUCCESS)
//            {
//                break;
//            }
//            bDisp = true;
//        } else if (iKey == '2')
//        {
//            //
//            uiStatus = SetGain();
//            if (uiStatus != CAM_API_STS_SUCCESS)
//            {
//                break;
//            }
//            bDisp = true;
//        } else if (iKey == '9')
//        {
//            break;
//        }
    }

    printf("ProcessLoop() finished!\n");
    return uiStatus;
}
