//Modified by Pierre-Yves Brèches //
//
//
//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $
//=============================================================================


#ifdef LINUX
#include <unistd.h>
#endif

#include "FlyCapture2.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/fill_image.h>
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"

using namespace FlyCapture2;


void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf( 
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

bool CheckSoftwareTriggerPresence( Camera* pCam )
{
	const unsigned int k_triggerInq = 0x530;

	Error error;
	unsigned int regVal = 0;

	error = pCam->ReadRegister( k_triggerInq, &regVal );

	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return false;
	}

	if( ( regVal & 0x10000 ) != 0x10000 )
	{
		return false;
	}

	return true;
}

bool PollForTriggerReady( Camera* pCam )
{
    const unsigned int k_softwareTrigger = 0x62C;
    Error error;
    unsigned int regVal = 0;

    do 
    {
        error = pCam->ReadRegister( k_softwareTrigger, &regVal );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }

    } while ( (regVal >> 31) != 0 );

    return true;
}

bool FireSoftwareTrigger( Camera* pCam )
{
    const unsigned int k_softwareTrigger = 0x62C;
    const unsigned int k_fireVal = 0x80000000;
    Error error;    

    error = pCam->WriteRegister( k_softwareTrigger, k_fireVal );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return false;
    }

    return true;
}

bool grabImage(sensor_msgs::Image &image, Camera* cam, std::string frame_id, ros::Time now){
    if(cam->IsConnected()){
// Make a FlyCapture2::Image to hold the buffer returned by the camera.
        Image rawImage;
// Retrieve an image

        Error error = cam->RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return false;
        }
 
        ImageMetadata metadata;
        metadata = rawImage.GetMetadata();
// Set header timestamp as embedded for now
        //TimeStamp embeddedTime = rawImage.GetTimeStamp();

        image.header.stamp = now;
// Get camera info to check if color or black and white chameleon and check the bits per pixel.
        CameraInfo cInfo;

// Set the image encoding
        std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
	clearImage(image);
        fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
        image.header.frame_id = frame_id;
        return true;
        
    }
    else
    {
        printf("camera not connected");
        return false;
    }
}







int main(int argc, char** argv)
{	
	ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    ros::NodeHandle n1 (n,"/camera1");
    ros::NodeHandle n2 (n,"/camera2");
    
    image_transport::ImageTransport it(n1);
    image_transport::Publisher it_pub = it.advertise("image_raw", 1);
    //ros::Publisher pub =n1.advertise<sensor_msgs::Image>("image1", 1);

    //image_transport::CameraPublisher it_pub;

    image_transport::ImageTransport it2(n2);
    image_transport::Publisher it_pub2 = it2.advertise ("image_raw", 1);
    //ros::Publisher pub2 =n2.advertise<sensor_msgs::Image>("image2", 1);

    
    //image_transport::CameraPublisher it_pub2;
    std::string frame_id="/camera1";
    camera_info_manager::CameraInfoManager cinfo(n1);  ///< Helper class to manage the calibration info for the camera
    ros:: Publisher pubinfo = n1.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    std::string frame_id2="/camera2";
    camera_info_manager::CameraInfoManager cinfo2(n2);
    ros::Publisher pubinfo2 = n2.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

	
    std::string url="file:///home/linaro/ROSTryphon_Organized/src/cameras/launch/calibration/camera1.yaml";
    //n.getParam("camera_info_url", url);
    cinfo.loadCameraInfo(url);
    std::string url2="file:///home/linaro/ROSTryphon_Organized/src/cameras/launch/calibration/camera2.yaml";
    //n.getParam("camera_info_url2", url2);
    cinfo2.loadCameraInfo(url2);

    
    PrintBuildInfo();

    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... exiting\n" );
        return -1;
    }

    unsigned int serialmaster=14316843;
    unsigned int serialslave=14316841;

    PGRGuid guid;
    error = busMgr.GetCameraFromSerialNumber(serialmaster, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    PGRGuid guid2;
    error = busMgr.GetCameraFromSerialNumber(serialslave, &guid2);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }


    Camera cam,cam2;

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = cam2.Connect(&guid2);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = cam.SetEmbeddedImageInfo(&info);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = cam2.SetEmbeddedImageInfo(&info);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Get the camera information
CameraInfo camInfo;
error = cam.GetCameraInfo(&camInfo);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

PrintCameraInfo(&camInfo);      

error = cam2.GetCameraInfo(&camInfo);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

PrintCameraInfo(&camInfo);

    // Check for external trigger support
TriggerModeInfo triggerModeInfo;
error = cam.GetTriggerModeInfo( &triggerModeInfo );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

if ( triggerModeInfo.present != true )
{
    printf( "Camera does not support external trigger! Exiting...\n" );
    return -1;
}

    // Get current trigger settings
TriggerMode triggerMode;
error = cam.GetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

    // Set camera to trigger mode 0
triggerMode.onOff = true;
triggerMode.mode = 0;
triggerMode.parameter = 0;

    // Triggering the camera externally using source 0.
triggerMode.source = 7;

error = cam.SetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

error = cam2.SetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

    
    // Poll to ensure camera is ready
bool retVal = PollForTriggerReady( &cam );
if( !retVal )
    {
      printf("\nError polling for trigger ready!\n");
      return -1;
    }

bool retVal2 = PollForTriggerReady( &cam2 );
if( !retVal )
    {
      printf("\nError polling for trigger ready!\n");
      return -1;
    }

    // Get the camera configuration
FC2Config config;
error = cam.GetConfiguration( &config );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
    return -1;
} 

    // Set the grab timeout to 5 seconds
config.grabTimeout = 5000;

    // Set the camera configuration
error = cam.SetConfiguration( &config );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
} 

error = cam2.SetConfiguration( &config );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}


VideoMode vmode;
FrameRate frate;
error = cam.GetVideoModeAndFrameRate( &vmode, &frate );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
} 
frate = FRAMERATE_30;
vmode=VIDEOMODE_640x480Y8;

    // Set the camera configuration
error = cam.SetVideoModeAndFrameRate( vmode, frate );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

error = cam2.SetVideoModeAndFrameRate( vmode, frate );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}


int gamma=1;
n.getParam("gamma", gamma);
int exposure=40;
n.getParam("exposure", exposure);
/*int exposure;
n.param("exposure", i, 30);
int exposure;
n.param("exposure", i, 30);
*/
Property prop;

prop.type=GAMMA;
cam.GetProperty(&prop);
prop.onOff=true;
prop.valueA=1;
ROS_INFO("valueA : %i, valueB :%i",prop.valueA,prop.valueB );

prop.autoManualMode=false;
error=cam.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error=cam2.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

prop.type=AUTO_EXPOSURE;
cam.GetProperty(&prop);
prop.onOff=false;
ROS_INFO("valueA : %i, valueB :%i",prop.valueA,prop.valueB );
prop.autoManualMode=false;
prop.valueA=50;
prop.valueB=0;
error=cam.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error=cam2.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
} 
/*

PropertyInfo pInfo;
pInfo.type=AUTO_EXPOSURE;
error=cam.GetPropertyInfo(&pInfo);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
ROS_INFO("max : %i, min :%i",pInfo.max,pInfo.min );
*/

prop.type=SHUTTER;
cam.GetProperty(&prop);
ROS_INFO("valueA : %i, valueB :%i",prop.valueA,prop.valueB );
prop.onOff=true;
prop.autoManualMode=false;
prop.absValue=12.0;
prop.absControl=true;
prop.valueA=12;
prop.valueB=0;
error=cam.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error=cam2.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}






/*
prop.type=AUTO_EXPOSURE;
cam.GetProperty(&prop);
prop.autoManualMode=false;
cam.SetProperty(&prop);
cam2.SetProperty(&prop);

prop.type=GAIN;
cam.GetProperty(&prop);
//prop.autoManualMode=true;
cam.SetProperty(&prop);
cam2.SetProperty(&prop);

prop.type=BRIGHTNESS;
cam.GetProperty(&prop);
//prop.autoManualMode=true;
cam.SetProperty(&prop);
cam2.SetProperty(&prop);
*/
/*
prop.type=SHUTTER;
cam2.GetProperty(&prop);
//prop.autoManualMode=true;
cam2.SetProperty(&prop);

prop.type=AUTO_EXPOSURE;
cam2.GetProperty(&prop);
prop.autoManualMode=false;
cam2.SetProperty(&prop);

prop.type=GAIN;
cam2.GetProperty(&prop);
//prop.autoManualMode=true;
cam2.SetProperty(&prop);

prop.type=BRIGHTNESS;
cam2.GetProperty(&prop);
//prop.autoManualMode=true;
cam2.SetProperty(&prop);*/




    // Camera is ready, start capturing images
error = cam.StartCapture();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

error = cam2.StartCapture();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
/* external trigger
const unsigned int gpio1 = 0x1120;  

    // Set pin 1 to be an output 
unsigned int pin = 1;
unsigned int direction = 1; // 1 is output
error = cam.SetGPIOPinDirection(pin,direction,false);
if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
*/

printf( "Trigger the camera by sending a trigger pulse to GPIO%d.\n", 
    triggerMode.source );


ros::Rate loop_rate(10);


// declaration des variables
sensor_msgs::CameraInfo ci;
ros::Time now,now2;

/*
unsigned int value=0x80080001;
error = cam.WriteRegister( gpio1, value, false );
if ( error != PGRERROR_OK )
{
    PrintError( error );
    return -1;
}
usleep(100);
*/
int count;
while(ros::ok())
{
  
  
        /*
        const unsigned int gpio0 = 0x1110;
        unsigned int value = 0;
        error = cam.ReadRegister( gpio0, &value );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
        printf( "Value of GPIO Pin 0  0x%x.\n", value );*/
if(count >10){count=1;}
    else{count++;}


prop.type=GAIN;
cam.GetProperty(&prop);
ROS_INFO("valueA : %i, valueB :%i, count :%i",prop.valueA,prop.valueB,count);
prop.onOff=true;
prop.autoManualMode=false;
prop.absControl=true;
prop.absValue=count;
prop.valueA=count;
prop.valueB=0;
error=cam.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error=cam2.SetProperty(&prop);
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}



	sensor_msgs::Image image;
	sensor_msgs::Image image2;

	PollForTriggerReady( &cam );
        PollForTriggerReady( &cam2 );
	
	now = ros::Time::now();
        bool retVal = FireSoftwareTrigger( &cam );
        if ( !retVal )
        {
            printf("\nError firing software trigger!\n");
            return -1;
        }
	now2 = ros::Time::now();
        bool retVal2 = FireSoftwareTrigger( &cam2 );
        if ( !retVal )
        {
            printf("\nError firing software trigger!\n");
            return -1;
        }

        /*
	value=0x80080000;
        error = cam.WriteRegister( gpio1, value, false );
        if ( error != PGRERROR_OK )
        {
         PrintError( error );
         return -1;
        }
        

        value=0x80080001;
        error = cam.WriteRegister( gpio1, value, false );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }*/
        if (grabImage(image,&cam,frame_id,now))
         {
            printf( "Image grabbed \n" );
            ci=cinfo.getCameraInfo();
            ci.header.stamp = now;
            ci.header.frame_id = frame_id;
            it_pub.publish (image);          //publish the image
            pubinfo.publish(ci);             //publish the info
        }
        else{
        }
	/*
        value=0x80080001;
        error = cam.WriteRegister( gpio1, value, false );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }*/
        if (grabImage(image2,&cam2,frame_id2,now2))
        {
            printf( "Image grabbed \n" );
            ci=cinfo2.getCameraInfo();
            ci.header.stamp = now2;
            ci.header.frame_id = frame_id2;
            it_pub2.publish (image2);          //publish the image
            pubinfo2.publish(ci);             //publish the info
        }
        else{
        }
	printf("delay ms: %f \n",(now2.toSec()-now.toSec())*1000);
        ros::spinOnce();
        loop_rate.sleep();
    
    
}

/*
value=0x80080000;
error = cam.WriteRegister( gpio1, value, false );
if ( error != PGRERROR_OK )
{
    PrintError( error );
    return -1;
}
*/

    // Turn trigger mode off.
triggerMode.onOff = false;    
error = cam.SetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error = cam2.SetTriggerMode( &triggerMode );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

printf( "\nFinished grabbing images\n" );

    // Stop capturing images
error = cam.StopCapture();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}    
error = cam2.StopCapture();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
} 

    // Disconnect the camera
error = cam.Disconnect();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}
error = cam2.Disconnect();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

printf( "Done! Press Enter to exit...\n" );
getchar();

return 0;
}
