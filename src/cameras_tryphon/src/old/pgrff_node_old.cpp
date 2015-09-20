//Modified by Pierre-Yves Brèches 
//
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

#include "stdafx.h"
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

bool grabImage(sensor_msgs::Image &image, Camera* cam, std::string frame_id){
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
        /*metadata = rawImage.GetMetadata();
// Set header timestamp as embedded for now
        TimeStamp embeddedTime = rawImage.GetTimeStamp();

        image.header.stamp.sec = embeddedTime.seconds;
        image.header.stamp.nsec = 1000*embeddedTime.microSeconds;
// Get camera info to check if color or black and white chameleon and check the bits per pixel.
        CameraInfo cInfo;

// Set the image encoding
        std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
        fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
        image.header.frame_id = frame_id;*/
        return true;
}







int main(int argc, char** argv)
{	
	ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it (n);
    image_transport::Publisher pub = it.advertise ("/camera/image", 1);
    
	camera_info_manager::CameraInfoManager info_mgr(n);  ///< Helper class to manage the calibration info for the camera
	ros::Publisher pubimg = n.advertise<sensor_msgs::Image>("imagecool", 1000);

	int width = 640;
	int height = 480;
	int fps = 60;
	int skip_frames=0;
	std::string frame = "camera";
	bool rotate = false;
	unsigned int pair_id = 0;
	/* set up information manager */
	std::string url;
	n.getParam("camera_info_url", url);
	info_mgr.loadCameraInfo(url);
	/* pull other configuration */
	//n.getParam("serial", serial);
	n.getParam("fps", fps);
	n.getParam("skip_frames", skip_frames);
	n.getParam("width", width);
	n.getParam("height", height);
	n.getParam("frame_id", frame);
	
	
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

    PGRGuid guid;
    error = busMgr.GetCameraFromSerialNumber(serialmaster, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    Camera cam;

    // Connect to a camera
    error = cam.Connect(&guid);
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

    // Get the camera information
CameraInfo camInfo;
error = cam.GetCameraInfo(&camInfo);
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
//triggerMode.parameter = 0;

    // Triggering the camera externally using source 0.
triggerMode.source = 0;

error = cam.SetTriggerMode( &triggerMode );
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


VideoMode vmode;
FrameRate frate;
error = cam.GetVideoModeAndFrameRate( &vmode, &frate );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
} 
frate = FRAMERATE_60;
vmode = VIDEOMODE_640x480Y8;

    // Set the camera configuration
error = cam.SetVideoModeAndFrameRate( vmode, frate );
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

    // Camera is ready, start capturing images
error = cam.StartCapture();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

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


printf( "Trigger the camera by sending a trigger pulse to GPIO%d.\n", 
    triggerMode.source );


ros::Rate loop_rate(60);

sensor_msgs::Image image;
std::string frame_id="/camera";

unsigned int value=0x80080001;
error = cam.WriteRegister( gpio1, value, false );
if ( error != PGRERROR_OK )
{
    PrintError( error );
    return -1;
}
usleep(100);
double s1,s2,s3,s4;

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
		//retVal = PollForTriggerReady( &cam );

	s1=ros::Time::now().toSec();

        value=0x80080000;
        error = cam.WriteRegister( gpio1, value, false );
        if ( error != PGRERROR_OK )
        {
         PrintError( error );
         return -1;
        }
	s2=ros::Time::now().toSec();

        value=0x80080001;
        error = cam.WriteRegister( gpio1, value, false );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
        s3=ros::Time::now().toSec();

        

        if (grabImage(image,&cam,frame_id))
        {
            //printf( "Image grabbed \n" );
            //pub.publish (image);          //publish the message
        }
        else{
        }
        //loop_rate.sleep();
	s4=ros::Time::now().toSec();
	printf("etape 1: %f; 2: %f; 3: %f \n",s1-s2,s2-s3,s3-s4);

}

value=0x80080000;
error = cam.WriteRegister( gpio1, value, false );
if ( error != PGRERROR_OK )
{
    PrintError( error );
    return -1;
}


    // Turn trigger mode off.
triggerMode.onOff = false;    
error = cam.SetTriggerMode( &triggerMode );
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


    // Disconnect the camera
error = cam.Disconnect();
if (error != PGRERROR_OK)
{
    PrintError( error );
    return -1;
}

printf( "Done! Press Enter to exit...\n" );
getchar();

return 0;
}



