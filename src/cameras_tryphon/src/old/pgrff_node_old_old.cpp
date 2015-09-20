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



int main(int argc, char** argv)
{	
	ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    
	image_transport::ImageTransport it (n);
    image_transport::Publisher pub = it.advertise ("/camera/image", 1);
  
	camera_info_manager::CameraInfoManager info_mgr(n);  ///< Helper class to manage the calibration info for the camera
	
	int width = 640;
	int height = 480;
	int fps = 10;
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

    //const int k_numImages = 600;

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

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
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

	// Power on the camera
	const unsigned int k_cameraPower = 0x610;
	const unsigned int k_powerVal = 0x80000000;
	error  = cam.WriteRegister( k_cameraPower, k_powerVal );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}

	const unsigned int millisecondsToSleep = 100;
	unsigned int regVal = 0;
	unsigned int retries = 10;

	// Wait for camera to complete power-up
	do 
	{
#if defined(WIN32) || defined(WIN64)
		Sleep(millisecondsToSleep);    
#else
		usleep(millisecondsToSleep * 1000);
#endif
		error = cam.ReadRegister(k_cameraPower, &regVal);
		if (error == PGRERROR_TIMEOUT)
		{
			// ignore timeout errors, camera may not be responding to
			// register reads during power-up
		}
		else if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		retries--;
	} while ((regVal & k_powerVal) == 0 && retries > 0);

	// Check for timeout errors after retrying
	if (error == PGRERROR_TIMEOUT)
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
    triggerMode.parameter = 0;

    // A source of 7 means software trigger
    triggerMode.source = 7;
    
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

    // Camera is ready, start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }   
    
	if (!CheckSoftwareTriggerPresence( &cam ))
	{
		printf( "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping application\n");
		return -1;
	}
    
    
    //Image image;
    Image rawImage;
    
    ////////////Get cols and row number ////////////
    //// Check that the trigger is ready
    //PollForTriggerReady( &cam);
    
    ////printf( "Press the Enter key to initiate a software trigger.\n" );
    ////getchar();
    
    //// Fire software trigger
    //bool retVal0 = FireSoftwareTrigger( &cam );
    //if ( !retVal0 )
    //{
		//printf("\nError firing software trigger!\n");
		//return -1;        
    //}
    
    //error = cam.RetrieveBuffer( &rawImage );
        //if (error != PGRERROR_OK)
        //{
            //PrintError( error );
            //return -1;
        //}
    
    //ROS_INFO ("Grabbed one image ");
 
    ////Get the column and row of one image
    //unsigned int n_cols = rawImage.GetCols ();
    //unsigned int n_rows = rawImage.GetRows ();
    //ROS_INFO ("Camera width is : %d", n_cols);
    //ROS_INFO ("Camera height is : %d", n_rows);
    /////////////////////////////////////////////////
    
    
    ros::Rate loop_rate(100);
    //bool start=true;
    while(ros::ok())
    {
		
		sensor_msgs::Image::Ptr image(new sensor_msgs::Image);
		// Check that the trigger is ready
		
		PollForTriggerReady( &cam);

		//printf( "Press the Enter key to initiate a software trigger.\n" );
		//getchar();
		ros::Time capture_time = ros::Time::now();
        // Fire software trigger
        bool retVal = FireSoftwareTrigger( &cam );
        if ( !retVal )
        {
			printf("\nError firing software trigger!\n");
			return -1;        
		}
		//if(start){
        // Grab image        
		error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            break;
        }
        
        
		//printf(".\n");
		//printf( "Grabbed image %d\n", imageCount );

        //// Create a converted image
        
		
        Image convertedImage;
        // Convert the raw image
        error = rawImage.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }  
        //// Create a unique filename
        //char filename[512];
        //sprintf( filename, "FlyCapture2Test-%u-%d.pgm", camInfo.serialNumber, imageCount );

        //// Save the image. If a file format is not passed in, then the file
        //// extension is parsed to attempt to determine the file format.
        //error = convertedImage.Save( filename );
        //if (error != PGRERROR_OK)
        //{
            //PrintError( error );
            //return -1;
        //}
        
        
        
        image->height = convertedImage.GetRows();
		image->width = convertedImage.GetCols();
		
		image->step = convertedImage.GetStride();
		image->encoding = sensor_msgs::image_encodings::RGB8;
		image->header.stamp = capture_time;
		image->header.seq = pair_id;
		image->header.frame_id = frame;
		
		int data_size = convertedImage.GetDataSize();
		image->data.resize(data_size);
		memcpy(&image->data[0], convertedImage.GetData(), data_size);
		//start=false;
		//}
		
        
        pub.publish (image);          //publish the message
        loop_rate.sleep();
        
		
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

    // Turn off trigger mode
    triggerMode.onOff = false;
    error = cam.SetTriggerMode( &triggerMode );
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



