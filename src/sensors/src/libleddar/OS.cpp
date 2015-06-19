// *****************************************************************************
// Module..: SerialDemo
//
/// \file    OS.c
///
/// \brief   Function definitions for the OS dependant part of the demo.
///
/// You may have to modify this file if you use a non-standard operating
/// system. Definitions provided are correct for Windows and Fedora Linux.
///
// Copyright (c) 2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#include <stdio.h>

#include <math.h>
#include <wchar.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#include "libleddar/OS.h"

#include "base64.h"
#include "robocomm.h"


/*void TransmitCommand (const char* command, int serial_if)
{
	size_t command_length;

	command_length = strlen(command);

	//printf("Sending %s",command);
	while(command_length > 0)
	{
		int data_written;

		data_written = write(serial_if, command, command_length);
		if(data_written == -1)
		{

			continue;
		}

		command        += data_written;
		command_length -= data_written;
	}
	 struct timespec timeout;

	timeout.tv_sec  = 0;
	timeout.tv_nsec = 1000000;

	nanosleep(&timeout, NULL);
}

unsigned long ReceiveULong (int serial_if)
{
	struct pollfd poll_data;
	unsigned long received_value;
	int           signaled_count;

	poll_data.fd     = serial_if;
	poll_data.events = POLLIN;

	signaled_count = poll(&poll_data, 1, 1000);
	if(signaled_count == 0)
	{
		printf("It has taken an unreasonable amount of time to receive data (ULong)\n");
		return 0;
	}

	received_value = 0;

	while(1)
	{
		int  read_size;
		char read_value;

		read_size = read(serial_if, &read_value, 1);
		if(read_size == -1)
		{
			printf("Error reading from serial port\n");

			//exit(-1);
			struct timespec timeout;
			long            ns_timeout;

			ns_timeout = 8*(1000000000/115200);

			timeout.tv_sec  = ns_timeout/1000000000;
			timeout.tv_nsec = ns_timeout%1000000000;

			nanosleep(&timeout, NULL);

			continue;
		}
		else if(read_size == 0)
		{
			struct timespec timeout;
			long            ns_timeout;

			ns_timeout = 8*(1000000000/115200);

			timeout.tv_sec  = ns_timeout/1000000000;
			timeout.tv_nsec = ns_timeout%1000000000;

			nanosleep(&timeout, NULL);

			continue;
		}

		if(read_value >= '0' && read_value <= '9')
			received_value = received_value*16+(read_value-'0');
		else if(read_value >= 'a' && read_value <= 'f')
			received_value = received_value*16+(read_value-'a'+10);
		else if(read_value >= 'A' && read_value <= 'F')
			received_value = received_value*16+(read_value-'A'+10);
		else if(read_value == '\n')
			break;
		else if(read_value != '\r')
		{
			printf("Unexpected data received\n");
			continue;
			//exit(-1);
		}
	}

	return received_value;
}*/


// *****************************************************************************
// Function: OpenSerialPort
//
/// \brief   Open the serial port with the given name.
///
/// \param   aPortName  Name of the device to open (must be valid for the
///                     platform).
/// \param   aHandle    Pointer to variable that will receive the handle on
///                     output.
///
/// \return  LT_SUCCESS or LT_ERROR.
// *****************************************************************************

LtResult
OpenSerialPort( char *aPortName, LtHandle *aHandle ){
	return LT_SUCCESS;// The port is already open by roboverocom
}

// *****************************************************************************
// Function: CloseSerialPort
//
/// \brief   Close the serial port for the given handle.
///
/// \param   aHandle  Handle returned by OpenSerialPort.
// *****************************************************************************

void
CloseSerialPort( LtHandle aHandle )
{}

// *****************************************************************************
// Function: WriteToSerialPort
//
/// \brief   Write data to the serial port.
///
/// \param   aHandle  Handle returned by OpenSerialPort.
/// \param   aData    Pointer to data to write.
/// \param   aLength  Number of bytes from aData to write.
///
/// \return  The number of bytes actually written or LT_ERROR.
// *****************************************************************************

// We can send to the maximal 255 char, minus 3 end caracters(\r\n\0), we also need 5 char for the "l XX "
// So we have 247 char at the maximun, if we have two "=" sign for the base64 offsets
// we have 244 char each worthing 6 bits, for a grand total of 6*244=1464 bits or 183 bytes
#define MAX_LEN_B64 183
LtResult
WriteToSerialPort(LtByte aAddressI2C, LtHandle aHandle, LtByte *aData, int aLength ){
	//p[array:get(P, Data)]rintf("WRITE: %d\n", aLength);
	char str[256];

	char* outputb64;
	int bLeft = aLength;
	uint bCopy = 0;
	size_t outputLen;
	while(bLeft > 0){
		outputb64 = base64_encode(aData + bCopy,
								  ((bLeft < MAX_LEN_B64) ? bLeft : MAX_LEN_B64),
								  &outputLen);
		sprintf(str,"l %s \r\n", outputb64);
		ROBOCOMM::TransmitCommand(str, aHandle);

		bLeft -= MAX_LEN_B64;
		bCopy += MAX_LEN_B64;

		free(outputb64);
	}

	// send buffer all at once
	sprintf(str,"sendBufferOverI2C %X\r\n", aAddressI2C);
	ROBOCOMM::TransmitCommand(str, aHandle);

	// Acknowledgement-
	ROBOCOMM::ReceiveULong(aHandle);

	return aLength;
}

// *****************************************************************************
// Function: ReadFromSerialPort
//
/// \brief   Read data from the serial port.
///
/// \param   aHandle  Handle returned by OpenSerialPort.
/// \param   aData    Pointer to where to put the data read.
/// \param   aLength  Maximum number of bytes to read (number read may
///                   actually be lower).
///
/// \return  The number of bytes actually read or LT_ERROR.
// *****************************************************************************

LtResult
ReadFromSerialPort(LtByte aAddressI2C, LtHandle aHandle, LtByte *aData, int aMaxLength )
{   
	//printf("READ: %d\n", aMaxLength);
	// TODO fetch more than one byte each time
	int toRead = aMaxLength;
	int bytesReceive = 0;
	char response[256];
	char command_read_fifo[20];
	//char *response;
	char *msg;
	unsigned char *buff_deco;
	size_t len_deco, len_msg, len_response;

	struct timeval lastTimeDataReceive, now, start;
	long int msMax = 5000000 / LT_SERIAL_SPEED; // 20m by default / 10m by experiment
	double elapsedTime;
	gettimeofday(&start, NULL);
	gettimeofday(&lastTimeDataReceive, NULL);

	// The first byte take the longuest to fetch
	// So we use a longer timeout
	lastTimeDataReceive.tv_sec += 1;

	sprintf(command_read_fifo,"r %X\r\n", aAddressI2C);
	int nbrEmptyFifo = 0;
	while(toRead > 0){
		// Send command to receive the content of the fifo in base64
		// First the len of the base64 string is send, follow by the base64 string
		ROBOCOMM::TransmitCommand(command_read_fifo, aHandle);
		len_response = ROBOCOMM::ReceiveStr(aHandle, response);

		// Parse length
		char* arg_ptr = strtok(response, " ");

		if(arg_ptr != NULL){
			len_msg = strtoul(arg_ptr, NULL, 16);
			// Parse message
			msg = strtok(NULL, " ");
		}
		else{
			len_msg = 0;
		}

		// Message empty == fifo empty
		if(len_msg == 0){
			//free(response);

			++nbrEmptyFifo;
			gettimeofday(&now, NULL);
			elapsedTime  = (now.tv_sec  - lastTimeDataReceive.tv_sec)  * 1000.0;   // sec to ms
			elapsedTime += (now.tv_usec - lastTimeDataReceive.tv_usec) / 1000.0;   // us to ms

			// If took too much time to receive data,
			// it means the leddar hang up
			if(elapsedTime > msMax){
				elapsedTime  = (now.tv_sec  - start.tv_sec)  * 1000.0;   // sec to ms
				elapsedTime += (now.tv_usec - start.tv_usec) / 1000.0;   // us to ms
				printf("Time for read: %f len:%d Empty:%d\n", elapsedTime, bytesReceive, nbrEmptyFifo);//*/
				return bytesReceive;
			}
		}
		else{
			// Convert base64 to byte
			buff_deco = base64_decode(msg,
									  len_msg,
									  &len_deco);
			if(buff_deco == NULL){
				printf("Error : Invalid base64 decoding\n");
				return -1;
			}
			// Copy decoding buffer into output buffer
			memcpy(aData + bytesReceive, buff_deco, fmin(len_deco, toRead));

			toRead -= len_deco;
			bytesReceive += len_deco;

			free(buff_deco);
			// Update timer
			gettimeofday(&lastTimeDataReceive, NULL);
		}
	}
	return bytesReceive;
}
