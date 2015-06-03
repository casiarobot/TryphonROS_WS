/*
   This robovero sample program is licensed under the simplified BSD license:

   Copyright 2012, Andrew Gottemoller
   All rights reserved.
   
   Embedded in C++ class by David St-Onge, 2014
   - correct carrier bug
   - correct IMU EN pin
   - add servo control
   - add SRF08, CMPS300, BLCTRL drivers

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer:

   Redistributions in binary form must reproduce the above copyright notice, this list
   of conditions and the following disclaimer in the documentation and/or other materials
   provided with the distribution.

   Neither the name Andrew Gottemoller nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written
   permission.
*/
//#ifdef STANDALONE
#include "robocomm.h"
#include "lpc.h"
//#include "includes/ahrs.h"
//#else
//#include "includes/robocomm.h"
//#include "includes/lpc.h"
//#endif

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*
   I don't like this function.  Sometimes the robovero will 'drop' commands
   if you transmit them too fast.  I suspect the serial buffer on the
   robovero is overflowing and dropping data.  The 'fix' is to sleep after
   each command transmission.

   A 1 millisecond sleep feels about right.  Feel free to try and tweak it
   though.

   If you find a better solution, please let me know!
 */

ROBOCOMM::ROBOCOMM(void)
{
  int i;
  robovero_dev = "/dev/ttyACM0";
  
  accel_gains.x() = 1;//0.00376390;
  accel_gains.y() = 1;//0.00376009;
  accel_gains.z() = 1;//0.00349265;
  
  setGYROOffsets(0,0,0);
  
  mag_cal[0] = 0.500; mag_cal[1] = -0.500;
  mag_cal[2] = 0.500; mag_cal[3] = -0.500;
  mag_cal[4] = 0.500; mag_cal[5] = -0.500;
  
  for(i=0;i<16;i++)
    IssonarCon[i]=0;
  for(i=0;i<16;i++)
    IsmotorCon[i]=0;
}

void ROBOCOMM::setGYROOffsets(float _Xoffset, float _Yoffset, float _Zoffset) 
{
    gyro_offsets[0] = _Xoffset;
    gyro_offsets[1] = _Yoffset;
    gyro_offsets[2] = _Zoffset;
}

void ROBOCOMM::GYROzeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) 
{
    double tmpOffsets[] = {0,0,0};

    for (unsigned int i = 0; i < totSamples; i++) 
	{
        usleep(sampleDelayMS*1000);
	imu::Vector<3> g = getIMUdata(0,'g');
        tmpOffsets[0] += g.x();
        tmpOffsets[1] += g.y();
        tmpOffsets[2] += g.z();
    }
    setGYROOffsets(-tmpOffsets[0] / (double)totSamples + 0.5, -tmpOffsets[1] / (double)totSamples + 0.5, -tmpOffsets[2] / (double)totSamples + 0.5);
    printf("Gyro offsets: %2.3f, %2.3f, %2.3f\n",gyro_offsets[0],gyro_offsets[1],gyro_offsets[2]);
}

void ROBOCOMM::WaitForRobovero (void)
{
    struct timespec timeout;

    timeout.tv_sec  = 0;
    timeout.tv_nsec = 1000000;

    nanosleep(&timeout, NULL);
}

/*
   If a command results in data being sent back to the host, we can approximate
   the amount of time it'll take.  The serial link communicates at 115200bps.

   So, the following equation approximates the wait time:

   wait time = (bytes to receive) * (bits per byte) * (nanoseconds per second / 115200)
 */
void ROBOCOMM::WaitForReception (size_t bytes)
{
    struct timespec timeout;
    long            ns_timeout;

    ns_timeout = bytes*8*(1000000000/115200);

    timeout.tv_sec  = ns_timeout/1000000000;
    timeout.tv_nsec = ns_timeout%1000000000;

    nanosleep(&timeout, NULL);
}

/*
   Used to transmit a command to the robovero.  Note the WaitForRobovero at
   the end of this function.  This waits for the robovero to process the
   command prior to sending the next one.  If you find a better solution
   please let me know :-)
 */
void ROBOCOMM::TransmitCommand (const char* command, int serial_if)
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
            if(errno == EAGAIN)
                tcdrain(serial_if);
            else
            {
                printf("Write failure\n");

                exit(-1);
            }

            continue;
        }

        command        += data_written;
        command_length -= data_written;
    }
    WaitForRobovero();
}

/*
   Receive an integer value from the robovero.  The integer value can be up
   to 4 bytes in length, so an unsigned long is the appropriate type.

   The data format expected is:
       XXXXXXXX\r\n

   The \r is actually ignored, so it's optional, but the serial port was
   opened with out any new-line translation and untranslated robovero
   responses will always have a \r before the \n.

   Where X is a hex digit.  Any number of digits prior to the new line is
   valid.
 */
unsigned long ROBOCOMM::ReceiveULong (int serial_if)
{
    struct pollfd poll_data;
    unsigned long received_value;
    int           signaled_count;

    poll_data.fd     = serial_if;
    poll_data.events = POLLIN;

    signaled_count = poll(&poll_data, 1, 1000);
    if(signaled_count == 0)
    {
        printf("It has taken an unreasonable amount of time to receive data\n");

        exit(-1);
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

            exit(-1);
        }
        else if(read_size == 0)
        {
            WaitForReception(1);

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

            exit(-1);
        }
    }

    return received_value;
}

void ROBOCOMM::ReceiveStr (int serial_if)
{
    struct pollfd poll_data;
    unsigned long received_value;
    int           signaled_count;

    poll_data.fd     = serial_if;
    poll_data.events = POLLIN;

    signaled_count = poll(&poll_data, 1, 1000);
    if(signaled_count == 0)
    {
        printf("It has taken an unreasonable amount of time to receive data\n");

        exit(-1);
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

            exit(-1);
        }
        else if(read_size == 0)
        {
            WaitForReception(1);

            continue;
        }

        printf("%c",read_value);
        if(read_value == '\n')
            break;
    }
}

void ROBOCOMM::setservo(int id, int angle)
{
  char str[100];
  int match_value = 600 + (angle*1800/180); //For HS55 sub-servo
  //printf("%i\n",angle);
  sprintf(str,"PWM_MatchUpdate %X 1 %X 0\r\n",LPC_PWM1,match_value);
  TransmitCommand(str, serial_if);
  WaitForReception(512);
}

void ROBOCOMM::scanAllI2C()
{
  char str[100];
  int res[128];
  int i,add;
  
  printf("I2C DEV:");
  for(i=1;i<=128;i++){ //Skip IndicatorLED address...
    sprintf(str,"scanI2C %X\r\n",i);
    TransmitCommand(str, serial_if);
    res[i] = ReceiveULong(serial_if);
    if(res[i])
      printf(" 0x%x",i*2);
  }
  printf("\n");
  
  printf("InitSonar");
  for(i=0;i<16;i++){ //from E0 to FE
    add=(int)(0xE0)/2+i;
    sprintf(str,"initsonar %X 6 A\r\n",add);
    if(res[add]){
      printf(" %x",add*2);
      TransmitCommand(str, serial_if);
      IssonarCon[i]=1;
    }
  }
  printf("\n");
  
  printf("InitMotor");
  for(i=0;i<16;i++){ //from A0 to BE
    add=(int)(0xA0)/2+i;
    sprintf(str,"initbl %x\r\n",add);
    if(res[add]){
      TransmitCommand(str, serial_if);
      printf(" %x",add*2);
      IsmotorCon[i]=1;
    }
  }
  printf("\n");
  //usleep(50000);
}

int ROBOCOMM::scanI2C(int id)
{
  char str[100];
  int res;
  
  sprintf(str,"scanI2C %X\r\n",id);
  TransmitCommand(str, serial_if);
  res = ReceiveULong(serial_if);
  
  return res;
}

void ROBOCOMM::init_moteur(int id){
  char str[100];
  sprintf(str,"initbl %x %x\r\n",id, 46);
  TransmitCommand(str, serial_if);
  //, COMMAND_INIT, 46
}


void ROBOCOMM::setMotor(int id,int speed,int dir){
  char str[100];
  /*if(dir)
    dir=112;
  else
    dir=240;*/
  sprintf(str,"sendblspeed %x %x %x\r\n",id,dir,speed);
  TransmitCommand(str, serial_if);
  //usleep(2000000);
  //TransmitCommand("sendblspeed 58 F0 50\r\n", serial_if);
}

int* ROBOCOMM::getMotorData(int id){
  //char out[100];
  char str[100];
  int rpm,volt,curr,temp;
  int* res = new int[3];
  
  sprintf(str,"getbldata %x\r\n",id);
  TransmitCommand(str, serial_if);
  res[0]=ReceiveULong(serial_if); //RPM
  res[1]=ReceiveULong(serial_if); //VOLT
  res[2]=ReceiveULong(serial_if); //CURR
  //printf("MotorData: %i %i %i %i\n",res[0],res[1],res[2],res[3]);
  return res;
}

void ROBOCOMM::setInd(int id, int com, int r, int g, int b){
  char str[100];
  sprintf(str,"setind %x %x %x %x %x\r\n", id, com, r, g, b);
  TransmitCommand(str, serial_if);
}

void ROBOCOMM::get_dist(){
  char out[100];
  int res=0,i;
  
  sprintf(out,"Dist:");
  for(i=0;i<16;i++){
    if(IssonarCon[i]){
      res=readSonar((int)(0xE0)/2+i);
      sprintf(out,"%s %i",out,res);
    }
  }
  sprintf(out,"%s\n",out);
  printf(out);
}

void ROBOCOMM::test_motor(int vit){  
  char out[100];
  int res=0,i;
  
  sprintf(out,"Launch(15%):");
  for(i=0;i<12;i++){
    if(IsmotorCon[i]){
      setMotor((int)(0xA0)/2+i,vit,112);
      sprintf(out," %i",i);
    }
  }
}

float ROBOCOMM::readCompass(int id){
  char str[100];
  float res=0;
  
  sprintf(str,"getheading\r\n");
  TransmitCommand(str, serial_if);
  //usleep(20000);
  res=(float)ReceiveULong(serial_if)/10;
      
  return res;
}

void ROBOCOMM::init_sonar(int id){
  char str[100];
  sprintf(str,"initsonar %X 6 A\r\n",id);
  TransmitCommand(str, serial_if);
}

int ROBOCOMM::readSonar(int id){
  char str[100];
  int res=0;
  
  sprintf(str,"getsonardist %X\r\n",id);
  TransmitCommand(str, serial_if);
  //usleep(20000);
  res=ReceiveULong(serial_if);
      
  return res;
}

int ROBOCOMM::askSonar(int id){
  char str[100];
  int res=0;
  
  sprintf(str,"asksonardist %X\r\n",id);
  TransmitCommand(str, serial_if);
  //usleep(20000);
  //res=ReceiveULong(serial_if);

  return res;
}

imu::Vector<3> ROBOCOMM::getIMUdata(bool output, char c)
{
    double x_axis;
    double y_axis;
    double z_axis;
    imu::Vector<3> r;
    
    switch(c){
      case 'a':
	  TransmitCommand("readAccel\r\n", serial_if);

	  x_axis = ((double)ReceiveULong(serial_if)-32768.0)/(double)(32768/4);
	  y_axis = ((double)ReceiveULong(serial_if)-32768.0)/(double)(32768/4);
	  z_axis = ((double)ReceiveULong(serial_if)-32768.0)/(double)(32768/4);
	  r = imu::Vector<3>(x_axis,y_axis,z_axis);
	  a[0]=x_axis*accel_gains[0];a[1]=y_axis*accel_gains[1];a[2]=z_axis*accel_gains[2];

	  if(output) printf("Accel:\t%.3f\t%.3f\t%.3f\n", x_axis, y_axis, z_axis);
	  break;
	  
      case 'm':
	  TransmitCommand("readMag\r\n", serial_if);
	  
	  x_axis = ((double)ReceiveULong(serial_if)-4096)/(double)(4096/0x19);
	  y_axis = ((double)ReceiveULong(serial_if)-4096)/(double)(4096/0x19);
	  z_axis = ((double)ReceiveULong(serial_if)-4096)/(double)(4096/0x19);
	  r = imu::Vector<3>(x_axis,y_axis,z_axis);
	  m[0]=x_axis-(mag_cal[0] + mag_cal[1])/2;m[1]=y_axis-(mag_cal[2] + mag_cal[3])/2;m[2]=z_axis-(mag_cal[4] + mag_cal[5])/2;

	  if(output) printf("Mag:\t%.3f\t%.3f\t%.3f\n", x_axis, y_axis, z_axis);
	  break;
	  
      case 'g':
	  TransmitCommand("readGyro\r\n", serial_if);

	  y_axis = -((double)ReceiveULong(serial_if)-32768)/(double)(32768/0xFA);
	  x_axis = ((double)ReceiveULong(serial_if)-32768)/(double)(32768/0xFA);
	  z_axis = ((double)ReceiveULong(serial_if)-32768)/(double)(32768/0xFA);
	  r = imu::Vector<3>(x_axis,y_axis,z_axis);
	  g[0]=x_axis+gyro_offsets[0];g[1]=y_axis+gyro_offsets[1];g[2]=z_axis+gyro_offsets[2];

	  if(output) printf("Gyro:\t%.3f\t%.3f\t%.3f\n", x_axis, y_axis, z_axis);
	  break;
    }
    return r;
}

int ROBOCOMM::Init()
{
    printf("\tOpening serial %s\n",robovero_dev);
    serial_if = open(robovero_dev, O_RDWR|O_NOCTTY|O_NDELAY);
    if(serial_if == -1)
    {
        printf("Couldn't open device file: %s\n", robovero_dev);

        return -1;
    }

    /*
       Configure the serial device to something suitable for the robovero
     */
    tcgetattr(serial_if, &if_mode);

    if_mode.c_cflag     = CS8|CREAD;
    if_mode.c_iflag     = 0;
    if_mode.c_lflag     = 0;
    if_mode.c_oflag     = 0;
    if_mode.c_cc[VMIN]  = 0;
    if_mode.c_cc[VTIME] = 0;

    cfsetispeed(&if_mode, B115200);
    cfsetospeed(&if_mode, B115200);

    error = tcsetattr(serial_if, TCSANOW, &if_mode);
    if(error != 0)
    {
        printf("Error setting serial attributes\n");

        return -1;
    }

    /*
       The first command sent to the robovero must be a newline character.
       This let's the robovero know what sort of line ending to expect
       (\n, or \r\n, etc.)
     */
    TransmitCommand("\r\n", serial_if);

    /*
       promptOff prevents the robovero from sending the [:)] or [:(] after
       each command.
     */
    TransmitCommand("promptOff\r\n", serial_if);

    /*
       resetConfig disables all peripherals on the robovero.
     */
    TransmitCommand("resetConfig\r\n", serial_if);

    /*
       roboveroConfig sets the default robovero configuration.
     */
    TransmitCommand("roboveroConfig\r\n", serial_if);

    /*
       When the robovero starts up and as it's configured it'll write
       to the serial port.  We don't care about any of this output, so
       wait a reasonable period of time and flush it.  The time we
       wait is computed based on an approximate number of bytes we
       expect -- 512.
     */
    WaitForReception(512);

    /*
       Flush the input buffer.
     */
    tcflush(serial_if, TCIFLUSH);

    /*TransmitCommand("heartbeatOff\r\n", serial_if);
    sleep(2);
    TransmitCommand("heartbeatOn\r\n", serial_if);
    */
   
    init_imu();

    /*Set up PWM to output a 1.5ms pulse at 50Hz.
	Set the period to 20000us = 20ms = 50Hz*/
    char str[100];
    sprintf(str,"initMatch 0 %X\r\n", 20000);
    TransmitCommand(str, serial_if);
    sprintf(str,"initMatch 1 %X\r\n", 1500);
    TransmitCommand(str, serial_if);
    sprintf(str,"PWM_ChannelCmd %X 1 1\r\n",LPC_PWM1);
    TransmitCommand(str, serial_if);
    sprintf(str,"PWM_ResetCounter %X\r\n",LPC_PWM1);
    TransmitCommand(str, serial_if);
    sprintf(str,"PWM_CounterCmd %X 1\r\n",LPC_PWM1);
    TransmitCommand(str, serial_if);
    sprintf(str,"PWM_Cmd %X 1\r\n",LPC_PWM1);
    TransmitCommand(str, serial_if);
	
    return 0;
}

void ROBOCOMM::init_imu(){
      /*
      Configure the sensors -- NOTE all parameters are in hex, NOT decimal
    Enable IMU by pulling IMU_EN low modes =     {INPUT:0, OUTPUT:1}
    */
    //GPIO_SetDir(1, (1 << 0),1);//pinMode(P1_0, OUTPUT)
    //GPIO_ClearValue(1, (1 << 0));//digitalWrite(P1_0, 0)
    TransmitCommand("GPIO_SetDir 1 1 1\r\n", serial_if);
    TransmitCommand("GPIO_ClearValue 1 1\r\n", serial_if);
    WaitForReception(512);

    TransmitCommand("configAccel 1 1 1 1 32 4\r\n", serial_if);
    TransmitCommand("configMag 1 0 96 19\r\n", serial_if);
    TransmitCommand("configGyro 1 1 1 1 64 fa\r\n", serial_if);
}

int ROBOCOMM::analog_read(int id){
  char str[100];
  int res=0;
  int start_now = ADC_START_NOW;
  int adc_data_done = ADC_DATA_DONE;
  int enable = 1;

  sprintf(str,"ADC_Init %x %x\r\n",LPC_ADC,200000);
  TransmitCommand(str, serial_if);
  sprintf(str,"ADC_ChannelCmd %x %x %x\r\n",LPC_ADC,id,enable);
  TransmitCommand(str, serial_if);
  sprintf(str,"ADC_StartCmd %x %x\r\n",LPC_ADC,start_now);
  TransmitCommand(str, serial_if);
  sprintf(str,"ADC_ChannelGetStatus %x %x %x\r\n",LPC_ADC,id,adc_data_done);
  TransmitCommand(str, serial_if);
  res=ReceiveULong(serial_if);
  while(!res){
    usleep(10);
    sprintf(str,"ADC_ChannelGetStatus %x %x %x\r\n",LPC_ADC,id,adc_data_done);
    TransmitCommand(str, serial_if);
    res=ReceiveULong(serial_if);
  }

  sprintf(str,"ADC_ChannelGetData %x %x\r\n",LPC_ADC,id);
  TransmitCommand(str, serial_if);
  res=ReceiveULong(serial_if);
  return res;
}

void ROBOCOMM::mag_calibration()
{
	for(int i = 0; i < 6; i++)
		mag_cal[i] = 0;

	for(int i = 0; i < 3000; i++)
	{
		imu::Vector<3> mag = getIMUdata(0,'m');

		if(mag[0] < mag_cal[0])
		    mag_cal[0] = mag[0];
		if(mag[0] > mag_cal[1])
		    mag_cal[1] = mag[0];

		if(mag[1] < mag_cal[2])
		    mag_cal[2] = mag[1];
		if(mag[1] > mag_cal[3])
		    mag_cal[3] = mag[1];

		if(mag[2] < mag_cal[4])
		    mag_cal[4] = mag[2];
		if(mag[2] > mag_cal[5])
		    mag_cal[5] = mag[2];

		usleep(20);
	}

	char magc[100]="mag_cal:";
	int pos=8;
	for(int i = 0; i < 6; i++)
	{
		pos += sprintf(&magc[pos], ", %2.3f", mag_cal[i]);
	}
	pos += sprintf(&magc[pos], "\n");
	printf(magc);
	//usleep(1000);
}

void ROBOCOMM::closeI() {

    /*
       Good practice to put the robovero back into its default configuration and flush the
       input buffer prior to closing the interface
     */
    TransmitCommand("resetConfig\r\n", serial_if);
    TransmitCommand("roboveroConfig\r\n", serial_if);

    WaitForReception(512);

    tcflush(serial_if, TCIFLUSH);

    close(serial_if);
}
