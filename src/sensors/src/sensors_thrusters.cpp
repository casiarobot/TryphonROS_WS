#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>


#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/sonarArray.h"
#include "sensors/props_command.h"

#include "std_msgs/Int32.h"

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "motors.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"
#include "robocomm.h"

#include "capteurs.h"

std_msgs::Bool magnet;
int file;
double  Fcontrol[12];
double  msgTime=0;

bool ecriture_moteur2(unsigned int adresse, unsigned int commande, unsigned int argument,bool sens)
{
	int inf_forward = 112;
	int inf_reverse = 240;
	int info1 = inf_forward;                // Default info1 value
	bool    ret = 0;

	if((int)argument >0 && (int)argument> 255){argument = 255;}
	if((int)argument <0 && (int)argument< -255){argument =  -255;}
	if (sens)  info1 = inf_forward;
	else  info1 = inf_reverse;

	ret=1;
	robovero.setMotor(adresse>>1,argument,info1);
	return(ret);
}


void controlcallback (const sensors::props_command commandArray)
{

	//ROS_INFO("Ca marche (control): %f,%f,%f,%f,%f,%f",Force.force.x,Force.force.y,Force.force.z,Force.torque.x,Force.torque.y,Force.torque.z);
	for(int i=0;i<12;i++)
	{
		Fcontrol[i]=commandArray.commands[i];
	}
	msgTime=ros::Time::now().toSec();

}

void magnetcallback (const std_msgs::Bool on)
{
	ROS_INFO("Ca marche ");
	if(magnet.data!=on.data){
		robovero.swRelay((int) 0x4c/2);
		ROS_INFO("Switch relay : %i",on.data);
		magnet=on;
	}

}


bool forceDir (double f)
{
	if(f<0){return false;}
	else {return true;}
}

bool isConMotor(sensors::motorArray mArray, unsigned int id)
{
	id=id>>1;
	for(int i=0; i<mArray.motors.size(); i++)
	{
		if(mArray.motors[i].id==id) return true;
	}
	return false;
}

void forces2motors(double commands[12], sensors::motorArray mArray)
{
	double Cxl=commands[0];
	double Cxr=commands[1];
	double Cyf=commands[2];
	double Cyb=commands[3];
	double Czfr=commands[4];
	double Czbr=commands[5];
	double Czbl=commands[6];
	double Czfl=commands[7];
	double Ctxl=commands[8];
	double Ctxr=commands[9];
	double Ctyf=commands[10];
	double Ctyb=commands[11];

	if(isConMotor(mArray,ADR_XLEFT)) ecriture_moteur2(ADR_XLEFT,   COMMAND_SEND_RPM, fabs(Cxl),  forceDir(Cxl));                       // Send command to motor x left
	if(isConMotor(mArray,ADR_XRIGHT)) ecriture_moteur2(ADR_XRIGHT,  COMMAND_SEND_RPM, fabs(Cxr),  forceDir(Cxr));

	if(isConMotor(mArray,ADR_YFRONT)) ecriture_moteur2(ADR_YFRONT,  COMMAND_SEND_RPM, fabs(Cyf),  forceDir(Cyf));              // Send command to motor y front
	if(isConMotor(mArray,ADR_YBACK)) ecriture_moteur2(ADR_YBACK,   COMMAND_SEND_RPM, fabs(Cyb),  forceDir(Cyb));                       // Send command to motor y bac

	if(isConMotor(mArray,ADR_XTLEFT)) ecriture_moteur2(ADR_XTLEFT,   COMMAND_SEND_RPM, fabs(Ctxl),  forceDir(Ctxl));                       // Send command to motor x left
	if(isConMotor(mArray,ADR_XTRIGHT)) ecriture_moteur2(ADR_XTRIGHT,  COMMAND_SEND_RPM, fabs(Ctxr),  forceDir(Ctxr));

	if(isConMotor(mArray,ADR_YTFRONT)) ecriture_moteur2(ADR_YTFRONT,  COMMAND_SEND_RPM, fabs(Ctyf),  forceDir(Ctyf));              // Send command to motor y front
	if(isConMotor(mArray,ADR_YTBACK)) ecriture_moteur2(ADR_YTBACK,   COMMAND_SEND_RPM, fabs(Ctyb),  forceDir(Ctyb));                       // Send command to motor y bac

	if(isConMotor(mArray,ADR_ZFRIGHT)) ecriture_moteur2(ADR_ZFRIGHT,  COMMAND_SEND_RPM, fabs(Czfr), forceDir(Czfr));              // Send command to motor z front left
	if(isConMotor(mArray,ADR_ZBRIGHT)) ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, fabs(Czbr), forceDir(Czbr));     // Send command to motor z front right
	if(isConMotor(mArray,ADR_ZBLEFT)) ecriture_moteur2(ADR_ZBLEFT,  COMMAND_SEND_RPM, fabs(Czbl), forceDir(Czbl));              // Send command to motor z back left
	if(isConMotor(mArray,ADR_ZFLEFT)) ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM, fabs(Czfl), forceDir(Czfl));     // Send command to motor z back right


	return;
}





void scan_i2c_motors(int file, sensors::motorArray *mArray)                              //$
{
	int adr, ret=0;
	sensors::motor motorInst;

	int i=0;


	printf("Motors I2C detectes:");
	for (adr = 0xA0 / 2; adr < 0xC0 / 2; adr++) { // de A0 a BE
		if(robovero.scanI2C(adr) > 0){
			printf(" 0x%02x", adr * 2);
			motorInst.id = adr;
			mArray->motors.push_back(motorInst);
			i++;
		}

	}
	printf("\n");

}

void getimudata(sensors::imuros *imuptr ){

	imu::Vector<3> a;
	imu::Vector<3> g;
	imu::Vector<3> m;

	a=robovero.getIMUdata(0,'a');//0 for no print
	g=robovero.getIMUdata(0,'g');
	m=robovero.getIMUdata(0,'m');

	imuptr->accel[0]=a[0];
	imuptr->accel[1]=a[1];
	imuptr->accel[2]=a[2];
	imuptr->gyro[0]=g[0];
	imuptr->gyro[1]=g[1];
	imuptr->gyro[2]=g[2];
	imuptr->magn[0]=m[0];
	imuptr->magn[1]=m[1];
	imuptr->magn[2]=m[2];

}




void init_sonar(sensors::sonarArray *sArray)
{

	int debut;
	long sys_time;
	struct tm *today;
	time(&sys_time);
	today = localtime(&sys_time);
	for(int i = 0; i < sArray->sonars.size(); ++i){
		robovero.init_sonar(sArray->sonars[i].id);
		usleep(25);
	}
}


void askdistance(sensors::sonarArray *sArray){
	for(int i=0; i<sArray->sonars.size(); ++i){
		robovero.askSonar(sArray->sonars[i].id);}
}



void getdistance(sensors::sonarArray *sArray){
	short distance;

	for(int i=0; i<sArray->sonars.size(); ++i){
		distance = robovero.readSonar(sArray->sonars[i].id);
		if ((distance <= 0) ||(distance> DISTANCE_MAX))
		    distance = DISTANCE_MAX;

		sArray->sonars[i].distance = distance;
	}
}

void getrz(sensors::compass *boussole)
{
	int     high, low, bearing;
	bearing=robovero.readCompass(boussole->id);

	if ((boussole->rz[0]-(float)bearing) < -180)
		boussole->head_init=boussole->head_init+360;
	else if ((boussole->rz[0]-(float)bearing) > 180)
		boussole->head_init=boussole->head_init-360;

	if ((float)bearing-boussole->head_init > 180)
		boussole->head_init=boussole->head_init+360;
	else if ((float)bearing-boussole->head_init < -180)
		boussole->head_init=boussole->head_init-360;

	boussole->rz[0] = (float)bearing;
}


void getmotor_data( sensors::motorArray *mArray){

	int data[3];

	for(int i=0; i < mArray->motors.size(); ++i){
		robovero.getMotorData(mArray->motors[i].id, data);
		mArray->motors[i].rpm = data[0];
		mArray->motors[i].volt = data[1];
		mArray->motors[i].curr = data[2];
	}
}



void scan_i2c_sensors(int file, sensors::sonarArray *sArray, sensors::compass *comp)
{
	int adr, ret=0;
	sensors::sonar sonarInst;
	int i=0;

	printf("Sonars I2C detectes:");
	for (adr=112; adr<128; adr++) {// de E0 a FC
		if(robovero.scanI2C(adr) > 0){
			printf(" 0x%02x", adr*2);
			sonarInst.id = adr;
			sArray->sonars.push_back(sonarInst);
			i++;
		}
	}
	printf("\n");
	adr=BOUSSOLE>>1; //C0
	if(robovero.scanI2C(adr)>0){
		printf("Compass at 0x%02x", 96*2);
		comp->id=adr;
		getrz(comp);
		comp->head_init=comp->rz[0];
	}
}





