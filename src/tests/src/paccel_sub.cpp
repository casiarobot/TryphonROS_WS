

// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>

//library for Android msg
#include <sensor_msgs/Imu.h>

//libraries for the sonar
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//libraries for the control
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <sstream>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "i2c-dev.h"
#include "motors.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"
#include "cube.h"       // Cube geometry, inertia, etc.

#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
                BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0

typedef struct
                {
                double Fx;                              // The Force in x-Direction
                double Fy;                              // The Force in y-Direction
                double Fz;                              // The Force in z-Direction
                double Tx;                              // The Torque around x-Direction
                double Ty;                              // The Torque around y-Direction
                double Tz;                              // The Torque around z-Direction
                }
                t_forces;

typedef struct
                {
                double x;                              
                double y;                             
                double z;                              
                double rx;                              
                double ry;                             
                double rz;                              
                }
                e_errors;

int file;
int start;


// Forces 
t_forces forces;

// Errors
e_errors errors_n; // errors now

e_errors errors_b; // errors before


//////////////////////////////////////////////////////
// Write to I2C ESC motors
//////////////////////////////////////////////////////
bool ecriture_moteur2(BYTE adresse, BYTE commande, BYTE argument,bool sens)
{
        int inf_forward = VIT_POS;
        int inf_reverse = VIT_NEG;
        int info1 = inf_forward;                // Default info1 value

        long sys_time;
        struct tm *today;

        time(&sys_time);
        today = localtime(&sys_time);
        int debut = today->tm_sec;

        bool    ret = 0;


//      printf("\nVIT:%f",(float)argument);
//      I2C_LOCK();
        if((int)argument >0 && (int)argument> VITESSE_MAX){argument = VITESSE_MAX;}
	if((int)argument <0 && (int)argument< -VITESSE_MAX){argument =  -VITESSE_MAX;}
	 if (abs((double)argument) < (((double)VITESSE_MIN)/2))  //      Commands bellow half the minimal speed are set to zero
        {
                argument = 0;

        }

        if (sens)  //DS : Les ESC prennent une commande sur 0-255 non-sign
        {
                argument = 2*argument;
                info1 = inf_forward;
//       printf("POS:info1=%i,arg=%d\n",info1,argument);
        }
        else 
        {
                //argument = (255 - (double)argument)*2;
                argument = argument*2;
                info1 = inf_reverse;
          printf("NEG:info1=%i,arg=%d\n",info1,argument);
        }
        if (ioctl(file, I2C_SLAVE, adresse >> 1) >= 0)
                        {
                        if (i2c_smbus_write_word_data(file, commande, (argument << 8) | info1) >= 0) {                  // send info1 AND PWM
                                ret = TRUE;
                                //printf("Envoi a %x : %i %i\n", adresse, info1, argument);
                                }
                        }
        else printf("Erreur adressage moteur %x\n", adresse);

        if (!(ret == TRUE))
                 printf("Erreur ecriture moteur %x\n", adresse);
//      I2C_UNLOCK();

        if (!ret)
                perror("ecriture_moteur2()");

        return(ret);
}

//////////////////////////////////////////////////////////////////
// Convert 6DDL Forces to motors commands
////////////////////////////////////////////////////////////////
void forces2motors(t_forces F)
{
double L=(double)CUBE_LENGTH;
int etat = INTERACTION;

/*...Local variables.............................................................*/                                                             
float Cxl=0;                                                                            // Command Variable for Motor x left
float Cxr=0;                                                                            // Command Variable for Motor x right
float Cyf=0;                                                                            // Command Variable for Motor y front
float Cyb=0;                                                                            // Command Variable for Motor y back
float Cxtl=0;                                                                           // Command Variable for Motor x top left
float Cxtr=0;                                                                           // Command Variable for Motor x top right
float Cytf=0;                                                                           // Command Variable for Motor y top front
float Cytb=0;                                                                           // Command Variable for Motor y top back
float Czfl=0;                                                                           // Command Variable for Motor z front left
float Czfr=0;                                                                           // Command Variable for Motor z front right
float Czbl=0;                                                                           // Command Variable for Motor z back left
float Czbr=0;                                                                           // Command Variable for Motor z back right

// The following variables need to be ajusted such that Fx=1 produces about 1N of thrust
float Mxr=MOTOR_GAIN;                                                                   // Motor gain for Motor x right
float Mxl=MOTOR_GAIN;                                                                   // Motor gain for Motor x left
float Myf=MOTOR_GAIN;                                                                   // Motor gain for Motor y front
float Myb=MOTOR_GAIN;                                                                   // Motor gain for Motor y back
float Mzfl=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z front left
float Mzfr=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z front right
float Mzbl=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z back left
float Mzbr=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z back right

/*...Calculations......................................................................*/
if(CFG_6DDL) {
        Cxr=Mxr*(float)(F.Fx/4+F.Tz/(4*L)-F.Ty/(4*L));                  // The Command for Motor x right/left With 12M
        Cxl=Mxl*(float)(F.Fx/4-F.Tz/(4*L)-F.Ty/(4*L));
        Cxtr=Mxr*(float)(F.Fx/4+F.Tz/(4*L)+F.Ty/(4*L));                 // The Command for TOP Motor x right/left With 12M
        Cxtl=Mxl*(float)(F.Fx/4-F.Tz/(4*L)+F.Ty/(4*L));
} else {
        Cxr=Mxr*(float)(F.Fx/2+F.Tz/(2*L));                             // Idem With 8M
        Cxl=Mxl*(float)(F.Fx/2-F.Tz/(2*L));
}
if(CFG_6DDL) {
        Cyf=Myf*(float)(F.Fy/4+F.Tz/(4*L)+F.Tx/(4*L));                          // The Command for Motor y front/back With 12M
        Cyb=Myb*(float)(F.Fy/4-F.Tz/(4*L)+F.Tx/(4*L));
        Cytf=Myf*(float)(F.Fy/4+F.Tz/(4*L)-F.Tx/(4*L));                         // The Command for TOP Motor y front/back With 12M
        Cytb=Myb*(float)(F.Fy/4-F.Tz/(4*L)-F.Tx/(4*L));
} else {
        Cyf=Myf*(float)(F.Fy/2+F.Tz/(2*L));                             // The Command for Motor y front/back With 12M
        Cyb=Myb*(float)(F.Fy/2-F.Tz/(2*L));
}
if(CFG_6DDL) {
        Czfl=Mzfl*(float)(F.Fz/4+(F.Tx-F.Ty)/(4*L));                            // The Command for Motor z front left
        Czfr=Mzfr*(float)(F.Fz/4+(-F.Tx-F.Ty)/(4*L));                           // The Command for Motor z front right
        Czbl=Mzbl*(float)(F.Fz/4+(F.Tx+F.Ty)/(4*L));                            // The Command for Motor z back left
        Czbr=Mzbr*(float)(F.Fz/4+(-F.Tx+F.Ty)/(4*L));                           // The Command for Motor z back right
} else {
        Czfl=Mzfl*(float)(F.Fz/4+(F.Tx-F.Ty)/(2*L));                            // The Command for Motor z front left
        Czfr=Mzfr*(float)(F.Fz/4+(-F.Tx-F.Ty)/(2*L));                           // The Command for Motor z front right
        Czbl=Mzbl*(float)(F.Fz/4+(F.Tx+F.Ty)/(2*L));                            // The Command for Motor z back left
        Czbr=Mzbr*(float)(F.Fz/4+(-F.Tx+F.Ty)/(2*L));                           // The Command for Motor z back right
}
//int collision = safetyv2();
/*...Send commands......................................................................*/
if (Cxl!=0 || Cxr!=0)//(pos_des.x!=0)||(pos_des.theta_z_ref!=0))        // Send command to motors in x Direction
{
      if(Cxl>=0)
	 { ecriture_moteur2(ADR_XLEFT, COMMAND_SEND_RPM, Cxl,true);}
	else{ ecriture_moteur2(ADR_XLEFT, COMMAND_SEND_RPM,-Cxl,false);}                      // Send command to motor x left
       if(Cxr>=0) { ecriture_moteur2(ADR_XRIGHT, COMMAND_SEND_RPM, Cxr,true);}
	else{ecriture_moteur2(ADR_XRIGHT, COMMAND_SEND_RPM,-Cxr,false);}                    // Send command to motor x right
       // ecriture_moteur2(ADR_XTLEFT, COMMAND_SEND_RPM, Cxtl);                   // Send command to motor x left TOP
       // ecriture_moteur2(ADR_XTRIGHT, COMMAND_SEND_RPM, Cxtr);          // Send command to motor x right TOP
        //printf("\nXL:%3.3f,XR:%3.3f",Cxl,Cxr);        
}       
else if (etat!=INTERACTION)
{
        ecriture_moteur2(ADR_XLEFT, COMMAND_SEND_RPM, 0,true);                       // Send command to motor x left
        ecriture_moteur2(ADR_XRIGHT, COMMAND_SEND_RPM, 0,true);
       // ecriture_moteur2(ADR_XTLEFT, COMMAND_SEND_RPM, 0);                      // Send command to motor x left
       // ecriture_moteur2(ADR_XTRIGHT, COMMAND_SEND_RPM, 0);
}
if (Cyb!=0 || Cyf!=0)//(pos_des.y!=0)||(pos_des.theta_z_ref!=0))        // Send command to motors in y Direction
{
        if(Cyb>=0){ecriture_moteur2(ADR_YFRONT, COMMAND_SEND_RPM, Cyb,true);}
	else{ecriture_moteur2(ADR_YFRONT, COMMAND_SEND_RPM,-Cyb,false);}
        if(Cyf>=0){ecriture_moteur2(ADR_YBACK, COMMAND_SEND_RPM, Cyf,true);}
	else{ecriture_moteur2(ADR_YBACK, COMMAND_SEND_RPM,-Cyf,false);}                      // Send command to motor y back
        //ecriture_moteur2(ADR_YTFRONT, COMMAND_SEND_RPM, Cytb);
        //ecriture_moteur2(ADR_YTBACK, COMMAND_SEND_RPM, Cytf);                   // Send command to motor y back
        //printf("\nYF:%3.3f,YB:%3.3f",Cyf,Cyb);
}
else if (etat!=INTERACTION)
{
        ecriture_moteur2(ADR_YFRONT, COMMAND_SEND_RPM, 0,true);              // Send command to motor y front
        ecriture_moteur2(ADR_YBACK, COMMAND_SEND_RPM, 0,true);                       // Send command to motor y back
        //ecriture_moteur2(ADR_YTFRONT, COMMAND_SEND_RPM, 0);             // Send command to motor y front
        //ecriture_moteur2(ADR_YTBACK, COMMAND_SEND_RPM, 0);                      // Send command to motor y back
}
if (Czfl!=0)//pos_des.z!=0)//Send command to motors in z Direction
{
        if(Czfl>=0){ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM, Czfl,true);}
	else{ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM,-Czfl,false);}            // Send command to motor z front left
        if(Czfr>=0){ecriture_moteur2(ADR_ZFRIGHT, COMMAND_SEND_RPM, Czfr,true);}
	else{ecriture_moteur2(ADR_ZFRIGHT, COMMAND_SEND_RPM, -Czfr,false);}  // Send command to motor z front right
        if(Czbl>=0){ecriture_moteur2(ADR_ZBLEFT, COMMAND_SEND_RPM, Czbl,true);}           // Send command to motor z back left
        else{ecriture_moteur2(ADR_ZBLEFT, COMMAND_SEND_RPM, -Czbl,false);} 
	if(Czbr>=0){ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, Czbr,true);}          // Send command to motor z back right
	else{ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, -Czbr,false);} 
        //printf("\nZFL:%3.3f,ZFR:%3.3f",Czfl,Czfr);    
}
else if (etat!=INTERACTION)
{
        ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM, 0,true);              // Send command to motor z front left
        ecriture_moteur2(ADR_ZFRIGHT, COMMAND_SEND_RPM, 0,true);     // Send command to motor z front right
        ecriture_moteur2(ADR_ZBLEFT, COMMAND_SEND_RPM, 0,true);              // Send command to motor z back left
        ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, 0,true);     // Send command to motor z back right                           
}
return;
}

////////////////////////////////////////////////////////////////////
// Scan I2C bus for connected motors
///////////////////////////////////////////////////////////////////
void scan_i2c_motors(int file, sensors::motorArray *mArray)                              //$
{
        int adr, ret=0;
        sensors::motor motorInst;
        int deb, fin, i=0;
        long sys_time;
        struct tm *today;

        time(&sys_time);
        today = localtime(&sys_time);
        deb = today->tm_sec;

        printf("Motors I2C detectes:");
        for (adr=80; adr<95; adr++) { // de E0 a FC
          if((ret=ioctl( file , I2C_SLAVE , adr )) >= 0){
              if (i2c_smbus_write_byte(file, 0x00) >= 0){
                        printf(" 0x%02x", adr*2);
                        motorInst.id = adr;
                        mArray->motors.push_back(motorInst);
                        //printf("Add %i",sArray->sonars[i].id);
                        i++;
                }
          }
        }

        time(&sys_time);
        today = localtime(&sys_time);
        fin = today->tm_sec;
        printf("\nTEMPS DE REACTION:%i",fin-deb);
        printf("\n");
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void poseCallback (const sensor_msgs::Imu::ConstPtr& wii)
{

	float x,y,z,tx,ty;

	//Get the linear acceleration from the accelerometers
 	x = wii->linear_acceleration.y;
	y = wii->linear_acceleration.x;
	z = wii->linear_acceleration.z;

	//tx = atan(y/z)*57;
	//ty = atan(x/z)*57;
	//ROS_INFO("ax: %f, ay: %f, az: %f",x,y,z);
	//ROS_INFO("tx: %f, ty: %f",tx,ty);
	forces.Fx=1*(atan(y/z)*5); //35 max
	forces.Fy=1*(atan(x/z)*5);
}

void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
int ds1=0;
int ds2=0;
int dwant=75;
  for (int i=0; i<msg->sonars.size(); ++i)
  {
   const sensors::sonar &sonar = msg->sonars[i];
   // ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
   // 	               ", D1: " << sonar.distance[1]);
   if (sonar.id == 126){
	for (int j=0;j<10;j++)
		{ds1=ds1+sonar.distance[j];}
	}
   if (sonar.id == 120){
	for (int j=0;j<10;j++)
                {ds2=ds2+sonar.distance[j];}
        }
  }

errors_n.x=dwant-(ds1+ds2)/20;
forces.Fz= forces.Fz + 0.07*(errors_n.x-errors_b.x) + 0.07*0/0.1*errors_n.x;
errors_b.x=errors_n.x;

if (abs(forces.Fz)>35){
	if(forces.Fz<0){
		forces.Fz=-35;}
	else {forces.Fz=35;}
}
clock_t begin;
clock_t end;
double elapsed_ms;

if (start>0){end = clock();
elapsed_ms = double(end - begin) / CLOCKS_PER_SEC*1000;}
ROS_INFO("time (ms): %f",elapsed_ms);
begin = clock();
}

//Main
int main(int argc, char **argv)
{
	start=0;

	forces.Fx=0;
	forces.Fy=0;
	forces.Fz=0;
	forces.Tx=0;
	forces.Ty=0;
	forces.Tz=0;

	errors_n.x=0;
	errors_n.y=0;
	errors_n.z=0;
	errors_n.rx=0;
	errors_n.ry=0;
	errors_n.rz=0;

	errors_b.x=0;
	errors_b.y=0;
	errors_b.z=0;
	errors_b.rx=0;
	errors_b.ry=0;
	errors_b.rz=0;	

	sensors::motor motorInst;
        sensors::motorArray mArray;

        printf("Ouverture bus I2C\n");
                if ((file = open(DEVICE_I2C, O_RDWR)) < 0)
                        perror("Erreur lors de l'ouverture");

        scan_i2c_motors(file, &mArray);

	ros::init(argc, argv, "accel_sub");

	ros::NodeHandle node;

	ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	ros::Subscriber subS = node.subscribe("sonars", 1, subSonar);
	while (ros::ok())
	{
		ROS_INFO("fx: %f, fy: %f, fz: %f",forces.Fx,forces.Fy,forces.Fz);
		forces2motors(forces);
		//usleep(100);
		ros::spinOnce();
		start=1;
	}

	return 0;
}
