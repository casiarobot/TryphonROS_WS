#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>


//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "i2c-dev.h"
#include "motors.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"
#include "cube.h"       // Cube geometry, inertia, etc.
#include "robocomm.h"

//#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
                BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0

ROBOCOMM robovero;

int file;
int mode;
//float incx,incy,incz;

/* typedef struct
                {
                double Fx;                              // The Force in x-Direction
                double Fy;                              // The Force in y-Direction
                double Fz;                              // The Force in z-Direction
                double Tx;                              // The Torque around x-Direction
                double Ty;                              // The Torque around y-Direction
                double Tz;                              // The Torque around z-Direction
                }
                t_forces;


t_forces Fcontrol, Fmanual,Foff, Fdyn;
*/

geometry_msgs::Wrench  Fcontrol,Fps3, Fmanual,Foff, Fdyn;

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
//          printf("NEG:info1=%i,arg=%d\n",info1,argument);
        }
        /*if (ioctl(file, I2C_SLAVE, adresse >> 1) >= 0)
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
 */
ret=TRUE; 
robovero.setMotor(adresse>>1,argument,info1);
        return(ret);
}
 

void forcecallback (const geometry_msgs::Wrench Force)
{

ROS_INFO("Ca marche (control): %f,%f,%f,%f,%f,%f",Force.force.x,Force.force.y,Force.force.z,Force.torque.x,Force.torque.y,Force.torque.z);

Fcontrol.force.x=Force.force.x;
Fcontrol.force.y=Force.force.y;
Fcontrol.force.z=Force.force.z;
Fcontrol.torque.x=Force.torque.x;
Fcontrol.torque.y=Force.torque.y;
Fcontrol.torque.z=Force.torque.z;
}


void ps3callback (const geometry_msgs::Wrench  ps3_comm)
{ 

//float k=0.01;
//if((incx<150 && ps3_comm.position.x>0)||(incx>-150 && ps3_comm.position.x<0)){incx=incx+ps3_comm.position.x;}
//if((incy<150 && ps3_comm.position.y>0)||(incy>-150 && ps3_comm.position.y<0)){incy=incy+ps3_comm.position.y;}
//if((incz<150 && ps3_comm.position.z>0)||(incz>-150 && ps3_comm.position.z<0)){incz=incz+ps3_comm.position.z;}

//if(ps3_comm.orientation.w==-1){incx=0;incy=0;}
//if(ps3_comm.orientation.w==1){incx=0;incy=0;incz=0;}

Fps3.force.x=ps3_comm.force.x;
Fps3.force.y=ps3_comm.force.y;
Fps3.force.z=ps3_comm.force.z;
Fps3.torque.x=ps3_comm.torque.x;
Fps3.torque.y=ps3_comm.torque.y;
Fps3.torque.z=ps3_comm.torque.z;
ROS_INFO("Fps3 : %f",Fps3.force.x);
}


/*void forcecallback2 (const geometry_msgs::Wrench ForceMan)
{

ROS_INFO("Ca marche (manual): %f,%f,%f,%f,%f,%f",ForceMan.force.x,ForceMan.force.y,ForceMan.force.z,ForceMan.torque.x,ForceMan.torque.y,ForceMan.torque.z);

Fmanual.force.x=ForceMan.force.x;
Fmanual.force.y=ForceMan.force.y;
Fmanual.force.z=ForceMan.force.z;
Fmanual.torque.x=ForceMan.torque.x;
Fmanual.torque.y=ForceMan.torque.y;
Fmanual.torque.z=ForceMan.torque.z;
} */


/* void forcecallback3 (const geometry_msgs::Wrench ForceDyn)
{

ROS_INFO("Ca marche (ForceDyn): %f,%f,%f,%f,%f,%f",ForceDyn.force.x,ForceDyn.force.y,ForceDyn.force.z,ForceDyn.torque.x,ForceDyn.torque.y,ForceDyn.torque.z);

Fdyn.force.x=ForceDyn.force.x;
Fdyn.force.y=ForceDyn.force.y;
Fdyn.force.z=ForceDyn.force.z;
Fdyn.torque.x=ForceDyn.torque.x;
Fdyn.torque.y=ForceDyn.torque.y;
Fdyn.torque.z=ForceDyn.torque.z;
} */

void forces2motors(geometry_msgs::Wrench F)
{
double L=(double)CUBE_LENGTH;
int etat = INTERACTION;
 
//...Local variables.............................................................
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
 
// The following variables need to be ajusted such that force.x=1 produces about 1N of thrust
float Mxr=MOTOR_GAIN;                                                                   // Motor gain for Motor x right
float Mxl=MOTOR_GAIN;                                                                   // Motor gain for Motor x left
float Myf=MOTOR_GAIN;                                                                   // Motor gain for Motor y front
float Myb=MOTOR_GAIN;                                                                   // Motor gain for Motor y back
float Mzfl=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z front left
float Mzfr=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z front right
float Mzbl=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z back left
float Mzbr=MOTOR_GAIN;                                                                  // Motor gain Variable for Motor z back right
 
//...Calculations......................................................................

if(CFG_6DDL) {
        Cxr=Mxr*(float)(F.force.x/4+F.torque.z/(4*L)-F.torque.y/(4*L));                  // The Command for Motor x right/left With 12M
        Cxl=Mxl*(float)(F.force.x/4-F.torque.z/(4*L)-F.torque.y/(4*L));
        Cxtr=Mxr*(float)(F.force.x/4+F.torque.z/(4*L)+F.torque.y/(4*L));                 // The Command for TOP Motor x right/left With 12M
        Cxtl=Mxl*(float)(F.force.x/4-F.torque.z/(4*L)+F.torque.y/(4*L));
} else {
        Cxr=Mxr*(float)(F.force.x/2+F.torque.z/(2*L));                             // Idem With 8M
        Cxl=Mxl*(float)(F.force.x/2-F.torque.z/(2*L));
}
if(CFG_6DDL) {
        Cyf=Myf*(float)(F.force.y/4+F.torque.z/(4*L)+F.torque.x/(4*L));                          // The Command for Motor y front/back With 12M
        Cyb=Myb*(float)(F.force.y/4-F.torque.z/(4*L)+F.torque.x/(4*L));
        Cytf=Myf*(float)(F.force.y/4+F.torque.z/(4*L)-F.torque.x/(4*L));                         // The Command for TOP Motor y front/back With 12M
        Cytb=Myb*(float)(F.force.y/4-F.torque.z/(4*L)-F.torque.x/(4*L));
} else {
        Cyf=Myf*(float)(F.force.y/2+F.torque.z/(2*L));                             // The Command for Motor y front/back With 12M
        Cyb=Myb*(float)(F.force.y/2-F.torque.z/(2*L));
}
if(CFG_6DDL) {
        Czfl=Mzfl*(float)(F.force.z/4+(F.torque.x-F.torque.y)/(4*L));                            // The Command for Motor z front left
        Czfr=Mzfr*(float)(F.force.z/4+(-F.torque.x-F.torque.y)/(4*L));                           // The Command for Motor z front right
        Czbl=Mzbl*(float)(F.force.z/4+(F.torque.x+F.torque.y)/(4*L));                            // The Command for Motor z back left
        Czbr=Mzbr*(float)(F.force.z/4+(-F.torque.x+F.torque.y)/(4*L));                           // The Command for Motor z back right
} else {
        Czfl=Mzfl*(float)(F.force.z/4+(F.torque.x-F.torque.y)/(2*L));                            // The Command for Motor z front left
        Czfr=Mzfr*(float)(F.force.z/4+(-F.torque.x-F.torque.y)/(2*L));                           // The Command for Motor z front right
        Czbl=Mzbl*(float)(F.force.z/4+(F.torque.x+F.torque.y)/(2*L));                            // The Command for Motor z back left
        Czbr=Mzbr*(float)(F.force.z/4+(-F.torque.x+F.torque.y)/(2*L));                           // The Command for Motor z back right
}
//int collision = safetorque.yv2();
//...Send commands......................................................................
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





 //////////////////////////////////////////////////////////////////
// Convert 6DDL Forces to motors commands
////////////////////////////////////////////////////////////////

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
        //  if((ret=ioctl( file , I2C_SLAVE , adr )) >= 0){
             if(robovero.scanI2C(adr)>0)// if (i2c_smbus_write_byte(file, 0x00) >= 0){
                        {
			printf(" 0x%02x", adr*2);
                        motorInst.id = adr;
                        mArray->motors.push_back(motorInst);
                        //printf("Add %i",sArray->sonars[i].id);
                        i++;
                }
         // }
        }
 
        time(&sys_time);
        today = localtime(&sys_time);
        fin = today->tm_sec;
        printf("\nTEMPS DE REACTION:%i",fin-deb);
        printf("\n");
}

void modeinfo(std_msgs::Int32 modei)
{
	mode=modei.data;
}

 int main(int argc, char **argv)
 {
 	ros::init(argc,argv, "thruster");
	robovero.Init();
 	ros::NodeHandle n;

	geometry_msgs::Wrench Ftotal;
	sensors::motor motorInst;
        sensors::motorArray mArray;
	mode=2; //O off; 1 Manual; 2 stabilizing;
	

        printf("Ouverture bus I2C\n");
	ros::Rate loop_rate(20);

	//if ((file = open(DEVICE_I2C, O_RDWR)) < 0)
  	//    perror("Erreur lors de l'ouverture");
        scan_i2c_motors(file, &mArray);

	Foff.force.x=0;
	Foff.force.y=0;
	Foff.force.z=0;
	Foff.torque.x=0;
	Foff.torque.y=0;
	Foff.torque.z=0;

	Fcontrol=Foff;
	Fmanual=Foff;
	Ftotal=Foff;

	ros::Subscriber Command_int= n.subscribe("mode",1,modeinfo);
	ros::Subscriber ps3_control = n.subscribe("ps3_control",1, ps3callback);
	//ros::Subscriber force_vector2= n.subscribe("manualcontrol", 1, forcecallback2);
	ros::Subscriber force_vector= n.subscribe("ilqr_control", 1, forcecallback);
	//ros::Subscriber force_vector3=n.subscribe("dyn_comp_control",1,forcecallback3);
	ros::Publisher force_dyn_comp = n.advertise<geometry_msgs::Wrench>("final_forces",1);

	while(ros::ok())
	{
		ROS_INFO("mode : %d",mode);
		switch(mode){
			case 0: forces2motors(Foff);
				force_dyn_comp.publish(Foff);
				break;
			case 1: forces2motors(Fmanual);
				force_dyn_comp.publish(Fmanual);
				break;
			case 2: Ftotal.force.x=Fcontrol.force.x+Fdyn.force.x;
				Ftotal.force.y=Fcontrol.force.y+Fdyn.force.y;
				Ftotal.force.z=Fcontrol.force.z+Fdyn.force.z;
				Ftotal.torque.x=Fcontrol.torque.x+Fdyn.torque.x;
				Ftotal.torque.y=Fcontrol.torque.y+Fdyn.torque.y;
				Ftotal.torque.z=Fcontrol.torque.z+Fdyn.torque.z;
				force_dyn_comp.publish(Ftotal);
				forces2motors(Ftotal);
				break;
			case 3: forces2motors(Fps3);
				ROS_INFO("Fuck yeah");
				force_dyn_comp.publish(Fps3);
				break;
			default:forces2motors(Foff);
				force_dyn_comp.publish(Foff);
				break;
		}
	       	ros::spinOnce();
		loop_rate.sleep();
	}
        return 0;
}
