#include "ros/ros.h"
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
#include "cube.h"	// Cube geometry, inertia, etc.

#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
                BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION	0

typedef struct
		{
		double Fx;				// The Force in x-Direction
		double Fy;				// The Force in y-Direction
		double Fz;				// The Force in z-Direction
		double Tx;				// The Torque around x-Direction
		double Ty;				// The Torque around y-Direction
		double Tz;				// The Torque around z-Direction
		}
		t_forces;

int file;

//////////////////////////////////////////////////////
// Write to I2C ESC motors
//////////////////////////////////////////////////////
bool ecriture_moteur2(BYTE adresse, BYTE commande, BYTE argument)
{
	int inf_forward = VIT_POS;
	int inf_reverse = VIT_NEG;
	int info1 = inf_forward;		// Default info1 value

	long sys_time;
        struct tm *today;

        time(&sys_time);
        today = localtime(&sys_time);
        int debut = today->tm_sec;

	bool	ret = 0;

//	printf("\nVIT:%f",(float)argument);
//	I2C_LOCK();
	if (abs((double)argument) < (((double)VITESSE_MIN)/2))  //	Commands bellow half the minimal speed are set to zero
	{
		argument = 0;	
	}

	if ((double)argument < 128)  //DS : Les ESC prennent une commande sur 0-255 non-sign
	{
		argument = 2*argument;
		info1 = inf_forward;
//       printf("POS:info1=%i,arg=%d\n",info1,argument);
	}
	else if ((double)argument >= 128)
	{
		argument = (255 - (double)argument)*2;
//  		argument = (unsigned char)argument;
		info1 = inf_reverse;
//  	    printf("NEG:info1=%i,arg=%d\n",info1,argument);
	}
    	if (ioctl(file, I2C_SLAVE, adresse >> 1) >= 0)
			{
			if (i2c_smbus_write_word_data(file, commande, (argument << 8) | info1) >= 0) {			// send info1 AND PWM
				ret = TRUE;
				//printf("Envoi a %x : %i %i\n", adresse, info1, argument);
				}
			}
	else printf("Erreur adressage moteur %x\n", adresse);

	if (!(ret == TRUE))
		 printf("Erreur ecriture moteur %x\n", adresse);
//	I2C_UNLOCK();

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
float Cxl=0;										// Command Variable for Motor x left
float Cxr=0;										// Command Variable for Motor x right
float Cyf=0;										// Command Variable for Motor y front
float Cyb=0;										// Command Variable for Motor y back
float Cxtl=0;										// Command Variable for Motor x top left
float Cxtr=0;										// Command Variable for Motor x top right
float Cytf=0;										// Command Variable for Motor y top front
float Cytb=0;										// Command Variable for Motor y top back
float Czfl=0;										// Command Variable for Motor z front left
float Czfr=0;										// Command Variable for Motor z front right
float Czbl=0;										// Command Variable for Motor z back left
float Czbr=0;										// Command Variable for Motor z back right

// The following variables need to be ajusted such that Fx=1 produces about 1N of thrust
float Mxr=MOTOR_GAIN;									// Motor gain for Motor x right
float Mxl=MOTOR_GAIN;									// Motor gain for Motor x left
float Myf=MOTOR_GAIN;									// Motor gain for Motor y front
float Myb=MOTOR_GAIN;									// Motor gain for Motor y back
float Mzfl=MOTOR_GAIN;									// Motor gain Variable for Motor z front left
float Mzfr=MOTOR_GAIN;									// Motor gain Variable for Motor z front right
float Mzbl=MOTOR_GAIN;									// Motor gain Variable for Motor z back left
float Mzbr=MOTOR_GAIN;									// Motor gain Variable for Motor z back right

/*...Calculations......................................................................*/
if(CFG_6DDL) {
	Cxr=Mxr*(float)(F.Fx/4+F.Tz/(4*L)-F.Ty/(4*L));			// The Command for Motor x right/left With 12M
	Cxl=Mxl*(float)(F.Fx/4-F.Tz/(4*L)-F.Ty/(4*L));
	Cxtr=Mxr*(float)(F.Fx/4+F.Tz/(4*L)+F.Ty/(4*L));			// The Command for TOP Motor x right/left With 12M
	Cxtl=Mxl*(float)(F.Fx/4-F.Tz/(4*L)+F.Ty/(4*L));
} else {
	Cxr=Mxr*(float)(F.Fx/2+F.Tz/(2*L));				// Idem With 8M
	Cxl=Mxl*(float)(F.Fx/2-F.Tz/(2*L));
}
if(CFG_6DDL) {
	Cyf=Myf*(float)(F.Fy/4+F.Tz/(4*L)+F.Tx/(4*L));				// The Command for Motor y front/back With 12M
	Cyb=Myb*(float)(F.Fy/4-F.Tz/(4*L)+F.Tx/(4*L));
	Cytf=Myf*(float)(F.Fy/4+F.Tz/(4*L)-F.Tx/(4*L));				// The Command for TOP Motor y front/back With 12M
	Cytb=Myb*(float)(F.Fy/4-F.Tz/(4*L)-F.Tx/(4*L));
} else {
	Cyf=Myf*(float)(F.Fy/2+F.Tz/(2*L));				// The Command for Motor y front/back With 12M
	Cyb=Myb*(float)(F.Fy/2-F.Tz/(2*L));
}
if(CFG_6DDL) {
	Czfl=Mzfl*(float)(F.Fz/4+(F.Tx-F.Ty)/(4*L));				// The Command for Motor z front left
	Czfr=Mzfr*(float)(F.Fz/4+(-F.Tx-F.Ty)/(4*L));				// The Command for Motor z front right
	Czbl=Mzbl*(float)(F.Fz/4+(F.Tx+F.Ty)/(4*L));				// The Command for Motor z back left
	Czbr=Mzbr*(float)(F.Fz/4+(-F.Tx+F.Ty)/(4*L));				// The Command for Motor z back right
} else {
	Czfl=Mzfl*(float)(F.Fz/4+(F.Tx-F.Ty)/(2*L));				// The Command for Motor z front left
	Czfr=Mzfr*(float)(F.Fz/4+(-F.Tx-F.Ty)/(2*L));				// The Command for Motor z front right
	Czbl=Mzbl*(float)(F.Fz/4+(F.Tx+F.Ty)/(2*L));				// The Command for Motor z back left
	Czbr=Mzbr*(float)(F.Fz/4+(-F.Tx+F.Ty)/(2*L));				// The Command for Motor z back right
}
//int collision = safetyv2();
/*...Send commands......................................................................*/
if (Cxl!=0 || Cxr!=0)//(pos_des.x!=0)||(pos_des.theta_z_ref!=0))	// Send command to motors in x Direction
{
	ecriture_moteur2(ADR_XLEFT, COMMAND_SEND_RPM, Cxl);			// Send command to motor x left
	ecriture_moteur2(ADR_XRIGHT, COMMAND_SEND_RPM, Cxr);			// Send command to motor x right
	ecriture_moteur2(ADR_XTLEFT, COMMAND_SEND_RPM, Cxtl);			// Send command to motor x left TOP
	ecriture_moteur2(ADR_XTRIGHT, COMMAND_SEND_RPM, Cxtr);		// Send command to motor x right TOP
	//printf("\nXL:%3.3f,XR:%3.3f",Cxl,Cxr);	
}	
else if (etat!=INTERACTION)
{
	ecriture_moteur2(ADR_XLEFT, COMMAND_SEND_RPM, 0);			// Send command to motor x left
	ecriture_moteur2(ADR_XRIGHT, COMMAND_SEND_RPM, 0);
	ecriture_moteur2(ADR_XTLEFT, COMMAND_SEND_RPM, 0);			// Send command to motor x left
	ecriture_moteur2(ADR_XTRIGHT, COMMAND_SEND_RPM, 0);
}
if (Cyb!=0 || Cyf!=0)//(pos_des.y!=0)||(pos_des.theta_z_ref!=0))	// Send command to motors in y Direction
{
	ecriture_moteur2(ADR_YFRONT, COMMAND_SEND_RPM, Cyb);
	ecriture_moteur2(ADR_YBACK, COMMAND_SEND_RPM, Cyf);			// Send command to motor y back
	ecriture_moteur2(ADR_YTFRONT, COMMAND_SEND_RPM, Cytb);
	ecriture_moteur2(ADR_YTBACK, COMMAND_SEND_RPM, Cytf);			// Send command to motor y back
	//printf("\nYF:%3.3f,YB:%3.3f",Cyf,Cyb);
}
else if (etat!=INTERACTION)
{
	ecriture_moteur2(ADR_YFRONT, COMMAND_SEND_RPM, 0);		// Send command to motor y front
	ecriture_moteur2(ADR_YBACK, COMMAND_SEND_RPM, 0);			// Send command to motor y back
	ecriture_moteur2(ADR_YTFRONT, COMMAND_SEND_RPM, 0);		// Send command to motor y front
	ecriture_moteur2(ADR_YTBACK, COMMAND_SEND_RPM, 0);			// Send command to motor y back
}
if (Czfl!=0)//pos_des.z!=0)//Send command to motors in z Direction
{
	ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM, Czfl);		// Send command to motor z front left
	ecriture_moteur2(ADR_ZFRIGHT, COMMAND_SEND_RPM, Czfr); 	// Send command to motor z front right
	ecriture_moteur2(ADR_ZBLEFT, COMMAND_SEND_RPM, Czbl); 		// Send command to motor z back left
	ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, Czbr);  	// Send command to motor z back right
	//printf("\nZFL:%3.3f,ZFR:%3.3f",Czfl,Czfr);	
}
else if (etat!=INTERACTION)
{
	ecriture_moteur2(ADR_ZFLEFT, COMMAND_SEND_RPM, 0);		// Send command to motor z front left
	ecriture_moteur2(ADR_ZFRIGHT, COMMAND_SEND_RPM, 0); 	// Send command to motor z front right
	ecriture_moteur2(ADR_ZBLEFT, COMMAND_SEND_RPM, 0); 		// Send command to motor z back left
	ecriture_moteur2(ADR_ZBRIGHT, COMMAND_SEND_RPM, 0);  	// Send command to motor z back right				
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

///////////////////////////////////////////////////////////////////
// Compute new command for each pose received by OptiTrack
//////////////////////////////////////////////////////////////////
void subOpti(const geometry_msgs::Pose::ConstrPtr& msg)
{
	t_forces F;
	// Compute forces from position
	F.Fx = 0.25*msg->position.x;
 	F.Fy = 0.25*msg->position.y;
 	F.Fz = 0.25*msg->position.z;
		//Quaternion x,y,z,w
 	F.Tx = 0.25*msg->orientation.x;
 	F.Ty = 0.25*msg->orientation.y;
	F.Tz = 0.25*msg->orientation.z;
	// Send forces to motors
	forces2motors(F);
}

///////////////////////////////////////////////////////////////////
// MAIN ROS NODE (subscriber)
//////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
        sensors::motor motorInst;
        sensors::motorArray mArray;

        printf("Ouverture bus I2C\n");
                if ((file = open(DEVICE_I2C, O_RDWR)) < 0)
                        perror("Erreur lors de l'ouverture");

        scan_i2c_motors(file, &mArray);

        ros::init(argc, argv, "OptiControl");
        ros::NodeHandle n;

        ros::Subscriber subO = n.subscribe<geometry_msgs::Pose>("pose", 1000, subOpti);
	ros::spin();
}
