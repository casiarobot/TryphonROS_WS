#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"
#include "sensors/forces.h"
#include "t_commande.h"

void subMotor(const sensors::motorArray::ConstPtr& msg)
{
  for (int i=0; i<msg->motors.size(); ++i)
  {
   const sensors::motor &motor = msg->motors[i];
    ROS_INFO_STREAM("ID: " << motor.id << " - RPM: " << motor.rpm <<
                    ", V: " << motor.volt << ", I: " << motor.curr << " , T: " << motor.temp);
  }
}

T_TOUCHE keyboard()
{
    ssize_t size;
    T_TOUCHE ch=NONE;

	size = read(STDIN_FILENO, &ch, 1);
	if((size==0)||(ch==255))
		return NONE;
	else
		return ch;
}

void init_keyboard(int on)
{
	static struct termio config;
	static struct termio work;
	if(on){
		ioctl(0,TCGETA,&config);
		ioctl(0,TCGETA,&work);
		work.c_iflag &= ~(INLCR|ICRNL);
		work.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK);
		work.c_cc[VMIN]=0;
		work.c_cc[VTIME]=0;
		ioctl(0,TCSETA,&work);
	} else
		ioctl(0,TCSETA,&config);
}

void sendCom(T_TOUCHE key, boost::array<float,6> F){
	switch(key){
		case TOUCHE_AVANT:
			F[0]=1;
			break;
		case TOUCHE_ARRIERE:
			F[0]=-1;
			break;
		case TOUCHE_GAUCHE:
			F[1]=1;
			break;
		case TOUCHE_DROITE:
			F[1]=-1;
			break;
		case TOUCHE_HAUT:
			F[2]=1;
			break;
		case TOUCHE_BAS:
			F[2]=-1;
			break;
		case TOUCHE_ROTATION_GAUCHE:
			F[5]=1;
			break;
		case TOUCHE_ROTATION_DROITE:
			F[5]=-1;
			break;
		case TOUCHE_ROTATION_XP:
			F[3]=1;
			break;
		case TOUCHE_ROTATION_XN:
			F[3]=-1;
			break;
		case TOUCHE_ROTATION_YP:
			F[4]=1;
			break;
		case TOUCHE_ROTATION_YN:
			F[4]=-1;
			break;
		case TOUCHE_ARRET_XYZ:
			F[0]=0;F[1]=0;F[2]=0;F[3]=0;F[4]=0;F[5]=0;
			break;
	}
}

int main(int argc, char **argv)
{
  init_keyboard(1);
  T_TOUCHE key = NONE;
  boost::array<float,6> Fc = {0,0,0,0,0,0};
  sensors::forces F;
  ros::init(argc, argv, "motorsSubscriber");
  ros::NodeHandle n;
  ros::Subscriber subM = n.subscribe("motors", 1000, subMotor);
  ros::Publisher pubF = n.advertise<sensors::forces>("forces",1000);
  ros::Rate loop_rate(100);

/*  while(ros::ok())
  {
	key=keyboard();
  	if(key!=NONE){
		printf("Key %c hit\n", key);
		sendCom(key, Fc);
		F.target = Fc;
		pubF.publish(F);
	}
	ros::Subscriber subM = n.subscribe("motors", 1000, subMotor);
	ros::spinOnce();
	loop_rate.sleep();
  }*/
  ros::spin();
  init_keyboard(0);
  return 0;
}
