#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/sonarArray.h"
#include "sensors/imuros.h"

#include "robocomm.h"


#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "i2c-dev.h"
#include "capteurs.h"

#define VERSION         "1.1a"
#define DEVICE_I2C      "/dev/i2c-3"
#define I2C_LOCK()              {pthread_mutex_lock(&mutex_i2c);}
#define I2C_UNLOCK()    {pthread_mutex_unlock(&mutex_i2c);}

typedef enum {FALSE, TRUE}
                BOOLEAN;

ROBOCOMM robovero;

int file;
time_t  seed;
double ax,ay;
double Fx[3],Fy[3],Ax[3],Ay[3];

void getimudata(sensors::imuros *imuptr )
{

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

ax=1*(atan(a[0]/a[2])*5);
ay=1*(atan(a[1]/a[2])*5);
//ROS_INFO("ax: %f, ay: %f",fx,fy);
//ROS_INFO("a0: %f,a1: %f,a2: %f",a[0],a[1],a[2]);
}


int main(int argc, char **argv)
{
       if(robovero.Init()==-1){return 0;}
        robovero.GYROzeroCalibrate(120,15); //# of data and millisecond to initiliaze
        robovero.mag_calibration();
        sensors::imuros imuInst; //ros geo msg

        printf("Ouverture bus I2C\n");
        ros::init(argc, argv, "minicube");
        ros::NodeHandle n;

        ros::Publisher pubI = n.advertise<sensors::imuros>("imu", 1000);
        ros::Rate loop_rate(20);

	for(int l=0;l<3;l++)
               {
                Fx[l]=0;
                Fy[l]=0;
                Ax[l]=0;
                Ay[l]=0;
        }
        while(ros::ok())
        {
                getimudata(&imuInst);
		for(int k=0;k<2;k++)
			{
			Fx[k]=Fx[k+1];
			Fy[k]=Fy[k+1];
			Ax[k]=Ax[k+1];
                        Ay[k]=Ay[k+1];
		}
		Ax[2]=ax;
		Ay[2]=ay;
		Fx[2]=0.1729*Ax[2]+0.3458*Ax[1]+0.1729*Ax[0]-(-0.5301*Fx[1]+0.2217*Fx[0]);
		Fy[2]=0.1729*Ay[2]+0.3458*Ay[1]+0.1729*Ay[0]-(-0.5301*Fy[1]+0.2217*Fy[0]);
		ROS_INFO("ax: %f, ay: %f",ax,ay);
		ROS_INFO("fx: %f, fy: %f",Fx[2],Fy[2]);
                pubI.publish(imuInst);
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
