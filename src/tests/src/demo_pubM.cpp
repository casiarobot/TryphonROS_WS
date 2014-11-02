#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "i2c-dev.h"
#include "motors.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"


#define	DEVICE_I2C	"/dev/i2c-3"
#define	I2C_LOCK()		{pthread_mutex_lock(&mutex_i2c);}
#define	I2C_UNLOCK()	{pthread_mutex_unlock(&mutex_i2c);}

typedef enum {FALSE, TRUE}
		BOOLEAN;

int file;

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

void getmotorinfo(sensors::motorArray *mArray)                              //$
{
 	struct tm *today;
        long sys_time;
	int debut;
	time(&sys_time);
        today = localtime(&sys_time);
        for(int i=0; i<mArray->motors.size(); ++i){
                if(ioctl(file,I2C_SLAVE,mArray->motors[i].id)>=0){
                        debut = today->tm_sec;
                        mArray->motors[i].speed=i2c_smbus_read_byte_data(file, MOT_RPM);
			mArray->motors[i].volt=(double)i2c_smbus_read_byte_data(file, MOT_VOLT)/10.0;
			mArray->motors[i].curr=(double)i2c_smbus_read_byte_data(file, MOT_CURR)/10.0;
			mArray->motors[i].temp=i2c_smbus_read_word_data(file, MOT_TEMP);
                }
        }
}

int main(int argc, char **argv)
{
	sensors::motor motorInst;
	sensors::motorArray mArray;

	printf("Ouverture bus I2C\n");
		if ((file = open(DEVICE_I2C, O_RDWR)) < 0)
			perror("Erreur lors de l'ouverture");
	//check_i2c_functions();
	scan_i2c_motors(file, &mArray);

	ros::init(argc, argv, "demo_pub");
	ros::NodeHandle n;

	ros::Publisher pubM = n.advertise<sensors::motorArray>("motors", 1000);
	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		getmotorinfo(&mArray);
		pubM.publish(mArray);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
