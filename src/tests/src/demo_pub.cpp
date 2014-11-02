#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/sonarArray.h"

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "i2c-dev.h"
#include "capteurs.h"

#define	VERSION		"1.1a"
#define	DEVICE_I2C	"/dev/i2c-3"
#define	I2C_LOCK()		{pthread_mutex_lock(&mutex_i2c);}
#define	I2C_UNLOCK()	{pthread_mutex_unlock(&mutex_i2c);}

typedef enum {FALSE, TRUE}
		BOOLEAN;

int file;
time_t	seed;


void check_i2c_functions()
{
	unsigned long	funcs;

	if (ioctl(file,I2C_FUNCS,&funcs) >= 0)
	{
		if (funcs & I2C_FUNC_I2C) puts("I2C-3");
		if (funcs & I2C_FUNC_10BIT_ADDR) puts("I2C_FUNC_10BIT_ADDR");
		if (funcs & I2C_FUNC_PROTOCOL_MANGLING) puts("I2C_FUNC_PROTOCOL_MANGLING");
		if (funcs & I2C_FUNC_SMBUS_QUICK) puts("I2C_FUNC_SMBUS_QUICK");
		if (funcs & I2C_FUNC_SMBUS_READ_BYTE) puts("I2C_FUNC_SMBUS_READ_BYTE");
		if (funcs & I2C_FUNC_SMBUS_WRITE_BYTE) puts("I2C_FUNC_SMBUS_WRITE_BYTE");
		if (funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA) puts("I2C_FUNC_SMBUS_READ_BYTE_DATA");
		if (funcs & I2C_FUNC_SMBUS_WRITE_BYTE_DATA) puts("I2C_FUNC_SMBUS_WRITE_BYTE_DATA");
		if (funcs & I2C_FUNC_SMBUS_READ_WORD_DATA) puts("I2C_FUNC_SMBUS_READ_WORD_DATA");
		if (funcs & I2C_FUNC_SMBUS_WRITE_WORD_DATA) puts("I2C_FUNC_SMBUS_WRITE_WORD_DATA");
		if (funcs & I2C_FUNC_SMBUS_PROC_CALL) puts("I2C_FUNC_SMBUS_PROC_CALL");
		if (funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA) puts("I2C_FUNC_SMBUS_READ_BLOCK_DATA");
		if (funcs & I2C_FUNC_SMBUS_WRITE_BLOCK_DATA) puts("I2C_FUNC_SMBUS_WRITE_BLOCK_DATA");
		if (funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK) puts("I2C_FUNC_SMBUS_READ_I2C_BLOCK");
		if (funcs & I2C_FUNC_SMBUS_WRITE_I2C_BLOCK) puts("I2C_FUNC_SMBUS_WRITE_I2C_BLOCK");
	}
	else perror("ioctl() I2C_FUNCS failed");
}

void getdistance(sensors::sonarArray *sArray){
	short distance;
	int debut;
	long sys_time;
	struct tm *today;
	time(&sys_time);
	today = localtime(&sys_time);
	for(int i=0; i<sArray->sonars.size(); ++i){
		if(ioctl(file,I2C_SLAVE,sArray->sonars[i].id)>=0){
			debut = today->tm_sec;
			if(i2c_smbus_write_byte_data(file,SONAR_COMMAND, SONAR_RANGING_CM)>=0){
				while (i2c_smbus_read_byte_data(file, SONAR_SOFTWARE_REVISION) < 0){
					usleep(1000);	// 1000
		   			if (today->tm_sec-debut>1)
		   			{
						printf("Delai depasse en ecriture sonar %02x\n", sArray->sonars[i].id*2);
						return;
					}
				}
				distance = i2c_smbus_read_byte_data(file,SONAR_FIRST_ECHO_HIGH);
				distance <<= 8;
				distance |= i2c_smbus_read_byte_data(file, SONAR_FIRST_ECHO_LOW);
				for(int j=0;j<9;j++)
					sArray->sonars[i].distance[j+1]=sArray->sonars[i].distance[j];
				sArray->sonars[i].distance[0]=distance;
			}
			else
				printf("Erreur ecriture sonar\n");
		}
		usleep(25);
	}
}

void getrz(sensors::compass *boussole)
{
        int     high, low, bearing;

        if (ioctl(file, I2C_SLAVE, boussole->id)>=0)
        	high = i2c_smbus_read_byte_data(file, BOUSSOLE_BEARING_WORD_HIGH);
        else{
                printf("Erreur adressage boussole %x\n", boussole->id*2);
                return;
        }
	usleep(25);

        if (high >= 0){
              	if (ioctl(file,I2C_SLAVE,boussole->id)>=0)
                	low = i2c_smbus_read_byte_data(file, BOUSSOLE_BEARING_WORD_LOW);
                else{
     	           printf("Erreur adressage boussole %x\n", boussole->id*2);
                   return;
                }

                if (low >= 0){
     		/* Combine the two bytes, and get the heading in degrees. */
                	bearing = (high * 256) + low;
                        bearing /= 10.0F;
                        //printf("\nAngle absolut: %d", bearing);

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
	}
}

void scan_i2c_sensors(int file, sensors::sonarArray *sArray, sensors::compass *comp)                              //$
{
        int adr, ret=0;
        sensors::sonar sonarInst;
        int deb, fin, i=0;
        long sys_time;
        struct tm *today;

        time(&sys_time);
        today = localtime(&sys_time);
        deb = today->tm_sec;

        printf("Sonars I2C detectes:");
        for (adr=112; adr<128; adr++) { // de E0 a FC
          if((ret=ioctl( file , I2C_SLAVE , adr )) >= 0){
              if (i2c_smbus_write_byte(file, 0x00) >= 0){
                        printf(" 0x%02x", adr*2);
                        sonarInst.id = adr;
                        sArray->sonars.push_back(sonarInst);
                        //printf("Add %i",sArray->sonars[i].id);
                        i++;
                }
          }
        }
        printf("\n");
	adr=96; //C0
        if((ret=ioctl(file,I2C_SLAVE,adr))>=0){
                if(i2c_smbus_write_byte(file, 0x00) >=0){
			printf("Compass at 0x%02x", 96*2);
                	comp->id=adr;
			getrz(comp);
			comp->head_init=comp->rz[0];
		}
        }

        time(&sys_time);
        today = localtime(&sys_time);
        fin = today->tm_sec;
        printf("\nTEMPS DE REACTION:%i",fin-deb);
        printf("\n");
}

int main(int argc, char **argv)
{
	sensors::sonar sonarInst;
	sensors::sonarArray sArray;
	sensors::compass compInst;

	printf("Ouverture bus I2C\n");
		if ((file = open(DEVICE_I2C, O_RDWR)) < 0)
			perror("Erreur lors de l'ouverture");
	//check_i2c_functions();
	scan_i2c_sensors(file, &sArray, &compInst);

	ros::init(argc, argv, "demo_pub");
	ros::NodeHandle n;

	ros::Publisher pubS = n.advertise<sensors::sonarArray>("sonars", 1000);
	ros::Publisher pubC = n.advertise<sensors::compass>("compass",1000);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		getdistance(&sArray);
		getrz(&compInst);
		pubS.publish(sArray);
		if(compInst.id!=0)
			pubC.publish(compInst);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
