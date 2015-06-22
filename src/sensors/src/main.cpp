typedef unsigned char byte;

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>


#include <stdio.h>
#include <unistd.h>
#include <unistd.h>
#include <string.h> /* for strncpy */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include "robocomm.h"
#include "imu_driver.h"


ROBOCOMM robovero;

#include "sensors_thrusters.cpp"



sensors::imuros getDataAndGenerateMessage(){
	robovero.updateIMUdata();
	imu::Vector<3> a, g, m;

	a = robovero.a;
	g = robovero.g;
	m = robovero.m;
	sensors::imuros msg;

	msg.gyro[0] = g[0];
	msg.gyro[1] = g[1];
	msg.gyro[2] = g[2];

	msg.accel[0] = a[0];
	msg.accel[1] = a[1];
	msg.accel[2] = a[2];

	msg.magn[0] = m[0];
	msg.magn[1] = m[1];
	msg.magn[2] = m[2];

	return msg;
}

std::string get_ip()
{
	int fd;
	struct ifreq ifr;
	char ip[100];

	fd = socket(AF_INET, SOCK_DGRAM, 0);

	/* I want to get an IPv4 IP address */
	ifr.ifr_addr.sa_family = AF_INET;

	/* I want IP address attached to "eth0" */
	strncpy(ifr.ifr_name, "wlan1", IFNAMSIZ-1);

	ioctl(fd, SIOCGIFADDR, &ifr);

	close(fd);

	/* display result */
	sprintf(ip,"%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));

	if(strncmp(ip, "192.168.", 8) != 0){
		strcpy(ip, "127.0.0.1");
		ROS_INFO("Ip is not format into 192.168.X.X, change to 127.0.0.1");
	}

	std::string s = ip;
	std::replace(s.begin(), s.end(), '.', '_');
	//ip=s.c_str();
	return s;
}



int main(int argc, char *argv[]){
	// Get ip
	std::string ip = get_ip();
	char rosname[100];

	sprintf(rosname,"sensors_thrusters_%s", ip.c_str());
	ROS_INFO("STARTING ROS NODE: %s",rosname);

	ros::init(argc, argv, rosname);
	//ros::init(argc, argv, "sensors_thruster", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	//////////////////////////////////
	///              Publisher      ///
	sensors::imubuff msg;
	sprintf(rosname,"/%s/sonars", ip.c_str());
	ros::Publisher pubS = n.advertise<sensors::sonarArray>(rosname, 1);
	sprintf(rosname,"/%s/compass", ip.c_str());
	ros::Publisher pubC = n.advertise<sensors::compass>(rosname,1);
	sprintf(rosname,"/%s/leddars", ip.c_str());
	ros::Publisher pubL = n.advertise<sensors::leddarArray>(rosname,1);
	//ros::Publisher force_dyn_comp = n.advertise<geometry_msgs::Wrench>("final_forces",1);
	sprintf(rosname,"/%s/motors_info", ip.c_str());
	ros::Publisher motors_info = n.advertise<sensors::motorArray>(rosname,1);
	sprintf(rosname,"/%s/battery_logic", ip.c_str());
	ros::Publisher battery_logic = n.advertise<std_msgs::Int32>(rosname,1);
	sprintf(rosname,"/%s/imubuff", ip.c_str());
	ros::Publisher pubImu = n.advertise<sensors::imubuff>(rosname, 1);


	//             Subscriber      ////
	sprintf(rosname,"/%s/command_props", ip.c_str());
	ros::Subscriber control_vector= n.subscribe(rosname, 1, controlcallback);
	sprintf(rosname,"/%s/magnet_on",ip.c_str());
	ros::Subscriber magnet=n.subscribe(rosname,1,magnetcallback);

	ros::Rate loop_rate(10);
	// Serial initiation loop
	while(ros::ok()){

		// Init sonars and motors message
		sensors::sonar sonarInst;
		sensors::sonarArray sArray;
		sensors::compass compInst;
		sensors::motor motorInst;
		sensors::motorArray mArray;
		int fifoData[3];

		for(int i=0;i<12;i++){Fcontrol[i]=0;}

		int print = 0;

		do{
			sleep(1);

			//If ros die, end everything
			if(!ros::ok())
				return 0;
		}
		while(robovero.Init());

		printf("\nRobovero.Init() succeded!\n");

		try{
			robovero.GYROzeroCalibrate(128, 2);
			robovero.mag_calibration();

			scan_i2c_motors(file, &mArray);
			scan_i2c_sensors(file, &sArray, &compInst);
			init_sonar(&sArray);
			robovero.init_leddarone();
		}
		catch(std::string e){
			printf("Error :\n %s \n", e.c_str());
			robovero.killConnection();
			continue; // Return to the start of the Serial initiation loop
		}


		printf("\nYou can send command NOW !\n\n");

		/*
			When a exception is throw:
			The exception is catch, the node wait for the Robovero to respond
			reopen the serial port and restart at the *begin* of the while.
			Each time you call a ReceiveULong the serial comm may fail and
			restart everything from the start of the Serial initiation loop
		*/
		try{
			while(ros::ok()){
				robovero.ask_dist_leddarone();
				askdistance(&sArray);

				msg.header.stamp = ros::Time::now();

				robovero.getFifo(0, fifoData);
				int max = fifoData[3]; //param true to output Fifo content
				max = std::min(12, max); // Max 12
				for(int i=0;i<max;i++){
					msg.buffer.push_back(getDataAndGenerateMessage());
				}

				pubImu.publish(msg);
				msg.buffer.clear();
				// checking if commands are published
				if(fabs(ros::Time::now().toSec()-msgTime)>2.0) // if the delay exceeds 2 secs forces go to 0
				{
					for(int i=0;i<12;i++){Fcontrol[i]=0;}
				}

				// send motor commands
				forces2motors(Fcontrol,mArray);

				// get motor data
				getmotor_data(&mArray);
				motors_info.publish(mArray);

				// check battery level
				for(int i = 0; i < mArray.motors.size(); ++i){
					if(mArray.motors[i].volt < 45.0)
						ROS_WARN("Motor %02X has low battery", mArray.motors[i].id * 2);
				}

				// get compass data
				if(compInst.id!=0){
					getrz(&compInst);
					pubC.publish(compInst);
				}

				// get battery logic level
				std_msgs::Int32 bat;
				bat.data = robovero.analog_read(0);
				battery_logic.publish(bat);

				// get sonar data
				getdistance(&sArray);
				pubS.publish(sArray);


				pubL.publish(robovero.get_dist_leddarone());

				//std::cout << loop_rate.cycleTime() << std::endl;
				ros::spinOnce();
				loop_rate.sleep();
			} // End of while( ros::ok) aka publishing loop
		}
		catch(std::string e){
			printf("Error :\n %s \n", e.c_str());
			robovero.killConnection();
			continue; // Restart serial initiation loop
		}
	} // End of serial initiation loop

	robovero.safeCloseConnection();
	return 0;
}



