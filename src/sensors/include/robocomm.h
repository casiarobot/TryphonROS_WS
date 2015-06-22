#ifndef ROBOCOMM_H
#define ROBOCOMM_H

#include <termios.h>
#include <unistd.h>
#include <vector>
#include "imumaths.h"

#include "Leddar.h"

// Version number of the Robovero
#define VERSION_ROBOVERO 1

class ROBOCOMM
{
public:
	imu::Vector<3> g;
	imu::Vector<3> a;
	imu::Vector<3> m;
	imu::Vector<3> accel_gains;
	float gyro_offsets[3];
	float mag_cal[6];
	int IssonarCon[16];
	int IsmotorCon[16];

	ROBOCOMM();
	int getACM();
	int millis();
	int opens();
	int Init();
	void updateIMUdata();
	imu::Vector<3> getIMUdata(bool ouput, char c);
	void getFifo(bool output, int* res);
	void init_sonar(int id);
	void init_leddarone();
    void test_leddarone(int id);
	void ask_dist_leddarone();
	sensors::leddarArray get_dist_leddarone();
	int askSonar(int id);
	int readSonar(int id);
	float readCompass(int id);
	void get_dist();
	void test_motor(int vit);
	void getMotorData(int vit, int* res);
	void init_moteur(int id);
	void setMotor(int id, int speed, int dir);
	int readLum(int id);
	void swRelay(int id);
	void addressing(int newid);
	void setInd(int id, int com, int r, int g, int b);
	void setservo(int id, int pwm);
	int analog_read(int id);
	void init_imu();
	int scanI2C(int id);
	void scanAllI2C();
	void mag_calibration();
	void setGYROOffsets(float _Xoffset, float _Yoffset, float _Zoffset);
	void GYROzeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS);
	void safeCloseConnection();
	void killConnection();

	static void TransmitCommand (const char* command, int serial_if);
	static unsigned long ReceiveULong (int serial_if);
	static size_t ReceiveStr (int serial_if, char * msg);
	static void WaitForRobovero (void);
	static void WaitForReception (size_t bytes);

	int getSerialIf();

private:
	struct termios if_mode;
	char           robovero_dev[16];
	int            serial_if;
	int            error;
	std::vector<Leddar*>            leddars;


};


#endif

