#ifndef ROBOCOMM_H
#define ROBOCOMM_H

#include <termios.h>
#include <unistd.h>
//#include <string>
#include "imumaths.h"

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
    int Init();
    imu::Vector<3> getIMUdata(bool ouput, char c);
    void init_sonar(int id);
    int readSonar(int id);
    int askSonar(int id);
    float readCompass(int id);
    void get_dist();
    void test_motor(int vit);
    int* getMotorData(int vit);
    void init_moteur(int id);
    void setMotor(int id, int speed, int dir);
    void setInd(int id, int com, int r, int g, int b);
    void setservo(int id, int pwm);
    int analog_read(int id);
    void init_imu();
    int scanI2C(int id);
    void scanAllI2C();
    void mag_calibration();
    void setGYROOffsets(float _Xoffset, float _Yoffset, float _Zoffset);
    void GYROzeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS);
    void closeI();

private:
    struct termios if_mode;
    const char*          robovero_dev;
    int            serial_if;
    int            error;
    
    void WaitForRobovero (void);
    void WaitForReception (size_t bytes);
    void TransmitCommand (const char* command, int serial_if);
    unsigned long ReceiveULong (int serial_if);
    void ReceiveStr (int serial_if);

};


#endif

