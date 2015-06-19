#ifndef LEDDAR_H
#define LEDDAR_H

#include <ros/ros.h>
#include "sensors/sonarArray.h"
#include "sensors/leddarArray.h"
#include "libleddar/OS.h"

#define LEDDAR_MAX_DETECTIONS 3

// Register addresses for configuration parameters
#define LEDDAR_CONFIG_ACCUMULATION   0
#define LEDDAR_CONFIG_OVERSAMPLING   1
#define LEDDAR_CONFIG_SAMPLE_COUNT   2
#define LEDDAR_CONFIG_LED_POWER      4
#define LEDDAR_CONFIG_BAUD_RATE      29
#define LEDDAR_CONFIG_MODBUS_ADDRESS 30

typedef struct _LtDetection
{
	float mDistance;
	float mAmplitude;
} LtDetection;

typedef struct _LtAcquisition
{
	LtU32 mTimestamp;
	float mTemperature;
	LtU16 mDetectionCount;

	LtDetection mDetections[LEDDAR_MAX_DETECTIONS];
} LtAcquisition;

class Leddar
{
public:
	Leddar();
	void connect(LtByte aAddressI2C, LtHandle aHandle);
	void askDistance();
	void getDistance();
	sensors::leddar getRosMsg();
private:
	void connect();
	void setBridgeParameter();
	void isBridgeConnected();
	void connectLeddar();
	LtResult readInputRegistersSend(LtU16 aNo, LtU16 aCount);
	LtResult readInputRegistersReceive(LtU16 aNo, LtU16 aCount, LtU16 *aValues);
	void addDistanceToMsg(const LtDetection &aDetection);

	LtByte mAddressI2C;
	LtHandle mHandle;
	LtByte mLastFunction;
	bool mWasAskDistanceCall;
	sensors::leddar mMsg;
};

#endif // LEDDAR_H
