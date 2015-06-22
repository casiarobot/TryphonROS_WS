
#include <stdlib.h>
#include <string>
#include <stdio.h>

#include "robocomm.h"
#include "Leddar.h"
#include "libleddar/Modbus.h"

Leddar::Leddar():
				mAddressI2C(0x90),
				mHandle(LT_INVALID_HANDLE),
				mWasAskDistanceCall(false){}

void Leddar::connect(LtByte aAddressI2C, LtHandle aHandle){
	mAddressI2C = aAddressI2C;
	mHandle = aHandle;

	connect();
}

void Leddar::connect(){
	isBridgeConnected();
	setBridgeParameter();

	connectLeddar();
}

void Leddar::isBridgeConnected(){
	char str[100];
	sprintf(str,"testLeddar %X\r\n", mAddressI2C);
	ROBOCOMM::TransmitCommand(str, mHandle);

	int testResult = ROBOCOMM::ReceiveULong(mHandle);
	if(testResult != 1)
		throw std::string("Test SC16IS750 (aka leddar Bridge): not connected!\n");
	//else
	//	printf("Test SC16IS750 (aka leddar Bridge): connected\n");
}

void Leddar::setBridgeParameter(){
	char str[100];
	//Reset device
	sprintf(str,"resetDevice %X\r\n", mAddressI2C);
	ROBOCOMM::TransmitCommand(str, mHandle);
	//Set baudrate
	sprintf(str,"setSerialBaudrate %X\r\n", mAddressI2C);
	ROBOCOMM::TransmitCommand(str, mHandle);
	//Set line parameter
	sprintf(str,"setLine %X\r\n", mAddressI2C);
	ROBOCOMM::TransmitCommand(str, mHandle);
}

void Leddar::connectLeddar(){
	LtResult lResult;

	lResult = ModbusConnect(mAddressI2C, mHandle, mLastFunction);

	if(lResult != LT_SUCCESS)
		throw std::string("[Modbus] ModbusConnect FAIL\n");

	lResult = ModbusSend(MODBUS_SERVER_ID, NULL, 0, mAddressI2C, mHandle, mLastFunction);

	if(lResult != LT_SUCCESS)
		throw std::string("[Modbus] ModbusSend FAIL\n");

	LtByte lId[MODBUS_MAX_PAYLOAD];

	//printf("[Modbus] Receive:\n");
	lResult = ModbusReceive(lId, mAddressI2C, 56, mHandle, mLastFunction);
	if (lResult < 0)
		throw std::string("[Modbus] ModbusReceive FAIL\n");


	// Make sure we are talking to a Leddar One
	if (lId[50] != 10  || lId[51] != 0 )
		throw std::string("[Modbus] The LeddarOne is not connected.\n");

	//printf("[Modbus] Connection SUCCESS!\n");
}


void Leddar::askDistance(){
	mWasAskDistanceCall = true;

	LtResult lResult = readInputRegistersSend( 20, 10);
	if(lResult != LT_SUCCESS)
		throw std::string("Fail to send data to LeddarOne.\n");
}

void Leddar::getDistance(){
	if(mWasAskDistanceCall){
		mWasAskDistanceCall = false;

		LtAcquisition aAcquisition;
		LtU16 lValues[10];

		LtResult lResult = readInputRegistersReceive( 20, 10, lValues);

		if (lResult == LT_SUCCESS){
			LtDetection *lDetections = aAcquisition.mDetections;

			aAcquisition.mTimestamp = lValues[0] + (lValues[1]<<16);
			aAcquisition.mTemperature = lValues[2]/256.f;
			aAcquisition.mDetectionCount = lValues[3] < LEDDAR_MAX_DETECTIONS ? lValues[3] : LEDDAR_MAX_DETECTIONS;

			lDetections->mDistance = lValues[4]/1000.f;
			lDetections->mAmplitude = lValues[5]/256.f;

			//printf("Leddar(0x%x): %7.3f %6.2f \r\n", mAddressI2C, lDetections[0].mDistance, lDetections[0].mAmplitude);
			addDistanceToMsg(*lDetections);
		}
		else{
			ROS_ERROR("Fail to receive responce from Leddar in getDistance()");

			ROS_INFO("Trying to reconnected it...");
			connect();
			//throw std::string("Fail to receive responce from Leddar in getDistance()");
		}
	}
	else{
		throw std::string("Call Leddar::askDistance() before Leddar::getDistance()");
	}
}


sensors::leddar Leddar::getRosMsg(){
	mMsg.id = mAddressI2C;
	return mMsg;
}

void Leddar::addDistanceToMsg(const LtDetection &aDetection){
	mMsg.distance = aDetection.mDistance * 100.0;
}

LtResult Leddar::readInputRegistersSend(LtU16 aNo, LtU16 aCount){
	LtByte   lPayload[MODBUS_MAX_PAYLOAD];
	LtResult lResult;

	lPayload[0] = aNo >> 8;
	lPayload[1] = aNo & 0xFF;
	lPayload[2] = aCount >> 8;
	lPayload[3] = aCount & 0xFF;

	lResult = ModbusSend( 4, lPayload, 4, mAddressI2C, mHandle, mLastFunction);

	return lResult;
}

LtResult Leddar::readInputRegistersReceive(LtU16 aNo, LtU16 aCount, LtU16 *aValues){
	LtResult lResult;
	LtByte   lPayload[MODBUS_MAX_PAYLOAD];

	lPayload[0] = aNo >> 8;
	lPayload[1] = aNo & 0xFF;
	lPayload[2] = aCount >> 8;
	lPayload[3] = aCount & 0xFF;

	lResult = ModbusReceive(lPayload, mAddressI2C, 25, mHandle, mLastFunction);

	if(lResult >= 0){
		for(LtU16 i = 0; i < aCount; ++i){
			aValues[i] = lPayload[ i * 2 + 1 ] * 256 + lPayload[ i * 2 + 2];
		}
		return LT_SUCCESS;
	}

	return lResult;
}
