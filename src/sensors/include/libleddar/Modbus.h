// *****************************************************************************
// Module..: SerialDemo
//
/// \file    Modbus.h
///
/// \brief   Declarations for the Modbus layer of the demo.
///
// Copyright (c) 2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#ifndef _MOBDUS_H_
#define _MOBDUS_H_

#include "OS.h"

#define MODBUS_MAX_PAYLOAD 252
#define MODBUS_SERVER_ID   0x11


void 
ModbusSetSerial(int serial_if);

LtBool
ModbusConnected( void );

LtResult
ModbusSend(LtByte aFunction, LtByte *aBuffer, LtByte aLength, LtByte aAddressI2C, LtHandle aHandle, LtByte &aLastFunction);

LtResult
ModbusReceive( LtByte *aBuffer, LtByte aAddressI2C, size_t aLenMessage, LtHandle aHandle, LtByte &aLastFunction);

LtResult
ModbusConnect(LtByte aAddressI2C, LtHandle aHandle, LtByte &aLastFunction);

void
ModbusDisconnect( void );

LtResult
ModbusReadInputRegisters(LtU16 aNo, LtU16 aCount, LtU16 *aValue, LtByte aAddressI2C, LtHandle aHandle, LtByte &aLastFunction);

LtResult
ModbusReadHoldingRegister( LtU16 aNo, LtU16 *aValue, LtByte aAddressI2C, LtHandle aHandle);

LtResult
ModbusWriteRegister( LtU16 aNo, LtU16 aValue, LtByte aAddressI2C, LtHandle aHandle);

#endif
