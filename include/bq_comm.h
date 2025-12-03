#ifndef BQ_COMM_H
#define BQ_COMM_H

#include <Arduino.h>
#include "config.h"
#include "bq_comm_defs.h"
#include "B0_reg.h" 
#include "bq79600_reg.h" 
#include "bms_data_types.h"

uint16_t calculateCRC16(const uint8_t *data, size_t length);

BMSErrorCode_t sendFrame(const uint8_t *frame, size_t length);
BMSErrorCode_t receiveFrame(uint8_t *buffer, size_t expected_length, uint32_t timeout_ms);
BMSErrorCode_t buildAndSendFrame(uint8_t deviceID, uint16_t regAddr, const uint8_t *data, uint8_t dataLen, uint8_t frameType);

BMSErrorCode_t bqWriteReg(uint8_t deviceID, uint16_t regAddr, uint64_t data, uint8_t numBytes, uint8_t frameType);
BMSErrorCode_t bqReadReg(uint8_t deviceID, uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesToRead, uint8_t frameType, uint32_t timeout_ms = SERIAL_TIMEOUT_MS);

BMSErrorCode_t bqBroadcastWrite(uint16_t regAddr, uint64_t data, uint8_t numBytes);
BMSErrorCode_t bqBroadcastRead(uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesToRead, uint32_t timeout_ms = SERIAL_TIMEOUT_MS * TOTAL_BQ_DEVICES);
BMSErrorCode_t bqStackWrite(uint16_t regAddr, uint64_t data, uint8_t numBytes);
BMSErrorCode_t bqStackRead(uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesPerDevice, uint32_t timeout_ms = SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES * 2); // Increased timeout multiplier
BMSErrorCode_t bqBroadcastWriteReverse(uint16_t regAddr, uint64_t data, uint8_t numBytes);

//MUXcontrol
BMSErrorCode_t bqSetGpioconfig(uint8_t deviceID, uint8_t gpioIndex, uint8_t configValue);
BMSErrorCode_t bqSetMuxChannel(uint8_t channel);

void bqInitCommunication();
BMSErrorCode_t bqWakePing();
BMSErrorCode_t bqWakeUpStack();
BMSErrorCode_t bqAutoAddressStack();

BMSErrorCode_t bqConfigureBridgeForRing();
BMSErrorCode_t bqConfigureStackForHeartbeat(); 
BMSErrorCode_t bqSleepDevices(); 
BMSErrorCode_t bqShutdownDevices(); 

void bqDelayUs(unsigned int us);
void bqDelayMs(unsigned int ms);

#endif // BQ_COMM_H