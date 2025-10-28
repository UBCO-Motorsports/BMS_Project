#include "bq_comm.h"
#include "config.h" 
#include "B0_reg.h" 
#include "bq79600_reg.h" 

static const uint16_t crc16_ibm_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t calculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF; 
    for (size_t i = 0; i < length; ++i) {
        crc = (crc >> 8) ^ crc16_ibm_table[(crc & 0xFF) ^ data[i]];
    }
    return crc;
}

void bqInitCommunication() {
    BQ_UART_SERIAL.begin(BQ_UART_BAUDRATE, SERIAL_8N1);
    BQ_UART_SERIAL.setTimeout(SERIAL_TIMEOUT_MS); 
}

BMSErrorCode_t sendFrame(const uint8_t *frame, size_t length) {
    BQ_UART_SERIAL.write(frame, length);
    BQ_UART_SERIAL.flush(); 
    delayMicroseconds(UART_TX_DELAY_US); 
    return BMS_OK;
}

BMSErrorCode_t receiveFrame(uint8_t *buffer, size_t expected_length, uint32_t timeout_ms) {
    unsigned long startTime = millis();
    size_t bytesRead = 0;
    if (expected_length == 0) return BMS_OK; // Nothing to read

    while (bytesRead < expected_length && (millis() - startTime) < timeout_ms) {
        if (BQ_UART_SERIAL.available()) {
            buffer[bytesRead++] = BQ_UART_SERIAL.read();
        }
    }
    if (bytesRead < expected_length) {
        // BMS_DEBUG_PRINTF("Receive timeout: expected %d, got %d bytes in %lu ms\n", expected_length, bytesRead, millis() - startTime);
        // Clear remaining buffer from UART to prevent old data carryover
        while(BQ_UART_SERIAL.available()) BQ_UART_SERIAL.read();
        return BMS_ERROR_COMM_TIMEOUT;
    }
    return BMS_OK;
}

BMSErrorCode_t buildAndSendFrame(uint8_t deviceID, uint16_t regAddr,
                                 const uint8_t *data, uint8_t dataLen,
                                 uint8_t frameType) {
    uint8_t frame[MAX_COMMAND_FRAME_SIZE];
    size_t idx = 0;
    //new logic handling of frameType
    bool isSingle = (frameType == FRMWRT_SGL_R || frameType == FRMWRT_SGL_W);
    bool isWrite = (frameType == FRMWRT_SGL_W || 
                    frameType == FRMWRT_STK_W ||
                    frameType == FRMWRT_ALL_W ||
                    frameType == FRMWRT_REV_ALL_W);
    bool isReverse = (frameType == FRMWRT_REV_ALL_W);
    bool isRead = (frameType == FRMWRT_SGL_R ||
                   frameType == FRMWRT_STK_R ||
                   frameType == FRMWRT_ALL_R);

    //Build command byte
    uint8_t cmd;

    if (isReverse) {
        // Reverse broadcast write command format: 0b111xxxxx (0xE0+len)
        cmd = 0xE0 | ((dataLen - 1) & 0x07);
    } else {
        cmd = 0x80 | (frameType & 0x70); // normal cmd byte format
        if (isWrite) cmd |= ((dataLen - 1) & 0x07);
    }

    frame[idx++] = cmd;

    // Single-device: include device ID
    if (isSingle) {
        frame[idx++] = deviceID;
    }

    // Register address
    frame[idx++] = (regAddr >> 8) & 0xFF;
    frame[idx++] = regAddr & 0xFF;

    // Read: append length-1 byte (only for read)
    if (isRead) {
        frame[idx++] = dataLen - 1;
    }

    // Write: append payload
    if (isWrite && dataLen > 0 && data != nullptr) {
        for (uint8_t i = 0; i < dataLen; i++) {
            frame[idx++] = data[i];
        }
    }

    // CRC16
    uint16_t crc = calculateCRC16(frame, idx);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = crc >> 8;
    //return frame
    return sendFrame(frame, idx);
}


BMSErrorCode_t bqWriteReg(uint8_t deviceID, uint16_t regAddr, uint64_t data, uint8_t numBytes, uint8_t frameType) {
    if (numBytes == 0 || numBytes > MAX_WRITE_DATA_BYTES) {
        return BMS_ERROR_INVALID_RESPONSE; 
    }
    uint8_t dataBytes[MAX_WRITE_DATA_BYTES];
    for (int i = 0; i < numBytes; ++i) {
        dataBytes[i] = (data >> (8 * (numBytes - 1 - i))) & 0xFF; 
    }
    return buildAndSendFrame(deviceID, regAddr, dataBytes, numBytes, frameType);
}

BMSErrorCode_t bqReadReg(uint8_t deviceID, uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesToRead, uint8_t frameType, uint32_t timeout_ms) {
    if (numBytesToRead == 0 || numBytesToRead > MAX_READ_DATA_BYTES) {
        return BMS_ERROR_INVALID_RESPONSE;
    }

    BMSErrorCode_t status = buildAndSendFrame(deviceID, regAddr, nullptr, numBytesToRead, frameType);
    if (status != BMS_OK) return status;

    size_t expectedResponseLength = SINGLE_DEVICE_RESPONSE_OVERHEAD + numBytesToRead;
    uint8_t responseFrame[MAX_SINGLE_DEVICE_RESPONSE_SIZE]; // Use max possible size for a single response
    if (expectedResponseLength > MAX_SINGLE_DEVICE_RESPONSE_SIZE) {
        Serial.println("Error: ReadReg expected response too large for buffer.");
        return BMS_ERROR_UNKNOWN;
    }
    
    status = receiveFrame(responseFrame, expectedResponseLength, timeout_ms);
    if (status != BMS_OK) return status;

    uint16_t received_crc = (uint16_t)(responseFrame[expectedResponseLength - 1] << 8) | responseFrame[expectedResponseLength - 2];
    uint16_t calculated_crc = calculateCRC16(responseFrame, expectedResponseLength - 2);
    if (received_crc != calculated_crc) {
        BMS_DEBUG_PRINTF("ReadReg CRC mismatch! Exp: 0x%04X, Got: 0x%04X for Dev %d, Reg 0x%04X\n", calculated_crc, received_crc, deviceID, regAddr);
        return BMS_ERROR_CRC;
    }

    // TODO: Verify response INIT byte, Device ID (if applicable), and Register Address
    if ((responseFrame[0] & 0x80) == 0x80) { // Check if it's a command frame (bit 7 = 1)
        Serial.println("Error: Expected response frame, got command frame type.");
        return BMS_ERROR_INVALID_RESPONSE;
    }
    if ((frameType == FRMWRT_SGL_R) && (responseFrame[1] != deviceID)) {
        BMS_DEBUG_PRINTF("Error: Response from wrong device ID. Expected %d, Got %d\n", deviceID, responseFrame[1]);
        return BMS_ERROR_INVALID_RESPONSE;
    }
    // Add more checks for REG_ADDR if needed

    memcpy(readBuffer, &responseFrame[4], numBytesToRead); 
    return BMS_OK;
}

BMSErrorCode_t bqBroadcastWrite(uint16_t regAddr, uint64_t data, uint8_t numBytes) {
    return bqWriteReg(0, regAddr, data, numBytes, FRMWRT_ALL_W); 
}

BMSErrorCode_t bqBroadcastRead(uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesToRead, uint32_t timeout_ms /* = SERIAL_TIMEOUT_MS * TOTAL_BQ_DEVICES */)
{
    /* ---- argument sanity ---- */
    if (readBuffer == nullptr ||
        numBytesToRead == 0 ||
        numBytesToRead > MAX_READ_DATA_BYTES)
        return BMS_ERROR_INVALID_RESPONSE;
#define BMS_DEBUG
    if (regAddr < 0x0000 || regAddr > 0x3FFF) {
        Serial.println(F("[BRD_R] Invalid register address"));
        return BMS_ERROR_INVALID_RESPONSE;
    }
#ifdef BMS_DEBUG
    Serial.print  (F("[BRD_R] reg 0x"));
    Serial.print  (regAddr, HEX);
    Serial.print  (F("  len "));
    Serial.println(numBytesToRead);
#endif

    /* ---- 1. send the ALL_R command frame ---- */
    BMSErrorCode_t st = buildAndSendFrame(
        0, regAddr,
        nullptr,
        numBytesToRead,          /* LEN = bytes to read */
        FRMWRT_ALL_R);           /* broadcast read opcode */
    if (st != BMS_OK) {
#ifdef BMS_DEBUG
        Serial.println(F("[BRD_R] buildAndSendFrame failed"));
#endif
        return st;
    }

    /* ---- 2. receive response frames ---- */
    constexpr size_t HDR_LEN  = 4;     /* INIT, ADDR, CMD, LEN */
    constexpr size_t CRC_LEN  = 2;     /* 16-bit CRC           */
    const     size_t FRAME_LEN  = HDR_LEN + numBytesToRead + CRC_LEN;
    const     size_t TOTAL_LEN  = FRAME_LEN * TOTAL_BQ_DEVICES;

    uint8_t rxBuf[TOTAL_LEN];
    st = receiveFrame(rxBuf, TOTAL_LEN, timeout_ms);
    if (st != BMS_OK) {
#ifdef BMS_DEBUG
        Serial.println(F("[BRD_R] receiveFrame timeout / UART error"));
#endif
        return st;
    }

#ifdef BMS_DEBUG
    Serial.print  (F("[BRD_R] got "));
    Serial.print  (TOTAL_LEN);
    Serial.println(F(" bytes"));
#endif

    /* ---- 3. CRC check + copy data ---- */
    for (size_t dev = 0; dev < TOTAL_BQ_DEVICES; ++dev) {
        size_t off = dev * FRAME_LEN;

        uint16_t crc_rx  = (rxBuf[off + FRAME_LEN - 1] << 8) |
                            rxBuf[off + FRAME_LEN - 2];
        uint16_t crc_cal = calculateCRC16(&rxBuf[off], FRAME_LEN - CRC_LEN);

#ifdef BMS_DEBUG
        Serial.print  (F("  frame["));
        Serial.print  (dev);
        Serial.print  (F("] INIT 0x"));
        Serial.print  (rxBuf[off], HEX);
        Serial.print  (F(" ADDR 0x"));
        Serial.print  (rxBuf[off+1], HEX);
        Serial.print  (F(" CRC "));
        Serial.println(crc_rx == crc_cal ? F("OK") : F("FAIL"));
#endif

        if (crc_rx != crc_cal) return BMS_ERROR_CRC;

        memcpy(readBuffer + dev * numBytesToRead,
               &rxBuf[off + HDR_LEN],
               numBytesToRead);
    }

#ifdef BMS_DEBUG
    Serial.println(F("[BRD_R] success"));
#endif
    return BMS_OK;
}

BMSErrorCode_t bqStackWrite(uint16_t regAddr, uint64_t data, uint8_t numBytes) {
    return bqWriteReg(0, regAddr, data, numBytes, FRMWRT_STK_W); 
}

BMSErrorCode_t bqStackRead(uint16_t regAddr, uint8_t *readBuffer, uint8_t numBytesPerDevice, uint32_t timeout_ms) {
    if (numBytesPerDevice == 0 || numBytesPerDevice > MAX_READ_DATA_BYTES) return BMS_ERROR_INVALID_RESPONSE;

    BMSErrorCode_t status = buildAndSendFrame(0, regAddr, nullptr, numBytesPerDevice, FRMWRT_STK_R); 
    if (status != BMS_OK) return status;

    size_t singleResponseLength = SINGLE_DEVICE_RESPONSE_OVERHEAD + numBytesPerDevice;
    size_t totalExpectedLength = singleResponseLength * NUM_BQ79616_DEVICES;
    
    if (totalExpectedLength > MAX_STACK_RESPONSE_BUFFER_SIZE) {
        Serial.println("Error: StackRead total expected response too large for buffer.");
        return BMS_ERROR_UNKNOWN;
    }
    uint8_t tempResponseFrame[MAX_STACK_RESPONSE_BUFFER_SIZE]; // Use a single large buffer

    status = receiveFrame(tempResponseFrame, totalExpectedLength, timeout_ms);
    if (status != BMS_OK) return status;

    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        size_t offset = i * singleResponseLength;
        uint16_t received_crc = (uint16_t)(tempResponseFrame[offset + singleResponseLength - 1] << 8) | tempResponseFrame[offset + singleResponseLength - 2];
        uint16_t calculated_crc = calculateCRC16(&tempResponseFrame[offset], singleResponseLength - 2);

        if (received_crc != calculated_crc) {
            BMS_DEBUG_PRINTF("StackRead CRC mismatch for device in response sequence (index %d, reported DevID %d) Reg 0x%04X!\n", i, tempResponseFrame[offset+1], regAddr);
            return BMS_ERROR_CRC;
        }
        // TODO: Verify responseFrame[offset + 0] (INIT byte), responseFrame[offset + 1] (Device ID)
        memcpy(readBuffer + (i * numBytesPerDevice), &tempResponseFrame[offset + 4], numBytesPerDevice);
    }
    return BMS_OK;
}

BMSErrorCode_t bqBroadcastWriteReverse(uint16_t regAddr, uint64_t data, uint8_t numBytes) {
    return bqWriteReg(0, regAddr, data, numBytes, FRMWRT_REV_ALL_W);
}

// This function is used to wake up the BQ79600 bridge device. It is *only* meant to be called during startup either during initialization or after an error. 
// you should probably call a shutdown commands before this one; I won't enforce it here now, but maybe later on if it makes sense. 
BMSErrorCode_t bqWakePing() {
    Serial.println("Waking up BQ79600 bridge...");
    if (BQ_UART_SERIAL) BQ_UART_SERIAL.end(); // If Serial 5 is active, then end it 
    delay(1);
    pinMode(BQ_UART_WAKE_PIN, OUTPUT); // Set the UART line (bit bang) to an output pin
    digitalWrite(BQ_UART_WAKE_PIN, LOW); // then drive it low
    delayMicroseconds(WAKE_PING_DURATION_US); // keep it low for the required duration
    digitalWrite(BQ_UART_WAKE_PIN, HIGH);  // then drive it high to release the BQ79600 from reset
    delay(1); // leave it high for the required duration
    BQ_UART_SERIAL.begin(BQ_UART_BAUDRATE, SERIAL_8N1); // then stop bit banging and start the UART
    BQ_UART_SERIAL.setTimeout(SERIAL_TIMEOUT_MS); // set the timeout for the UART
    delay(1); // Allow UART to settle

    delayMicroseconds(BQ79600_WAKE_TO_ACTIVE_US); 

    BMSErrorCode_t status = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, 0x20, 1, FRMWRT_SGL_W);   // SEND_WAKE (bit 5) = 1
    if (status != BMS_OK) {
        Serial.println("Failed to send WAKE_STACK command to bridge.");
        return status;
    }
    bqDelayUs(STACK_WAKE_PROPAGATION_DELAY_US); // this is set in the config and as per the datasheet. Probably shoudn't fuck with. 
    Serial.println("BQ79600 WAKE ping sent.");


    return BMS_OK; // if this went OK, then we can return BMS_OK (otherwise it will fail and probably time out)
}

BMSErrorCode_t bqAutoAddressStack() {
    Serial.println("Starting auto-addressing sequence...");
    BMSErrorCode_t status;

    // we undergo the "dummy writes" to synchronize the DLL -- this is important if the stack is coming up from complete, dark shutdown. 
    // The original code did something werid with this; i don't care to dig into it much though since the actual implementation seems to mirror this one.
    Serial.println("  DLL Sync (dummy writes)...");
    // we write to each of the ECC registers 1-8
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqBroadcastWrite(reg, 0x00, 1);
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("DLL Sync Write failed for reg 0x%X\n", reg); return status; }
    }
    
    // and then allow them to auto-addresss... (step 4 in datasheet)
    Serial.println("  Enable auto-address mode...");
    status = bqBroadcastWrite(CONTROL1, 0x01, 1); // Set ADDR_WR bit (TODO: Add datasheet reference) 
    if (status != BMS_OK) { Serial.println("Enable auto-address mode failed"); return status; }


    // TODO (and this is a big TODO): There is a special case where we only have one segment board. In this case, we need to set its address to both base and TOS. 
    // The old code does this (and seems to work fine) so that implementation should just be carried over. 

    // step 5
    Serial.println("  Setting device addresses...");
    for (uint8_t i = 0; i < TOTAL_BQ_DEVICES; ++i) { // Bridge (0) + segments (1 to N)
        status = bqBroadcastWriteReverse(DIR0_ADDR, i, 1); 
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Setting address %d failed\n", i); return status; }
    }

    // step 6
    Serial.println("  Configure BQ79616s as stack devices...");
    status = bqBroadcastWrite(COMM_CTRL, 0x02, 1); // STACK_DEV=1, TOP_STACK=0 for all BQ79616s
    if (status != BMS_OK) { Serial.println("Configure stack devices failed"); return status; }

    // step 7
    if (NUM_BQ79616_DEVICES > 0) {
        Serial.println("  Configure Top of Stack device...");
        uint8_t tos_address = NUM_BQ79616_DEVICES; 
        status = bqWriteReg(tos_address, COMM_CTRL, 0x03, 1, FRMWRT_SGL_W); // STACK_DEV=1, TOP_STACK=1
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Configure ToS device (Addr %d) failed\n", tos_address); return status; }
    }

    // Gemini tried to do this but YOU DO NOT NEED TO DO THIS. (and it might mess things up?)
    // // Set BQ79600 (Bridge, device ID 0) as base device
    // status = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, COMM_CTRL, 0x00, 1, FRMWRT_SGL_W); // STACK_DEV=0, TOP_STACK=0
    // if (status != BMS_OK) { Serial.println("Configure bridge as base device failed"); return status; }

    // step 8 in datasheet
    Serial.println("  DLL Sync (dummy reads)...");
    uint8_t dummyReadBuf[MAX_READ_DATA_BYTES]; 
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqStackRead(reg, dummyReadBuf, 1); 
        if (status != BMS_OK && status != BMS_ERROR_CRC && status != BMS_ERROR_COMM_TIMEOUT) { 
             BMS_DEBUG_PRINTF("Warning: Dummy stack read for DLL sync failed for reg 0x%X with status %d\n", reg, status);
             // Not returning error here as per TI guide for dummy reads
        }
    }

    /* ---- Step 9 : verify addresses by stack-reading DIR0_ADDR (0x0306) ---- */
    Serial.println("  Verifying stack device addresses (DIR0_ADDR)...");
    uint8_t addrBuf[NUM_BQ79616_DEVICES];                 // one byte per monitor
    status = bqStackRead(DIR0_ADDR, addrBuf, 1);          // frames come Btm→Top
    if (status != BMS_OK) {
        Serial.println("  Address-verification read failed.");
        return status;                                    // abort auto-address
    }
    /* print what each device reported */
    for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        Serial.print("    Dev "); Serial.print(i + 1);
        Serial.print(" reports 0x"); Serial.println(addrBuf[i], HEX);
    }

    /* ---- Step 10 : read BQ79600 DEV_CONF1 (0x2001) and verify == 0x14 ---- */
    Serial.println("  Verifying BQ79600-Q1 DEV_CONF1 (0x2001)...");
    uint8_t devConf1 = 0;
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, 0x2001, &devConf1, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        Serial.println("    DEV_CONF1 read failed.");
        return status;
    }
    Serial.print ("    DEV_CONF1 = 0x"); Serial.println(devConf1, HEX);
    if (devConf1 != 0x14) {
        Serial.println("    ERROR: DEV_CONF1 mismatch (expected 0x14)");
        return BMS_ERROR_INVALID_RESPONSE;
    }
    
    Serial.println("  Resetting communication faults from auto-addressing...");
    status = bqBroadcastWrite(FAULT_RST2, 0xFF, 1); // Reset all in FAULT_RST2
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST2 failed"); return status; }
    // The original code doesn't do this. I don't know if this needs it, but if it's problematic... y'know... backspace....
    status = bqBroadcastWrite(FAULT_RST1, 0xFF, 1); // Reset all in FAULT_RST1
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST1 failed"); return status; }


    Serial.println("Auto-addressing complete.");
    return BMS_OK;
}

// This function configures the BQ79616 stack with register writes matching the legacy set_registers() logic.
void configure_stack(uint8_t &reg_val_8bit, BMSErrorCode_t &status, bool &retFlag)
{
    retFlag = true;
    Serial.println("Daisy Step 4: Starting detailed BQ79616 stack configurations...");

    // 1. Mask CUST_CRC so config changes don't flag a fault (FAULT_MSK2 = 0x40)
    status = bqStackWrite(FAULT_MSK2, 0x40, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_MSK2 (mask CUST_CRC)"); g_bmsData.communicationFault = true; return; }

    // 2. Mask FAULT_PWR so TSREF_UV doesn't flag a fault (FAULT_MSK1 = 0xFFFE, 2 bytes)
    status = bqWriteReg(0, FAULT_MSK1, 0xFFFE, 2, FRMWRT_STK_W);
    if (status != BMS_OK) { Serial.println("Failed FAULT_MSK1 (mask FAULT_PWR)"); g_bmsData.communicationFault = true; return; }

    // 3. Reset all faults
    status = bqStackWrite(FAULT_RST2, 0xFF, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_RST2"); g_bmsData.communicationFault = true; return; }
    status = bqStackWrite(FAULT_RST1, 0xFF, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_RST1"); g_bmsData.communicationFault = true; return; }

    // 4. Enable TSREF (CONTROL2 = 0x01)
    status = bqStackWrite(CONTROL2, 0x01, 1);
    if (status != BMS_OK) { Serial.println("Failed CONTROL2 (TSREF_EN)"); g_bmsData.communicationFault = true; return; }

    // 5. Configure GPIOs as temp inputs (all = 0x09)
    status = bqStackWrite(GPIO_CONF1, 0x09, 1);
    if (status != BMS_OK) { Serial.println("Failed GPIO_CONF1"); g_bmsData.communicationFault = true; return; }
    status = bqStackWrite(GPIO_CONF2, 0x09, 1);
    if (status != BMS_OK) { Serial.println("Failed GPIO_CONF2"); g_bmsData.communicationFault = true; return; }
    status = bqStackWrite(GPIO_CONF3, 0x09, 1);
    if (status != BMS_OK) { Serial.println("Failed GPIO_CONF3"); g_bmsData.communicationFault = true; return; }
    status = bqStackWrite(GPIO_CONF4, 0x09, 1);
    if (status != BMS_OK) { Serial.println("Failed GPIO_CONF4"); g_bmsData.communicationFault = true; return; }

    // 6. OTUT_THRESH: OV thresh to 80%, UT thresh to 20% (0xDA)
    //status = bqStackWrite(OTUT_THRESH, 0xDA, 1);
    //if (status != BMS_OK) { Serial.println("Failed OTUT_THRESH"); g_bmsData.communicationFault = true; return; }

    // 7. OV_THRESH: Over voltage protection to 4.25V (0x25)
    status = bqStackWrite(OV_THRESH, 0x25, 1);
    if (status != BMS_OK) { Serial.println("Failed OV_THRESH"); g_bmsData.communicationFault = true; return; }

    // 8. UV_THRESH: Under voltage protection to 3.0V (0x24)
    status = bqStackWrite(UV_THRESH, 0x24, 1);
    if (status != BMS_OK) { Serial.println("Failed UV_THRESH"); g_bmsData.communicationFault = true; return; }

    // 9. OVUV_CTRL: voltage controls (0x05)
    status = bqStackWrite(OVUV_CTRL, 0x05, 1);
    if (status != BMS_OK) { Serial.println("Failed OVUV_CTRL"); g_bmsData.communicationFault = true; return; }

    // 10. OTUT_CTRL: temperature controls (0x05)
    status = bqStackWrite(OTUT_CTRL, 0x05, 1);
    if (status != BMS_OK) { Serial.println("Failed OTUT_CTRL"); g_bmsData.communicationFault = true; return; }

    // 11. BAL_CTRL1: balance length to 10s (0x01)
    status = bqStackWrite(BAL_CTRL1, 0x01, 1);
    if (status != BMS_OK) { Serial.println("Failed BAL_CTRL1"); g_bmsData.communicationFault = true; return; }

    // 12. BAL_CTRL2: enables auto balancing (0x31)
    status = bqStackWrite(BAL_CTRL2, 0x31, 1);
    if (status != BMS_OK) { Serial.println("Failed BAL_CTRL2"); g_bmsData.communicationFault = true; return; }

    // 13. ACTIVE_CELL: set all cells to active (ACTIVECHANNELS - 6)
    reg_val_8bit = (uint8_t)(CELLS_PER_SLAVE - 6);
    status = bqStackWrite(ACTIVE_CELL, reg_val_8bit, 1);
    if (status != BMS_OK) { Serial.println("Failed ACTIVE_CELL"); g_bmsData.communicationFault = true; return; }

    // 14. ADC_CONF1: LPF_ON, LPF = 9ms (0x04)
    status = bqStackWrite(ADC_CONF1, 0x04, 1);
    if (status != BMS_OK) { Serial.println("Failed ADC_CONF1"); g_bmsData.communicationFault = true; return; }

    // 15. COMM_TIMEOUT_CONF: sleep after 10s (0x3C)
    status = bqStackWrite(COMM_TIMEOUT_CONF, 0x3C, 1);
    if (status != BMS_OK) { Serial.println("Failed COMM_TIMEOUT_CONF"); g_bmsData.communicationFault = true; return; }

    // 16. Reset all faults again (as in legacy code)
    status = bqStackWrite(FAULT_RST2, 0xFF, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_RST2 (second reset)"); g_bmsData.communicationFault = true; return; }
    status = bqStackWrite(FAULT_RST1, 0xFF, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_RST1 (second reset)"); g_bmsData.communicationFault = true; return; }

    // 17. ADC_CTRL1: start main ADC (0x0E)
    status = bqStackWrite(ADC_CTRL1, 0x0E, 1);
    if (status != BMS_OK) { Serial.println("Failed ADC_CTRL1"); g_bmsData.communicationFault = true; return; }

    // 18. ADC_CTRL2: (0x00)
    status = bqStackWrite(ADC_CTRL2, 0x00, 1);
    if (status != BMS_OK) { Serial.println("Failed ADC_CTRL2"); g_bmsData.communicationFault = true; return; }

    // 19. ADC_CTRL3: (0x06)
    status = bqStackWrite(ADC_CTRL3, 0x06, 1);
    if (status != BMS_OK) { Serial.println("Failed ADC_CTRL3"); g_bmsData.communicationFault = true; return; }

    // 20. FAULT_MSK1: unmask all (0x00)
    status = bqStackWrite(FAULT_MSK1, 0x00, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_MSK1 (unmask)"); g_bmsData.communicationFault = true; return; }

    // 21. FAULT_MSK2: (0x60)
    status = bqStackWrite(FAULT_MSK2, 0x60, 1);
    if (status != BMS_OK) { Serial.println("Failed FAULT_MSK2 (final)"); g_bmsData.communicationFault = true; return; }

    if (g_bmsData.communicationFault)
    {
        Serial.println("Startup Error (Daisy S4): Failure during detailed BQ79616 configuration.");
        return;
    }
    Serial.println("Daisy Step 4: Detailed BQ79616 configurations complete.");

    delayMicroseconds(20);
    retFlag = false;
}

BMSErrorCode_t bqSleepDevices() {
    Serial.println("Commanding all devices to SLEEP...");
    BMSErrorCode_t status_stack, status_bridge;
    // CONTROL1 (0x309 from B0_reg.h) bit 2 = GOTO_SLEEP for BQ79616
    status_stack = bqBroadcastWrite(CONTROL1, (1ULL << 2), 1); 
    if (status_stack != BMS_OK) {
        Serial.println("Failed to command BQ79616 stack to SLEEP.");
    }
    // BQ79600_CONTROL1 (0x0309 from bq79600_reg.h) bit 2 = GOTO_SLEEP for BQ79600
    status_bridge = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, BQ79600_CTRL1_GOTO_SLEEP_BIT, 1, FRMWRT_SGL_W);
     if (status_bridge != BMS_OK) {
        Serial.println("Failed to command BQ79600 bridge to SLEEP.");
    }
    return (status_stack == BMS_OK && status_bridge == BMS_OK) ? BMS_OK : BMS_ERROR_UNKNOWN;
}

BMSErrorCode_t bqShutdownDevices() {
    Serial.println("Commanding all devices to SHUTDOWN...");
    BMSErrorCode_t status_stack, status_bridge;
    // CONTROL1 (0x309 from B0_reg.h) bit 3 = SHUTDOWN_CMD for BQ79616
    status_stack = bqBroadcastWrite(CONTROL1, (1ULL << 3), 1); 
    if (status_stack != BMS_OK) {
        Serial.println("Failed to command BQ79616 stack to SHUTDOWN.");
    }
    // BQ79600_CONTROL1 (0x0309 from bq79600_reg.h) bit 3 = GOTO_SHUTDOWN for BQ79600
    status_bridge = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, BQ79600_CTRL1_GOTO_SHUTDOWN_BIT, 1, FRMWRT_SGL_W);
     if (status_bridge != BMS_OK) {
        Serial.println("Failed to command BQ79600 bridge to SHUTDOWN.");
    }
    return (status_stack == BMS_OK && status_bridge == BMS_OK) ? BMS_OK : BMS_ERROR_UNKNOWN;
}

void bqDelayUs(unsigned int us) {
    delayMicroseconds(us);
}

