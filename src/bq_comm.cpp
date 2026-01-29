#include "bq_comm.h"
BMSErrorCode_t bqAutoAddressStack() {
    Serial.println("Starting auto-addressing sequence...");
    BMSErrorCode_t status;


    // we undergo the "dummy writes" to synchronize the DLL -- this is important if the stack is coming up from complete, dark shutdown. 
    // The original code did something werid with this; i don't care to dig into it much though since the actual implementation seems to mirror this one.
    Serial.println("  DLL Sync (dummy writes)...");
    // we write to each of the ECC registers 1-8
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqBroadcastWrite(reg, 0x00, 1);
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("DLL Sync Write failed for reg 0x%X\n", reg); return status; }
    }
    
    // and then allow them to auto-addresss... (step 4 in datasheet)
    Serial.println("  Enable auto-address mode...");
    status = bqBroadcastWrite(CONTROL1, 0x01, 1); // Set ADDR_WR bit (TODO: Add datasheet reference)
    if (status != BMS_OK) { Serial.println("Enable auto-address mode failed"); return status; }



    // TODO (and this is a big TODO): There is a special case where we only have one segment board. In this case, we need to set its address to both base and TOS. 
    // The old code does this (and seems to work fine) so that implementation should just be carried over. 


    // step 5
    Serial.println(" Setting device addresses...");
    for (uint8_t i = 0; i < TOTAL_BQ_DEVICES; ++i) { // Bridge (0) + segments (1 to N)
        status = bqBroadcastWrite(DIR0_ADDR, i, 1); 
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Setting address %d failed\n", i); return status; }
    }


    // step 6
    Serial.println("  Configure BQ79616s as stack devices...");
    status = bqBroadcastWrite(COMM_CTRL, 0x02, 1); // STACK_DEV=1, TOP_STACK=0 for all BQ79616s
    if (status != BMS_OK) { Serial.println("Configure stack devices failed"); return status; }


    // step 7
    if (NUM_BQ79616_DEVICES > 0) {
        Serial.println("  Configure Top of Stack device...");
        uint8_t tos_address = NUM_BQ79616_DEVICES; 
        status = bqWriteReg(tos_address, COMM_CTRL, 0x03, 1, FRMWRT_SGL_W); // STACK_DEV=1, TOP_STACK=1
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Configure ToS device (Addr %d) failed\n", tos_address); return status; }
    }


    // Gemini tried to do this but YOU DO NOT NEED TO DO THIS. (and it might mess things up?)
    // // Set BQ79600 (Bridge, device ID 0) as base device
    // status = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, COMM_CTRL, 0x00, 1, FRMWRT_SGL_W); // STACK_DEV=0, TOP_STACK=0
    // if (status != BMS_OK) { Serial.println("Configure bridge as base device failed"); return status; }


    // step 8 in datasheet
    Serial.println("  DLL Sync (dummy reads)...");
    uint8_t dummyReadBuf[MAX_READ_DATA_BYTES]; 
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqStackRead(reg, dummyReadBuf, 1); 
        if (status != BMS_OK && status != BMS_ERROR_CRC && status != BMS_ERROR_COMM_TIMEOUT) { 
             BMS_DEBUG_PRINTF("Warning: Dummy stack read for DLL sync failed for reg 0x%X with status %d\n", reg, status);
             // Not returning error here as per TI guide for dummy reads
        }
    }


    /* ---- Step 9 : verify addresses by stack-reading DIR0_ADDR (0x0306) ---- */
    Serial.println("  Verifying stack device addresses (DIR0_ADDR)...");
    uint8_t addrBuf[NUM_BQ79616_DEVICES];                 // one byte per monitor
    status = bqStackRead(DIR0_ADDR, addrBuf, 1);          // frames come Btm→Top
    if (status != BMS_OK) {
        Serial.println("  Address-verification read failed.");
        return status;                                    // abort auto-address
    }
    /* print what each device reported */
    for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        Serial.print("    Dev "); Serial.print(i + 1);
        Serial.print(" reports 0x"); Serial.println(addrBuf[i], HEX);
    }


    /* ---- Step 10 : read BQ79600 DEV_CONF1 (0x2001) and verify == 0x14 ---- */
    Serial.println("  Verifying BQ79600-Q1 DEV_CONF1 (0x2001)...");
    uint8_t devConf1 = 0;
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, 0x2001, &devConf1, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        Serial.println("    DEV_CONF1 read failed.");
        return status;
    }
    Serial.print ("    DEV_CONF1 = 0x"); Serial.println(devConf1, HEX);
    if (devConf1 != 0x14) {
        Serial.println("    ERROR: DEV_CONF1 mismatch (expected 0x14)");
        return BMS_ERROR_INVALID_RESPONSE;
    }
    
    Serial.println("  Resetting communication faults from auto-addressing...");
    status = bqBroadcastWrite(FAULT_RST2, 0xFF, 1); // Reset all in FAULT_RST2
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST2 failed"); return status; }
    // The original code doesn't do this. I don't know if this needs it, but if it's problematic... y'know... backspace....
    status = bqBroadcastWrite(FAULT_RST1, 0xFF, 1); // Reset all in FAULT_RST1
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST1 failed"); return status; }



    Serial.println("Auto-addressing complete.");
    return BMS_OK;
}



// New MUX control 

BMSErrorCode_t bqSetGpioconfig(uint8_t deviceID, uint8_t gpioIndex, uint8_t configValue){
    uint16_t = conf_reg;
    uint8_t = conf_val;
    uint8_t = bit_pos;


    // Determine the register and bit position for the 3-bit configuration group
    if (gpioIndex >= 1 && gpioIndex <= 2) {
        conf_reg = GPIO_CONF1;
        bit_pos = (gpioIndex - 1) * 3;
    } else if (gpioIndex >= 3 && gpioIndex <= 4) {
        conf_reg = GPIO_CONF2;
        bit_pos = (gpioIndex - 3) * 3;
    } else if (gpioIndex >= 5 && gpioIndex <= 6) {
        conf_reg = GPIO_CONF3;
        bit_pos = (gpioIndex - 5) * 3;
    } else if (gpioIndex >= 7 && gpioIndex <= 8) {
        conf_reg = GPIO_CONF4;
        // GPIO7 uses bits 5:3, GPIO8 uses bits 2:0
        if (gpioIndex == 7) bit_pos = 3; 
        else bit_pos = 0; 
    } else {
        BMS_DEBUG_PRINTF("Invalid GPIO index %d for configuration.\n", gpioIndex);
        return BMS_ERROR_UNKNOWN;
    }

    // Read current config
    BMSErrorCode_t status = bqReadReg(deviceID, conf_reg, &conf_val, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) return status;

    // Clear the existing 3 bits (0b111) and set the new 3 bits
    conf_val &= ~(0b111 << bit_pos); // Clear bits
    conf_val |= (configValue << bit_pos); // Set new value

    // Write back new config
    return bqWriteReg(deviceID, conf_reg, conf_val, 1, FRMWRT_SGL_W);
}

// New function to set MUX select lines S0, S1, S2 (GPIO6, GPIO7, GPIO8) simultaneously
BMSErrorCode_t bqSetMuxChannel(uint8_t channel) {
    // Channel (0-7) maps to S2(bit 2), S1(bit 1), S0(bit 0)
    // S0 = GPIO6, S1 = GPIO7, S2 = GPIO8
    uint8_t s0_state_cfg = (channel & 0b001) ? GPIO_OUT_HIGH : GPIO_OUT_LOW;
    uint8_t s1_state_cfg = (channel & 0b010) ? GPIO_OUT_HIGH : GPIO_OUT_LOW;
    uint8_t s2_state_cfg = (channel & 0b100) ? GPIO_OUT_HIGH : GPIO_OUT_LOW;

    bool all_ok = true;
    BMSErrorCode_t status;
    //step 4
    // we undergo the "dummy writes" to synchronize the DLL -- this is important if the stack is coming up from complete, dark shutdown. 
    // The original code did something werid with this; i don't care to dig into it much though since the actual implementation seems to mirror this one.
    Serial.println("DLL Sync (dummy writes)...");
    // we write to each of the ECC registers 1-8
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqBroadcastWrite(reg, 0x00, 1);
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("DLL Sync Write failed for reg 0x%X\n", reg); return status; }
    }
    

    //step 5
    Serial.println("Change stack device directions");
    status = bqBroadcastWriteReverse(CONTROL1, 0x80, 1); //write 0x80 (DIR_SEL bit to 1) to CONTROL1
    if (status != BMS_OK) { Serial.println("Reverse stack direction failed"); return status; }

    //step 6
    Serial.println("Write to COMM_CTRL"); 
    status = bqBroadcastWrite(COMM_CTRL, 0x02, 1); //needed after direction reversal to reset top stack device to prevent faults
    if (status != BMS_OK) { Serial.println("Write to COMM_CTRL failed"); return status; }

    //step 7
    Serial.println("Enable auto-adress mode..."); 
    status = bqBroadcastWrite(CONTROL1, 0x81, 1); // Set ADDR_WR and DIR_SEL bits in CONTROL1
    if (status != BMS_OK) { Serial.println("Enable auto-address mode failed"); return status; }

    //step 8
    Serial.println("  Setting device addresses...");
    for (uint8_t i = 0; i < TOTAL_BQ_DEVICES; ++i) { // Bridge (0) + segments (1 to N) [Make sure TOTAL_BQ_DEVICES is correct]
        status = bqBroadcastWrite(DIR1_ADDR, i, 1);  //write to 0x307
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Setting address %d failed\n", i); return status; }
    }

    //step 9
    Serial.println("Configure BQ79616s as stack devices...");
    status = bqBroadcastWrite(COMM_CTRL, 0x02, 1); // STACK_DEV=1, TOP_STACK=0 for all BQ79616s
    if (status != BMS_OK) { Serial.println("Configure stack devices failed"); return status; }

    //step 10
    if (NUM_BQ79616_DEVICES > 0) {
        Serial.println("  Configure Top of Stack device..."); //find which device is top of stack and set it
        uint8_t tos_address = NUM_BQ79616_DEVICES; 
        status = bqWriteReg(tos_address, COMM_CTRL, 0x03, 1, FRMWRT_SGL_W); // STACK_DEV=1, TOP_STACK=1
        if (status != BMS_OK) { BMS_DEBUG_PRINTF("Configure ToS device (Addr %d) failed\n", tos_address); return status; }
    }

    //step 11
    Serial.println("  DLL Sync (dummy reads)...");
    uint8_t dummyReadBuf[MAX_READ_DATA_BYTES]; 
    for (uint16_t reg = OTP_ECC_DATAIN1; reg <= OTP_ECC_DATAIN8; ++reg) { 
        status = bqStackRead(reg, dummyReadBuf, 1); 
        if (status != BMS_OK && status != BMS_ERROR_CRC && status != BMS_ERROR_COMM_TIMEOUT) { 
             BMS_DEBUG_PRINTF("Warning: Dummy stack read for DLL sync failed for reg 0x%X with status %d\n", reg, status);
             // Not returning error here as per TI guide for dummy reads
        }
    }

    //step 12
    Serial.println("  Verifying stack device addresses (DIR0_ADDR)...");
    uint8_t addrBuf[NUM_BQ79616_DEVICES];                 // one byte per monitor
    status = bqStackRead(DIR1_ADDR, addrBuf, 1);          // frames come Btm→Top
    if (status != BMS_OK) {
        Serial.println("  Address-verification read failed.");
        return status;                                    // abort auto-address
    }
    //print device reports
    for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        Serial.print("    Dev "); Serial.print(i + 1);
        Serial.print(" reports 0x"); Serial.println(addrBuf[i], HEX);
    }

    //step 13
    Serial.println("  Verifying BQ79600-Q1 DEV_CONF1 (0x2001)...");
    uint8_t devConf1 = 0;
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, 0x2001, &devConf1, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        Serial.println("    DEV_CONF1 read failed.");
        return status;
    }
    Serial.print ("DEV_CONF1 = 0x"); Serial.println(devConf1, HEX);
    if (devConf1 != 0x14) {
        Serial.println("    ERROR: DEV_CONF1 mismatch (expected 0x14)");
        return BMS_ERROR_INVALID_RESPONSE;
    }

    //not in datasheet but these are in TI's EVM scripts and are good to have to clear transient comms
    Serial.println("  Resetting communication faults from auto-addressing...");
        status = bqBroadcastWrite(FAULT_RST1, 0xFF, 1); // Reset all in FAULT_RST1
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST1 failed"); return status; }
    status = bqBroadcastWrite(FAULT_RST2, 0xFF, 1); // Reset all in FAULT_RST2
    if (status != BMS_OK) { Serial.println("Resetting FAULT_RST2 failed"); return status; }
    //step 14
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

return all_ok ? BMS_OK : BMS_ERROR_UNKNOWN;
