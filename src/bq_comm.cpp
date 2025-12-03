Hmm strange, I like what your on to. Below I've attached the old code which does the auto addressing steps slightly different and with the loop fixed...
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
    uint8_t = conf_pos;


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
    // Assuming the MUX control pins are physically connected to the first BQ79616 (Device ID 1)
    uint8_t deviceID = 1; 

    // Set GPIO6 (S0) configuration
    status = bqSetGpioconfig(deviceID, MUX_S0, s0_state_cfg);
    if (status != BMS_OK) { all_ok = false; }
    // Set GPIO7 (S1) configuration
    status = bqSetGpioconfig(deviceID, MUX_S1, s1_state_cfg);
    if (status != BMS_OK) { all_ok = false; }
    // Set GPIO8 (S2) configuration
    status = bqSetGpioconfig(deviceID, MUX_S2, s2_state_cfg);
    if (status != BMS_OK) { all_ok = false; }
    
    //settle time
    bqDelayUs(100); 

    return all_ok ? BMS_OK : BMS_ERROR_UNKNOWN;
}