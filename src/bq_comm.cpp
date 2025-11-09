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
    Serial.println("  Setting device addresses...");
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


