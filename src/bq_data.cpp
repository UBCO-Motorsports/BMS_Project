#include "bq_data.h"
#include "config.h" 
#include "B0_reg.h"
#include "bq79600_reg.h"
#include <math.h> 
#include <string.h> // For memset


BMSOverallData_t g_bmsData; // Definition of g_bmsData

float resistanceToTemperature(float resistance) {
    if (resistance <= 0.1f) return -273.15f; 
    float log_r_norm = logf(resistance / NTC_R_REF); // logf for float
    float term1 = NTC_A1;
    float term2 = NTC_B1 * log_r_norm;
    float term3 = NTC_C1 * powf(log_r_norm, 2); // powf for float
    float term4 = NTC_D1 * powf(log_r_norm, 3); 

    float sum_terms = term1 + term2 + term3 + term4;
    if (fabsf(sum_terms) < 1e-9) return -273.15; // Avoid division by zero if sum is extremely small

    return (1.0f / sum_terms) - 273.15f; 
}

BMSErrorCode_t bqGetAllCellVoltages(BMSOverallData_t *bmsData) {
    // BMS_DEBUG_PRINTLN("Reading all cell voltages...");
    uint8_t rawVoltageData[MAX_STACK_RESPONSE_BUFFER_SIZE]; 
    
    uint16_t start_addr = VCELL16_HI; 
    
    BMSErrorCode_t status = bqStackRead(start_addr, rawVoltageData, CELLS_PER_SLAVE * 2, SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES * 2);
    // // Print rawVoltageData as hex bytes for debugging
    // for (size_t idx = 0; idx < CELLS_PER_SLAVE * 2 * NUM_BQ79616_DEVICES; ++idx) {
    //     Serial.print(rawVoltageData[idx], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Failed to read cell voltages from stack.");

        return status;
    }

    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < CELLS_PER_SLAVE; ++j) {
            int raw_idx = (i * CELLS_PER_SLAVE * 2) + (j * 2);
            uint8_t msb = rawVoltageData[raw_idx];
            uint8_t lsb = rawVoltageData[raw_idx + 1];
            // BQ79616 voltage data is 16-bit signed, MSB first in memory map (e.g. VCELLx_HI then VCELLx_LO)
            // Stack read concatenates these. If VCELL1_LO is start, then data is CELL1_LO, CELL1_HI, CELL2_LO, CELL2_HI...
            // So lsb is at raw_idx, msb is at raw_idx + 1.
            int16_t raw_val = (int16_t)((msb << 8) | lsb); 

            float voltage_uv = (float)raw_val * 190.73f;
            bmsData->modules[i].cellVoltages[j] = (uint16_t)(voltage_uv / 1000.0f);
        }
    }
    processRawVoltages(bmsData);
    return BMS_OK;
}

BMSErrorCode_t bqGetAllTemperatures(BMSOverallData_t *bmsData) {
    // Reference: legacy code and cell voltage access pattern
    uint8_t rawTempData[(TEMP_SENSORS_PER_SLAVE * 2 + 6) * NUM_BQ79616_DEVICES] = {0};

    BMSErrorCode_t status = bqStackRead(GPIO1_HI, rawTempData, TEMP_SENSORS_PER_SLAVE * 2, SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Failed to read GPIO temperatures from stack.");
        return status;
    }

    // Steinhart-Hart coefficients (use legacy values)
    const float a1 = 0.003354016f;
    const float b1 = 0.000256524f;
    const float c1 = 2.61E-6f;
    const float d1 = 6.33E-8f;
    const float REF_RESISTOR = 10000.0f;
    const float REF_VOLTAGE = 5.0f;

    for (uint16_t cb = 0; cb < NUM_BQ79616_DEVICES; cb++) {
        // Each module's temp data is packed sequentially, just like voltages
        for (int i = 0; i < TEMP_SENSORS_PER_SLAVE; i++) {
            int raw_idx = (cb * TEMP_SENSORS_PER_SLAVE * 2) + (i * 2);
            uint8_t msb = rawTempData[raw_idx];
            uint8_t lsb = rawTempData[raw_idx + 1];
            uint16_t raw_data = ((msb & 0xFF) << 8) | (lsb & 0xFF);
            float temp_voltage = (float)raw_data * 0.15259f;
            if (temp_voltage >= 4800.0f) {
                bmsData->modules[cb].cellTemperatures[i] = 255;
            } else {
                float voltage = temp_voltage / 1000.0f;
                float resistance = (REF_RESISTOR * voltage) / (REF_VOLTAGE - voltage);
                float log_r = logf(resistance / REF_RESISTOR);
                float temp_c = 1.0f / (a1 + b1 * log_r + c1 * powf(log_r, 2) + d1 * powf(log_r, 3)) - 273.15f;
                bmsData->modules[cb].cellTemperatures[i] = (int16_t)(temp_c * 10.0f);
            }
        }
    }
    // Read die temperatures after GPIO temperatures
    // Reading from DIETEMP1_LO means data comes as [LO, HI] byte order
    status = bqStackRead(DIETEMP1_LO, rawTempData, 2 * NUM_BQ79616_DEVICES, SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Failed to read die temperatures from stack.");
        return status;
    }
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        int raw_idx = i * 2; 
        uint8_t lsb = rawTempData[raw_idx];      // LO byte comes first
        uint8_t msb = rawTempData[raw_idx + 1];  // HI byte comes second
        int16_t raw_dietemp = (int16_t)((msb << 8) | lsb);
        bmsData->modules[i].dieTemperature = (int16_t)((float)raw_dietemp * 0.025f * 10.0f);
    }

    processRawTemperatures(bmsData);
    return BMS_OK;
}

BMSErrorCode_t bqGetStackFaultStatus(BMSOverallData_t *bmsData) {
    uint8_t faultSummaryBuffer[NUM_BQ79616_DEVICES]; 

    BMSErrorCode_t status = bqStackRead(FAULT_SUMMARY, faultSummaryBuffer, 1, SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Failed to read stack fault summaries.");
        for(int i=0; i < NUM_BQ79616_DEVICES; ++i) bmsData->modules[i].fault_summary = 0xFF; 
        return status;
    }

    for(int i=0; i < NUM_BQ79616_DEVICES; ++i) {
        bmsData->modules[i].fault_summary = faultSummaryBuffer[i];
        uint8_t deviceAddress = i + 1; 

        if (faultSummaryBuffer[i] != 0) {
            // BMS_DEBUG_PRINTF("Fault summary on BQ79616 module %d (Addr %d): 0x%02X. Reading details...\n", i, deviceAddress, faultSummaryBuffer[i]);
            
            if(bqReadReg(deviceAddress, FAULT_OV1, &(bmsData->modules[i].fault_ov1), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_ov1=0xFF;
            if(bqReadReg(deviceAddress, FAULT_OV2, &(bmsData->modules[i].fault_ov2), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_ov2=0xFF;
            if(bqReadReg(deviceAddress, FAULT_UV1, &(bmsData->modules[i].fault_uv1), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_uv1=0xFF;
            if(bqReadReg(deviceAddress, FAULT_UV2, &(bmsData->modules[i].fault_uv2), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_uv2=0xFF;
            if(bqReadReg(deviceAddress, FAULT_OT, &(bmsData->modules[i].fault_ot), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_ot=0xFF;
            if(bqReadReg(deviceAddress, FAULT_UT, &(bmsData->modules[i].fault_ut), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_ut=0xFF;
            if(bqReadReg(deviceAddress, FAULT_COMM1, &(bmsData->modules[i].fault_comm1), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_comm1=0xFF;
            if(bqReadReg(deviceAddress, FAULT_COMM2, &(bmsData->modules[i].fault_comm2), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_comm2=0xFF;
            if(bqReadReg(deviceAddress, FAULT_COMM3, &(bmsData->modules[i].fault_comm3), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_comm3=0xFF;
            if(bqReadReg(deviceAddress, FAULT_SYS, &(bmsData->modules[i].fault_sys), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_sys=0xFF;
            if(bqReadReg(deviceAddress, FAULT_PROT1, &(bmsData->modules[i].fault_prot1), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_prot1=0xFF;
            if(bqReadReg(deviceAddress, FAULT_PROT2, &(bmsData->modules[i].fault_prot2), 1, FRMWRT_SGL_R) != BMS_OK) bmsData->modules[i].fault_prot2=0xFF;
        } else {
            bmsData->modules[i].fault_ov1 = 0; bmsData->modules[i].fault_ov2 = 0;
            bmsData->modules[i].fault_uv1 = 0; bmsData->modules[i].fault_uv2 = 0;
            bmsData->modules[i].fault_ot = 0; bmsData->modules[i].fault_ut = 0;
            bmsData->modules[i].fault_comm1 = 0; bmsData->modules[i].fault_comm2 = 0; bmsData->modules[i].fault_comm3 = 0;
            bmsData->modules[i].fault_sys = 0;
            bmsData->modules[i].fault_prot1 = 0; bmsData->modules[i].fault_prot2 = 0;
        }
    }
    return BMS_OK;
}

BMSErrorCode_t bqGetBridgeFaultStatus(BMSOverallData_t *bmsData) {
    uint8_t faultRegVal = 0;
    BMSErrorCode_t status_overall = BMS_OK;

    BMSErrorCode_t status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_SUMMARY_REG, &faultRegVal, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Failed to read BQ79600 bridge fault summary.");
        bmsData->bridge.fault_summary_bridge = 0xFF; 
        status_overall = status; // Propagate error
    } else {
        bmsData->bridge.fault_summary_bridge = faultRegVal;
    }

    if (bmsData->bridge.fault_summary_bridge != 0 && bmsData->bridge.fault_summary_bridge != 0xFF) {
        // BMS_DEBUG_PRINTF("Fault summary on BQ79600 bridge: 0x%02X. Reading details...\n", bmsData->bridge.fault_summary_bridge);
        
        status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_COMM1, &faultRegVal, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) bmsData->bridge.fault_comm1_bridge = faultRegVal; else {bmsData->bridge.fault_comm1_bridge = 0xFF; status_overall = status;}
        
        if (bmsData->bridge.fault_comm1_bridge & BQ79600_FLTCOMM1_HB_FAIL_BIT) {
            bmsData->bridge.stackHeartbeatOk = false;
        } else {
            if (status == BMS_OK || !(bmsData->bridge.fault_comm1_bridge & BQ79600_FLTCOMM1_HB_FAIL_BIT) ) bmsData->bridge.stackHeartbeatOk = true;
        }

        status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_COMM2, &faultRegVal, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) bmsData->bridge.fault_comm2_bridge = faultRegVal; else {bmsData->bridge.fault_comm2_bridge = 0xFF; status_overall = status;}
        
        if (bmsData->bridge.fault_comm2_bridge != 0 && bmsData->bridge.fault_comm2_bridge != 0xFF) {
            // BMS_DEBUG_PRINTF("Bridge FAULT_COMM2: 0x%02X\n", bmsData->bridge.fault_comm2_bridge);
            bmsData->bridge.ringPathOk = false; // Any comm2 fault suggests ring path issue
        } else if (status == BMS_OK) { // Only if read was OK and value is 0
            bmsData->bridge.ringPathOk = true;
        }
        // Read other BQ79600 fault registers if needed
        status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_SYS, &faultRegVal, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) bmsData->bridge.fault_sys_bridge = faultRegVal; else {bmsData->bridge.fault_sys_bridge = 0xFF; status_overall = status;}
        status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_PWR, &faultRegVal, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) bmsData->bridge.fault_pwr_bridge = faultRegVal; else {bmsData->bridge.fault_pwr_bridge = 0xFF; status_overall = status;}

    } else if (bmsData->bridge.fault_summary_bridge == 0) { // Summary is 0, clear detailed faults
            bmsData->bridge.stackHeartbeatOk = true;
            bmsData->bridge.ringPathOk = true;
            bmsData->bridge.fault_comm1_bridge = 0;
            bmsData->bridge.fault_comm2_bridge = 0;
            bmsData->bridge.fault_sys_bridge = 0;
            bmsData->bridge.fault_pwr_bridge = 0;
    }
    return status_overall; // Return the first error encountered or BMS_OK
}

void processRawVoltages(BMSOverallData_t *bmsData) {
    bmsData->minCellVoltage_mV = 5000;
    bmsData->maxCellVoltage_mV = 0;
    bmsData->totalPackVoltage_mV = 0;

    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < CELLS_PER_SLAVE; ++j) {
            uint16_t voltage = bmsData->modules[i].cellVoltages[j];
            if (voltage < 1000 || voltage > 5000) continue; // Skip erroneous readings for min/max/total
            bmsData->totalPackVoltage_mV += voltage;
            if (voltage < bmsData->minCellVoltage_mV) bmsData->minCellVoltage_mV = voltage;
            if (voltage > bmsData->maxCellVoltage_mV) bmsData->maxCellVoltage_mV = voltage;
        }
    }
}

void processRawTemperatures(BMSOverallData_t *bmsData) {
    bmsData->minCellTemp_C10 = 20000; // 2000.0 C
    bmsData->maxCellTemp_C10 = -20000; // -2000.0 C

    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < TEMP_SENSORS_PER_SLAVE; ++j) {
            int16_t temp = bmsData->modules[i].cellTemperatures[j];
            if (temp < -1000 || temp > 2000) continue; // Skip erroneous readings for min/max
            if (temp < bmsData->minCellTemp_C10) bmsData->minCellTemp_C10 = temp;
            if (temp > bmsData->maxCellTemp_C10) bmsData->maxCellTemp_C10 = temp;
        }
    }
}

void updatePackStatistics(BMSOverallData_t *bmsData) {
    bmsData->overVoltageFault = (bmsData->maxCellVoltage_mV > MAX_CELL_VOLTAGE_MV);
    bmsData->underVoltageFault = (bmsData->minCellVoltage_mV < MIN_CELL_VOLTAGE_MV && bmsData->minCellVoltage_mV > 500); // Ensure min_mv is somewhat valid
    bmsData->overTemperatureFault = (bmsData->maxCellTemp_C10 > (MAX_CELL_TEMP_C * 10));
    bmsData->underTemperatureFault = (bmsData->minCellTemp_C10 < (MIN_CELL_TEMP_C * 10) && bmsData->minCellTemp_C10 > -1000); // Ensure min_temp is somewhat valid
}

uint8_t estimateSOC(const BMSOverallData_t *bmsData) {
    if (NUM_BQ79616_DEVICES == 0 || CELLS_PER_SLAVE == 0) return 0;
    if (bmsData->totalPackVoltage_mV == 0) return 0; // Avoid division by zero if pack voltage isn't read yet

    float avgCellVoltage_mV = (float)bmsData->totalPackVoltage_mV / (NUM_BQ79616_DEVICES * CELLS_PER_SLAVE);

    if (avgCellVoltage_mV <= SOC_LOOKUP_VOLTAGES_MV[0]) {
        return (uint8_t)SOC_LOOKUP_PERCENTAGES[0];
    }
    if (avgCellVoltage_mV >= SOC_LOOKUP_VOLTAGES_MV[SOC_VOLTAGE_POINTS - 1]) {
        return (uint8_t)SOC_LOOKUP_PERCENTAGES[SOC_VOLTAGE_POINTS - 1];
    }

    for (int i = 0; i < SOC_VOLTAGE_POINTS - 1; ++i) {
        if (avgCellVoltage_mV >= SOC_LOOKUP_VOLTAGES_MV[i] && avgCellVoltage_mV <= SOC_LOOKUP_VOLTAGES_MV[i+1]) {
            float V1 = SOC_LOOKUP_VOLTAGES_MV[i];
            float SOC1 = SOC_LOOKUP_PERCENTAGES[i];
            float V2 = SOC_LOOKUP_VOLTAGES_MV[i+1];
            float SOC2 = SOC_LOOKUP_PERCENTAGES[i+1];
            if (fabsf(V2 - V1) < 1e-3) return (uint8_t)SOC1; 
            return (uint8_t)(SOC1 + (avgCellVoltage_mV - V1) * (SOC2 - SOC1) / (V2 - V1));
        }
    }
    // Fallback, should ideally not be reached if table covers the expected voltage range
    BMS_DEBUG_PRINTF("Warning: Avg cell voltage %.0fmV outside SOC lookup range.\n", avgCellVoltage_mV);
    if (avgCellVoltage_mV > SOC_LOOKUP_VOLTAGES_MV[SOC_VOLTAGE_POINTS -1]) return (uint8_t)SOC_LOOKUP_PERCENTAGES[SOC_VOLTAGE_POINTS -1];
    return (uint8_t)SOC_LOOKUP_PERCENTAGES[0];
}

void readSystemInputs(BMSOverallData_t *bmsData) {
    bmsData->reset_pin_active = (digitalRead(RESET_PIN) == HIGH); 
    bmsData->imd_status_ok = (digitalRead(IMD_STATUS_PIN) == HIGH); 
    bmsData->pos_air_closed = (digitalRead(POS_AIR_STATUS_PIN) == HIGH);
    bmsData->neg_air_closed = (digitalRead(NEG_AIR_STATUS_PIN) == HIGH);
}


BMSErrorCode_t bqStartCellBalancing(BMSOverallData_t *bmsData) {
    // Issue the legacy stack-wide balance command for all modules
    BMSErrorCode_t status = bqWriteReg(0, BAL_CTRL2, 0x33, 1, FRMWRT_STK_W);
    if (status == BMS_OK) {
        bmsData->balancing_cycle_active = true;
        bmsData->balancing_request = false; // Clear request since we're now active
        Serial.println("Stack-wide cell balancing started.");
    } else {
        Serial.println("Failed to start stack-wide cell balancing.");
    }
    return status;
}

BMSErrorCode_t bqStopCellBalancing(BMSOverallData_t *bmsData) {
    // Stop balancing for the entire stack by writing MB_TIMER_CTRL = 0 to all modules
    BMSErrorCode_t status = bqWriteReg(0, MB_TIMER_CTRL, 0x0, 1, FRMWRT_STK_W);
    if (status == BMS_OK) {
        // Optionally clear tracking for all modules
        for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
            bmsData->activeBalancingCells[i] = 0;
        }
        Serial.println("Stack-wide cell balancing stopped.");
    } else {
        Serial.println("Failed to stop stack-wide cell balancing.");
    }
    return status;
}

BMSErrorCode_t bqCheckBalancingStatus(uint8_t moduleIndex, BMSOverallData_t *bmsData) {
    if (moduleIndex >= NUM_BQ79616_DEVICES) return BMS_ERROR_UNKNOWN;
    uint8_t deviceAddress = moduleIndex + 1;
    BMSErrorCode_t status;
    uint8_t bal_stat_val;
    uint8_t cb_complete_buf[2];

    status = bqReadReg(deviceAddress, BAL_STAT, &bal_stat_val, 1, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTF("Failed to read BAL_STAT for module %d\n", moduleIndex);
        bmsData->modules[moduleIndex].bal_stat = 0xFF; // Indicate error
        return status;
    }
    bmsData->modules[moduleIndex].bal_stat = bal_stat_val;

    status = bqReadReg(deviceAddress, CB_COMPLETE1, cb_complete_buf, 2, FRMWRT_SGL_R);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTF("Failed to read CB_COMPLETE1 for module %d\n", moduleIndex);
        bmsData->modules[moduleIndex].cb_complete1 = 0xFFFF; 
        return status;
    }
    // Assuming stack read buffer is MSB first for multi-byte registers
    bmsData->modules[moduleIndex].cb_complete1 = (uint16_t)(cb_complete_buf[0] << 8) | cb_complete_buf[1]; 

    return BMS_OK;
}

void printBQDump() {
    BMS_DEBUG_PRINTLN("\n--- BQ Device Register Dump (FAULT_CRITICAL) ---");
    uint8_t read_val_buf[2]; // Buffer for 1-byte or 2-byte reads
    BMSErrorCode_t status;

    // --- BQ79600 Bridge Device Dump (Address 0x00) ---
    BMS_DEBUG_PRINTF("\nBQ79600 Bridge (Addr 0x00) Registers:\n");
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_SUMMARY_REG, read_val_buf, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_SUMMARY: 0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_SUMMARY: READ FAILED");
    
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_COMM1, read_val_buf, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_COMM1:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_COMM1:   READ FAILED");
    
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_COMM2, read_val_buf, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_COMM2:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_COMM2:   READ FAILED");

    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_SYS, read_val_buf, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_SYS:     0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_SYS:     READ FAILED");

    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_FAULT_PWR, read_val_buf, 1, FRMWRT_SGL_R);
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_PWR:     0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_PWR:     READ FAILED");

    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_DEV_CONF1, read_val_buf, 1, FRMWRT_SGL_R); // Corrected: No _REG
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  DEV_CONF1:     0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  DEV_CONF1:     READ FAILED");
    
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_DEV_CONF2, read_val_buf, 1, FRMWRT_SGL_R); // Corrected: No _REG
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  DEV_CONF2:     0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  DEV_CONF2:     READ FAILED");
    
    status = bqReadReg(BQ79600_BRIDGE_DEVICE_ID, BQ79600_CONTROL1, read_val_buf, 1, FRMWRT_SGL_R); // Corrected: No _REG, use BQ79600_CONTROL1
    if (status == BMS_OK) BMS_DEBUG_PRINTF("  CONTROL1 (0x0309): 0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  CONTROL1 (0x0309): READ FAILED");

    // ... (rest of BQ79616 dump remains the same, uses `read_val_buf` and correct B0_reg.h names) ...
    // Example for CB_COMPLETE1 which is 16-bit
    // status = bqReadReg(deviceAddress, CB_COMPLETE1, read_val_buf, 2, FRMWRT_SGL_R);
    // if (status == BMS_OK) BMS_DEBUG_PRINTF("  CB_COMPLETE1:0x%04X\n", (uint16_t)(read_val_buf[0] << 8) | read_val_buf[1]); else BMS_DEBUG_PRINTLN("  CB_COMPLETE1:READ FAILED");

    for (uint8_t i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        uint8_t deviceAddress = i + 1; 
        BMS_DEBUG_PRINTF("\nBQ79616 Device %d (Addr 0x%02X) Registers:\n", i, deviceAddress);

        status = bqReadReg(deviceAddress, DIR0_ADDR, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  DIR0_ADDR:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  DIR0_ADDR:   READ FAILED");
        // ... (Include all other BQ79616 registers as previously listed, using read_val_buf)
        status = bqReadReg(deviceAddress, DEV_CONF, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  DEV_CONF:    0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  DEV_CONF:    READ FAILED");
        status = bqReadReg(deviceAddress, FAULT_SUMMARY, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_SUMMARY: 0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_SUMMARY: READ FAILED");
        status = bqReadReg(deviceAddress, FAULT_SYS, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_SYS:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_SYS:   READ FAILED");
        status = bqReadReg(deviceAddress, FAULT_COMM1, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_COMM1: 0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_COMM1: READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_COMM2, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_COMM2: 0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_COMM2: READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_OV1, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_OV1:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_OV1:   READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_OV2, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_OV2:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_OV2:   READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_UV1, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_UV1:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_UV1:   READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_UV2, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_UV2:   0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_UV2:   READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_OT, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_OT:    0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_OT:    READ FAILED");

        status = bqReadReg(deviceAddress, FAULT_UT, read_val_buf, 1, FRMWRT_SGL_R);
        if (status == BMS_OK) BMS_DEBUG_PRINTF("  FAULT_UT:    0x%02X\n", read_val_buf[0]); else BMS_DEBUG_PRINTLN("  FAULT_UT:    READ FAILED");

        // Add more registers here...
    }
    BMS_DEBUG_PRINTLN("--- End of BQ Register Dump ---");
}

void send_pack_data(const BMSOverallData_t *d)
{

    // 2) Pack metrics
    Serial.println("\n--- Pack Metrics ---");
    Serial.printf("Total Pack Voltage : %u mV\n", d->totalPackVoltage_mV);
    Serial.printf("Cell Voltage   : min %u mV, max %u mV\n",
                  d->minCellVoltage_mV, d->maxCellVoltage_mV);
    Serial.printf("Cell Temp (°C): min %.1f, max %.1f\n",
                  d->minCellTemp_C10/10.0f, d->maxCellTemp_C10/10.0f);
    Serial.printf("State of Charge  : %u %%\n", d->stateOfCharge_pct);

    // 3) Fault flags
    Serial.println("\n--- Fault Flags ---");
    Serial.printf("Comm Fault        : %s\n", d->communicationFault      ? "YES":"no");
    Serial.printf("Over Voltage      : %s\n", d->overVoltageFault       ? "YES":"no");
    Serial.printf("Under Voltage     : %s\n", d->underVoltageFault      ? "YES":"no");
    Serial.printf("Over Temperature : %s\n", d->overTemperatureFault   ? "YES":"no");
    Serial.printf("Under Temperature: %s\n", d->underTemperatureFault  ? "YES":"no");
    Serial.printf("Overall Fault    : %s\n", d->overallFaultStatus     ? "YES":"no");

    // 4) Balancing & interlocks
    Serial.println("\n--- Balancing & Interlocks ---");
    // for (uint8_t m=0; m<NUM_BQ79616_DEVICES; ++m) {
    //     Serial.printf("Module %u active balance cell: %u\n",
    //                   m, d->activeBalancingCells[m]);
    // }
    Serial.printf("Balancing requested  : %s\n", d->balancing_request      ? "YES":"no");
    Serial.printf("Balancing active     : %s\n", d->balancing_cycle_active? "YES":"no");
    Serial.printf("Reset pin active     : %s\n", d->reset_pin_active      ? "YES":"no");
    Serial.printf("IMD status OK        : %s\n", d->imd_status_ok         ? "YES":"no");
    Serial.printf("Positive air closed  : %s\n", d->pos_air_closed        ? "YES":"no");
    Serial.printf("Negative air closed  : %s\n", d->neg_air_closed        ? "YES":"no");

    // 5) Per-module cell voltages & temps
    for (uint8_t mod = 0; mod < NUM_BQ79616_DEVICES; ++mod)
    {
        Serial.printf("\nModule %u\n", mod);
        // voltages
        Serial.print("Volt :");
        for (uint8_t c = 0; c < CELLS_PER_SLAVE; ++c)
            Serial.printf("%5u", d->modules[mod].cellVoltages[c]);
        Serial.println("  (mV)");
        // temps
        Serial.print("Temp :");
        for (uint8_t c = 0; c < TEMP_SENSORS_PER_SLAVE; ++c)
            Serial.printf("%5.0f", d->modules[mod].cellTemperatures[c]/10.0f);
        Serial.println("  (°C)");
    }
    Serial.println();
}
