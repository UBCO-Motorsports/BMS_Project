#include "fault_handler.h"
#include "bq_comm.h"
#include "B0_reg.h"
#include "bq79600_reg.h"
#include <string.h> // For strcmp

FaultState_t g_faultState; // Definition of g_faultState

void initFaultHandler() {
    memset(&g_faultState, 0, sizeof(FaultState_t));
    pinMode(NFAULT_PIN, INPUT_PULLUP); 
    pinMode(AMS_FAULT_PIN, OUTPUT);
    digitalWrite(AMS_FAULT_PIN, HIGH); 
}

void readNFaultPin() {
    g_faultState.n_fault_pin_raw_status = (digitalRead(NFAULT_PIN) == LOW);

    if (g_faultState.n_fault_pin_raw_status) {
        if (g_faultState.n_fault_pin_asserted_ts == 0) { 
            g_faultState.n_fault_pin_asserted_ts = millis();
            // BMS_DEBUG_PRINTLN("NFAULT pin asserted (LOW).");
        }
    } else {
        if (g_faultState.n_fault_pin_asserted_ts != 0) { 
            // BMS_DEBUG_PRINTLN("NFAULT pin released (HIGH).");
        }
        g_faultState.n_fault_pin_asserted_ts = 0; 
        if(g_faultState.unspecified_n_fault_confirmed) { // If it was confirmed and pin released, clear it.
             BMS_DEBUG_PRINTLN("Clearing unspecified NFAULT due to pin release.");
             g_faultState.unspecified_n_fault_confirmed = false;
        }
    }
}

void processBMSFaults(BMSOverallData_t *bmsData) {
    unsigned long currentTime = millis();
    bool prev_overall_fault = g_bmsData.overallFaultStatus;

    // --- Communication Faults (from bmsData->communicationFault) ---
    if (bmsData->communicationFault) { 
        if (g_faultState.comm_fault_first_detected_ts == 0) {
            g_faultState.comm_fault_first_detected_ts = currentTime;
        }
        if (!g_faultState.comm_fault_confirmed && (g_faultState.comm_fault_first_detected_ts != 0) && (currentTime - g_faultState.comm_fault_first_detected_ts > COMM_FAULT_CONFIRMATION_MS)) {
            g_faultState.comm_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: BQ Stack Communication Fault!");
        }
    } else {
        if (g_faultState.comm_fault_first_detected_ts != 0) { 
            // BMS_DEBUG_PRINTLN("BQ Stack Comm condition cleared.");
        }
        g_faultState.comm_fault_first_detected_ts = 0;
        if (g_faultState.comm_fault_confirmed) { // Only clear if it was confirmed
            BMS_DEBUG_PRINTLN("Clearing confirmed BQ Stack Comm Fault.");
            g_faultState.comm_fault_confirmed = false; 
        }
    }

    // --- Bridge Communication Faults (bridge.fault_summary_bridge == 0xFF indicates read failure) ---
    if (bmsData->bridge.fault_summary_bridge == 0xFF) { 
         if (g_faultState.bridge_comm_first_detected_ts == 0) {
            g_faultState.bridge_comm_first_detected_ts = currentTime;
        }
        if (!g_faultState.bridge_comm_fault_confirmed && (g_faultState.bridge_comm_first_detected_ts != 0) && (currentTime - g_faultState.bridge_comm_first_detected_ts > COMM_FAULT_CONFIRMATION_MS)) {
            g_faultState.bridge_comm_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: Bridge Communication Fault!");
        }
    } else {
        if (g_faultState.bridge_comm_first_detected_ts != 0) {
            // BMS_DEBUG_PRINTLN("Bridge Comm (read error) condition cleared.");
        }
        g_faultState.bridge_comm_first_detected_ts = 0;
        if(g_faultState.bridge_comm_fault_confirmed){
            BMS_DEBUG_PRINTLN("Clearing confirmed Bridge Comm Fault.");
            g_faultState.bridge_comm_fault_confirmed = false;
        }
    }

    // --- Stack Heartbeat Fault ---
    if (!bmsData->bridge.stackHeartbeatOk) { // stackHeartbeatOk is false if HB missed or comms to read it failed
        if (g_faultState.stack_heartbeat_first_detected_ts == 0) {
            g_faultState.stack_heartbeat_first_detected_ts = currentTime;
        }
        if (!g_faultState.stack_heartbeat_fault_confirmed && (g_faultState.stack_heartbeat_first_detected_ts != 0) && (currentTime - g_faultState.stack_heartbeat_first_detected_ts > COMM_FAULT_CONFIRMATION_MS)) { 
            g_faultState.stack_heartbeat_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: Stack Heartbeat Fault!");
        }
    } else {
        if (g_faultState.stack_heartbeat_first_detected_ts != 0) {
            // BMS_DEBUG_PRINTLN("Stack Heartbeat issue cleared.");
        }
        g_faultState.stack_heartbeat_first_detected_ts = 0;
        if(g_faultState.stack_heartbeat_fault_confirmed){
            BMS_DEBUG_PRINTLN("Clearing confirmed Stack Heartbeat Fault.");
            g_faultState.stack_heartbeat_fault_confirmed = false;
        }
    }

    // --- OV/UV Faults ---
    bool current_ovuv_condition = bmsData->overVoltageFault || bmsData->underVoltageFault;
    if (current_ovuv_condition) {
        if (g_faultState.ovuv_first_detected_ts == 0) {
            g_faultState.ovuv_first_detected_ts = currentTime;
        }
        if (!g_faultState.ovuv_fault_confirmed && (g_faultState.ovuv_first_detected_ts != 0) && (currentTime - g_faultState.ovuv_first_detected_ts > VOLTAGE_FAULT_CONFIRMATION_MS)) {
            g_faultState.ovuv_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: OV/UV Fault!");
        }
    } else {
        if (g_faultState.ovuv_first_detected_ts != 0) {
            // BMS_DEBUG_PRINTLN("OV/UV condition cleared.");
        }
        g_faultState.ovuv_first_detected_ts = 0;
        if(g_faultState.ovuv_fault_confirmed){
            BMS_DEBUG_PRINTLN("Clearing confirmed OV/UV Fault.");
            g_faultState.ovuv_fault_confirmed = false;
        }
    }

    // --- OT/UT Faults ---
    bool current_otut_condition = bmsData->overTemperatureFault || bmsData->underTemperatureFault;
    if (current_otut_condition) {
        if (g_faultState.otut_first_detected_ts == 0) {
            g_faultState.otut_first_detected_ts = currentTime;
        }
        if (!g_faultState.otut_fault_confirmed && (g_faultState.otut_first_detected_ts != 0) && (currentTime - g_faultState.otut_first_detected_ts > TEMPERATURE_FAULT_CONFIRMATION_MS)) {
            g_faultState.otut_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: OT/UT Fault!");
        }
    } else {
        if (g_faultState.otut_first_detected_ts != 0) {
            // BMS_DEBUG_PRINTLN("OT/UT condition cleared.");
        }
        g_faultState.otut_first_detected_ts = 0;
        if(g_faultState.otut_fault_confirmed){
            BMS_DEBUG_PRINTLN("Clearing confirmed OT/UT Fault.");
            g_faultState.otut_fault_confirmed = false;
        }
    }
    
    bool any_specific_fault_confirmed_now = g_faultState.comm_fault_confirmed ||
                                     g_faultState.ovuv_fault_confirmed ||
                                     g_faultState.otut_fault_confirmed ||
                                     g_faultState.bridge_comm_fault_confirmed ||
                                     g_faultState.stack_heartbeat_fault_confirmed;

    if (g_faultState.n_fault_pin_raw_status && g_faultState.n_fault_pin_asserted_ts != 0) {
        if (!any_specific_fault_confirmed_now && 
            !g_faultState.unspecified_n_fault_confirmed &&
            (currentTime - g_faultState.n_fault_pin_asserted_ts > NFAULT_PIN_ASSERTION_TIMEOUT_MS)) {
            g_faultState.unspecified_n_fault_confirmed = true;
            BMS_DEBUG_PRINTLN("Confirmed: Unspecified NFAULT Assertion (Timeout)!");
        }
    }
    if (any_specific_fault_confirmed_now && g_faultState.unspecified_n_fault_confirmed) {
        BMS_DEBUG_PRINTLN("Clearing unspecified NFAULT because a specific fault is now confirmed.");
        g_faultState.unspecified_n_fault_confirmed = false; // Specific fault takes precedence
    }
    
    updateExternalFaultIndicator(); 
    if (g_bmsData.overallFaultStatus != prev_overall_fault) {
        BMS_DEBUG_PRINTF("Overall BMS Fault Status changed to: %s\n", g_bmsData.overallFaultStatus ? "FAULT" : "OK");
    }
}

void updateExternalFaultIndicator() {
    bool any_confirmed_fault = g_faultState.comm_fault_confirmed ||
                               g_faultState.ovuv_fault_confirmed ||
                               g_faultState.otut_fault_confirmed ||
                               g_faultState.bridge_comm_fault_confirmed ||
                               g_faultState.stack_heartbeat_fault_confirmed ||
                               g_faultState.unspecified_n_fault_confirmed;
    
    digitalWrite(AMS_FAULT_PIN, any_confirmed_fault ? LOW : HIGH);
    digitalWrite(AMS_ERROR_PIN, any_confirmed_fault ? LOW : HIGH);
    g_bmsData.overallFaultStatus = any_confirmed_fault;
}

BMSErrorCode_t resetAllBQFaults() {
    BMS_DEBUG_PRINTLN("Resetting all BQ device faults...");
    BMSErrorCode_t status_stack1, status_stack2, status_bridge;
    
    status_stack1 = bqBroadcastWrite(FAULT_RST1, 0xFF, 1); 
    status_stack2 = bqBroadcastWrite(FAULT_RST2, 0xFF, 1);  // reset stack device faults
    status_bridge = bqWriteReg(BQ79600_BRIDGE_DEVICE_ID, 0x2030, 0xFF, 1, FRMWRT_SGL_W);  // Use RST_ALL_FLGS bit
    
    if(status_stack1 != BMS_OK) BMS_DEBUG_PRINTLN("Failed to reset FAULT_RST1");
    if(status_stack2 != BMS_OK) BMS_DEBUG_PRINTLN("Failed to reset FAULT_RST2");
    if(status_bridge != BMS_OK) BMS_DEBUG_PRINTLN("Failed to reset BQ79600_FAULT_RST");
    
    BMS_DEBUG_PRINTLN("BQ Faults reset commands sent. Clearing local confirmed software fault flags.");
    g_faultState.comm_fault_confirmed = false; g_faultState.comm_fault_first_detected_ts = 0; 
    // Keep comm_fault_retries, manage it in the FAULT_COMM state
    g_faultState.ovuv_fault_confirmed = false; g_faultState.ovuv_first_detected_ts = 0;
    g_faultState.otut_fault_confirmed = false; g_faultState.otut_first_detected_ts = 0;
    g_faultState.bridge_comm_fault_confirmed = false; g_faultState.bridge_comm_first_detected_ts = 0;
    g_faultState.stack_heartbeat_fault_confirmed = false; g_faultState.stack_heartbeat_first_detected_ts = 0;
    g_faultState.unspecified_n_fault_confirmed = false; 
    updateExternalFaultIndicator(); 
    return (status_stack1 == BMS_OK && status_stack2 == BMS_OK && status_bridge == BMS_OK) ? BMS_OK : BMS_ERROR_UNKNOWN;
}

void clearSingleFaultConfirmation(const char* faultType) {
    BMS_DEBUG_PRINTF("Clearing confirmed software fault: %s\n", faultType);
    if (strcmp(faultType, "comm") == 0) {
        g_faultState.comm_fault_confirmed = false;
        g_faultState.comm_fault_first_detected_ts = 0;
    } else if (strcmp(faultType, "ovuv") == 0) {
        g_faultState.ovuv_fault_confirmed = false;
        g_faultState.ovuv_first_detected_ts = 0;
    } else if (strcmp(faultType, "otut") == 0) {
        g_faultState.otut_fault_confirmed = false;
        g_faultState.otut_first_detected_ts = 0;
    } else if (strcmp(faultType, "bridge_comm") == 0) {
        g_faultState.bridge_comm_fault_confirmed = false;
        g_faultState.bridge_comm_first_detected_ts = 0;
    } else if (strcmp(faultType, "heartbeat") == 0) {
        g_faultState.stack_heartbeat_fault_confirmed = false;
        g_faultState.stack_heartbeat_first_detected_ts = 0;
    } else if (strcmp(faultType, "unspec_nfault") == 0) {
        g_faultState.unspecified_n_fault_confirmed = false;
    }
    updateExternalFaultIndicator();
}