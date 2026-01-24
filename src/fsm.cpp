#include "fsm.h"
#include "config.h"
#include "bq_comm.h"
#include "bq_data.h"     
#include "fault_handler.h" 
#include "can_bus.h"
#include "B0_reg.h"      
#include <math.h>        
#include <string.h> 

// Voltage thresholds are defined in config.h; do not redefine here.

FSM_State_t g_currentState;
// BMSOverallData_t g_bmsData; // Definition moved to bq_data.cpp

unsigned long g_lastFsmRunTime = 0;
unsigned long g_lastCanTransmitTime = 0;
unsigned long fault_startup_error_entry_ts = 0;
static unsigned long startup_attempt_timestamp = 0;


void fsm_change_state(FSM_State_t newState) {
    auto fsm_state_to_string = [](FSM_State_t state) -> const char* {
        switch (state) {
            case FSM_STATE_INITIAL: return "INITIAL";
            case FSM_STATE_STARTUP: return "STARTUP";
            case FSM_STATE_NORMAL_OPERATION: return "NORMAL_OPERATION";
            case FSM_STATE_CELL_BALANCING: return "CELL_BALANCING";
            case FSM_STATE_FAULT_COMMUNICATION: return "FAULT_COMMUNICATION";
            case FSM_STATE_FAULT_MEASUREMENT: return "FAULT_MEASUREMENT";
            case FSM_STATE_FAULT_STARTUP_ERROR: return "FAULT_STARTUP_ERROR";
            case FSM_STATE_FAULT_CRITICAL: return "FAULT_CRITICAL";
            default: return "UNKNOWN_STATE";
        }
    };

    if (g_currentState != newState) {
        BMS_DEBUG_PRINTF("FSM: %s -> %s\n", fsm_state_to_string(g_currentState), fsm_state_to_string(newState));
        g_currentState = newState;
    }
}

volatile bool imd_fault_latched = false;
volatile unsigned long imd_isr_trigger_time = 0;

void red_car() {
    imd_fault_latched = true;
    g_bmsData.imd_status_ok = false;
    digitalWrite(IMD_ERROR_PIN, HIGH);
    g_bmsData.balancing_request = false;
    g_bmsData.balancing_cycle_active = false;
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        g_bmsData.activeBalancingCells[i] = 0;
    }
    while (true) {
        // Stay here forever, system is halted due to IMD fault
        // this doesn't update the data from the pack though, so this is kind of dangersou and should be fixed
        can_send_bms_data(&g_bmsData);
        send_pack_data(&g_bmsData);
        delay(1000);
    }
}

void imd_isr() {
    static unsigned long imd_fall_time = 0;
    if (digitalRead(IMD_STATUS_PIN) == LOW) { // Fault asserted
        if (imd_fall_time == 0) {
            imd_fall_time = millis();
        }
        if ((millis() - imd_fall_time) >= 3000) {
            red_car();
        }
    } else {
        imd_fall_time = 0; // Pin released, reset timer
    }
}

void fsm_init() {
    Serial.println("FSM Init");
    memset(&g_bmsData, 0, sizeof(BMSOverallData_t)); // Initialize global BMS data
    initFaultHandler(); 
    bqInitCommunication(); 
    can_init();


    pinMode(FAN_PIN, OUTPUT); digitalWrite(FAN_PIN, LOW);
    pinMode(RESET_PIN, INPUT_PULLDOWN); 
    pinMode(IMD_STATUS_PIN, INPUT_PULLDOWN); // Assuming pull-down if signal is active HIGH for OK
    attachInterrupt(digitalPinToInterrupt(IMD_STATUS_PIN), imd_isr, FALLING);
    pinMode(POS_AIR_STATUS_PIN, INPUT_PULLDOWN);
    pinMode(NEG_AIR_STATUS_PIN, INPUT_PULLDOWN);
    pinMode(IMD_ERROR_PIN, OUTPUT); digitalWrite(IMD_ERROR_PIN, LOW);
    //pinMode(AMS_ERROR_PIN, OUTPUT); digitalWrite(AMS_ERROR_PIN, LOW);

    fsm_change_state(FSM_STATE_INITIAL);
}

void fsm_run() {
    if (millis() - g_lastFsmRunTime < STATE_MACHINE_INTERVAL_MS) {
        return; 
    }
    g_lastFsmRunTime = millis();

    FSM_State_t nextState = g_currentState; 

    readNFaultPin(); 
    readSystemInputs(&g_bmsData); 

    switch (g_currentState) {
        case FSM_STATE_INITIAL: action_initial(); break;
        case FSM_STATE_STARTUP: action_startup(); break;
        case FSM_STATE_NORMAL_OPERATION: action_normal_operation(); break;
        case FSM_STATE_CELL_BALANCING: action_cell_balancing(); break;
        case FSM_STATE_FAULT_COMMUNICATION: action_fault_communication(); break;
        case FSM_STATE_FAULT_MEASUREMENT: action_fault_measurement(); break;
        case FSM_STATE_FAULT_STARTUP_ERROR: action_fault_startup_error(); break;
        case FSM_STATE_FAULT_CRITICAL: action_fault_critical(); break;
        default: Serial.println("FSM: Unknown state in action dispatch!"); break;
    }

    processBMSFaults(&g_bmsData); 

    switch (g_currentState) {
        case FSM_STATE_INITIAL: nextState = transition_initial(); break;
        case FSM_STATE_STARTUP: nextState = transition_startup(); break;
        case FSM_STATE_NORMAL_OPERATION: nextState = transition_normal_operation(); break;
        case FSM_STATE_CELL_BALANCING: nextState = transition_cell_balancing(); break;
        case FSM_STATE_FAULT_COMMUNICATION: nextState = transition_fault_communication(); break;
        case FSM_STATE_FAULT_MEASUREMENT: nextState = transition_fault_measurement(); break;
        case FSM_STATE_FAULT_STARTUP_ERROR: nextState = transition_fault_startup_error(); break;
        case FSM_STATE_FAULT_CRITICAL: nextState = transition_fault_critical(); break;
        default:
            Serial.println("FSM: Unknown state in transition dispatch! Resetting to INITIAL.");
            nextState = FSM_STATE_INITIAL;
            break;
    }

    fsm_change_state(nextState); // Use fsm_change_state to handle print only on change

    if (millis() - g_lastCanTransmitTime >= CAN_TRANSMIT_INTERVAL_MS) {
        if (g_currentState == FSM_STATE_NORMAL_OPERATION || g_currentState == FSM_STATE_CELL_BALANCING || 
            g_currentState == FSM_STATE_FAULT_MEASUREMENT || g_currentState == FSM_STATE_FAULT_COMMUNICATION ||
            g_currentState == FSM_STATE_FAULT_STARTUP_ERROR || g_currentState == FSM_STATE_FAULT_CRITICAL) { 
            can_send_bms_data(&g_bmsData); 
            g_lastCanTransmitTime = millis();
        }
    }

    // --- Always print BMS data at the end of each FSM cycle ---
    send_pack_data(&g_bmsData);
}


// This might end up removed later. I might just reset the entire BMS if something goes wrong. 
// Removed: applyEssentialStackConfigsForRecovery()
// If recovery is needed, transition to STARTUP and attempt full boot sequence again.

void action_initial() { }

FSM_State_t transition_initial() { 
    return FSM_STATE_STARTUP; 
}

void action_startup() {
    Serial.println("Action: STARTUP");
    BMSErrorCode_t status;
    uint8_t reg_val_8bit;
    
    g_bmsData.communicationFault = false; 
    // fault_startup_error_entry_ts is managed by fsm_change_state

    bqShutdownDevices(); // Ensure all devices are in shutdown state before startup
    delay(100); // Allow devices to settle after shutdown

    // The following 2 sections are based on the BQ7961X Daisy Chain Startup Sequence on page 27 of the BQ79600 datasheet
    // Look at the diagram between the two tables to find which one matches the architcture. If the architecture ever switches, you MUST go through the code and comment/uncomment necessary lines as marked

    //CURRENTLY FOLLOWING: Table 7-9

    /* ---------------------------------------------------------------------------------------------- */
    /* [1] WAKE ping                    (Steps 1-3) TABLE 7-10                  (Steps 1-2) Table 7-9 */
    /* ---------------------------------------------------------------------------------------------- */
    Serial.println("[1] Waking up BQ79600 bridge and BQ79616 stack...");
    status = bqWakePing(); 
    if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error: Bridge or stack wakeup failed."); return; }
    delayMicroseconds(10); 

    /* ------------------------------------------------------------------------------------------------------ */
    /* [2] Auto-Addressing                    (Steps 4-14) TABLE 7-10                  (Steps 3-11) Table 7-9 */
    /* ------------------------------------------------------------------------------------------------------ */
    Serial.println("[2] Auto Addressing...");
    status = bqAutoAddressStack(); 
    if (status != BMS_OK) { g_bmsData.communicationFault = true;
        Serial.println("[2] Startup Error: Stack autoaddress failed."); return; }
    delayMicroseconds(10);

    //UBCO Motorsports Debugging has only occurred above this line in the Wake and Autoaddressing parts has not been looked at properly
    //The only edits have been to change the numbers of each step (since we deleted #3 we changed 5-->4 and 4-->3)

    // -----------------------------------------------------------------------------------------------------------
    // all the stuff above this, I think there is already a function for. I need to compare the two but will do it later
    // -----------------------------------------------------------------------------------------------------------

    // 3. Set Registers (Detailed BQ79616 Configurations)
    // TODO: I think this should end up either refactored or rewritten to be more readable. 
    // This is one of those things I want to be able to change one day (and probably evaluate how our .h files are set up)

    Serial.println("[3] Configuring Stack...");
    // status = bqConfigureStackForHeartbeat(); 
    // if (status != BMS_OK) { g_bmsData.communicationFault = true; Serial.println("Startup Error (Daisy S4): Stack heartbeat config failed."); return; }


    // this replaced the pile of gemini code that was here before
    bool retFlag;
    configure_stack(reg_val_8bit, status, retFlag);
    retFlag = false;

    //Adding the MUX COntrol to the control Stack 

    Serial.println(" Configuring MUX S0/S1/S2 (GPIO6/7/8) as Outputs (Default Low)... for MUX control");
    
    // Configure all stack devices with the default state (Output Low)
    for (uint8_t i = 1; i <= NUM_BQ79616_DEVICES; ++i) {
        // Configure GPIO6 (S0) as Output Low (0b101)
        status = bqSetGpioconfig(i, 6, GPIO_OUT_LOW);
        if (status != BMS_OK) { Serial.printf("MUX GPIO6 (S0) config failed %d\n", i); retFlag = true; return; }
        // Configure GPIO7 (S1) as Output Low (0b101)
        status = bqSetGpioconfig(i, 7, GPIO_OUT_LOW);
        if (status != BMS_OK) { Serial.printf("MUX GPIO7 (S1) config failed %d\n", i); retFlag = true; return; }
        // Configure GPIO8 (S2) as Output Low (0b101)
        status = bqSetGpioconfig(i, 8, GPIO_OUT_LOW);
        if (status != BMS_OK) { Serial.printf("MUX GPIO8 (S2) config failed %d\n", i); retFlag = true; return; }
    }

    //return


    // Serial.println("[4.5] Disabling OV/UV and OT/UT faults on all stack devices...");
    // Serial.println("[WARNING!!!!!] THIS IS DANGEROUS! DO NOT USE THIS!");
    // /* Mask-off UT, OT, and UV faults on every stack device (bits 6‒4 = 1) */
    status = bqStackWrite(FAULT_MSK1, 0x70, 1);   // 0x40 | 0x20 | 0x10 = 0x70
    // // REMOVE THIS BEFORE EVER ALLOWING THIS TO CONNECT TO ACTUAL CELLS


    // 4. Final Fault Reset
    Serial.println("[4] Resetting all BQ device faults post-configuration...");
    status = resetAllBQFaults(); 
    if (status != BMS_OK) {
        Serial.println("Startup Warning [5]: Failed to reset BQ faults post-configuration.");
    }

    Serial.println("Startup sequence complete. ");
    g_bmsData.balancing_request = false; 
    g_bmsData.balancing_cycle_active = false;
    for(int i=0; i<NUM_BQ79616_DEVICES; ++i) g_bmsData.activeBalancingCells[i] = 0;

}

FSM_State_t transition_startup() {
    if (g_bmsData.communicationFault) { // If action_startup set this due to a failure
        return FSM_STATE_FAULT_STARTUP_ERROR;
    }
    // If processBMSFaults (called after action_startup) confirms a fault from this startup attempt
    if (g_faultState.bridge_comm_fault_confirmed || g_faultState.comm_fault_confirmed) {
        return FSM_STATE_FAULT_STARTUP_ERROR;
    }
    Serial.println("Startup successful, transitioning to Normal Operation.");
    return FSM_STATE_NORMAL_OPERATION;
}

void action_normal_operation() {
    BMSErrorCode_t status;
    bool current_cycle_comm_ok = true;

    status = bqGetAllCellVoltages(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetAllTemperatures(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;
    send_pack_data(&g_bmsData);
    //status = bqGetStackFaultStatus(&g_bmsData); 
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetBridgeFaultStatus(&g_bmsData); 
    if (status != BMS_OK) current_cycle_comm_ok = false; 
    
    g_bmsData.communicationFault = !current_cycle_comm_ok; // Update based on this cycle's operations

    updatePackStatistics(&g_bmsData); 
    g_bmsData.stateOfCharge_pct = estimateSOC(&g_bmsData); 

    bool any_temp_high = false;
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < TEMP_SENSORS_PER_SLAVE; ++j) {
            if (g_bmsData.modules[i].cellTemperatures[j] > 350) { 
                any_temp_high = true; break;
            }
        }
        if (any_temp_high) break;
    }
    digitalWrite(FAN_PIN, any_temp_high ? HIGH : LOW);
    digitalWrite(IMD_ERROR_PIN, g_bmsData.imd_status_ok ? LOW : HIGH);
    digitalWrite(AMS_ERROR_PIN, g_bmsData.overallFaultStatus ? HIGH : LOW); // AMS_ERROR if overall fault


    float min_v_overall = 5000.0f, max_v_overall = 0.0f;
    bool cells_in_bal_range = false;
    if (NUM_BQ79616_DEVICES > 0) { // Prevent access if no devices
        min_v_overall = g_bmsData.minCellVoltage_mV;
        max_v_overall = g_bmsData.maxCellVoltage_mV;
        // Check if max_v_overall is within balancing range to request balancing
        if (max_v_overall > MIN_VOLTAGE_FOR_BALANCING_MV && max_v_overall < MAX_VOLTAGE_FOR_BALANCING_MV) {
            cells_in_bal_range = true;
        }
    }

    if (cells_in_bal_range && (max_v_overall - min_v_overall > CELL_BALANCE_VOLTAGE_DIFF_THRESHOLD_MV)) {
        if (!g_bmsData.balancing_cycle_active) { // Only request if not already in a cycle
            g_bmsData.balancing_request = true;
        }
    } else {
        g_bmsData.balancing_request = false; 
    }
}

FSM_State_t transition_normal_operation() {
    if (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) {
        return FSM_STATE_FAULT_COMMUNICATION;
    }
    if (g_faultState.ovuv_fault_confirmed || g_faultState.otut_fault_confirmed) {
        return FSM_STATE_FAULT_MEASUREMENT;
    }
    if (g_faultState.unspecified_n_fault_confirmed) {
        Serial.println("Transitioning to FAULT_CRITICAL due to unspecified NFAULT assertion.");
        return FSM_STATE_FAULT_CRITICAL; 
    }
    if (g_bmsData.balancing_request && !g_bmsData.balancing_cycle_active) {
        BMSErrorCode_t status = bqStartCellBalancing(&g_bmsData);
        if (status != BMS_OK) {
            Serial.println("Failed to start cell balancing.");
            // Optionally set a fault or handle error here
        }
        return FSM_STATE_CELL_BALANCING;
    }
    return FSM_STATE_NORMAL_OPERATION;
}

// This function can be dropped directly into your fsm.cpp file.

void action_cell_balancing() {
    Serial.println("Action: CELL_BALANCING (monitoring)");

    BMSErrorCode_t status;
    bool current_cycle_comm_ok = true;
    bool all_balancing_done = true; // Assume true, will be set to false if any module is not done.

    // --- Standard Data Acquisition (Verbatim from your code) ---
    status = bqGetAllCellVoltages(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetAllTemperatures(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    send_pack_data(&g_bmsData);

    status = bqGetStackFaultStatus(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    status = bqGetBridgeFaultStatus(&g_bmsData);
    if (status != BMS_OK) current_cycle_comm_ok = false;

    g_bmsData.communicationFault = !current_cycle_comm_ok;

    updatePackStatistics(&g_bmsData);
    g_bmsData.stateOfCharge_pct = estimateSOC(&g_bmsData);

    // --- Fan and Error Pin Logic (Verbatim from your code) ---
    bool any_temp_high = false;
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        for (int j = 0; j < TEMP_SENSORS_PER_SLAVE; ++j) {
            if (g_bmsData.modules[i].cellTemperatures[j] > 350) { 
                any_temp_high = true; break;
            }
        }
        if (any_temp_high) break;
    }
    digitalWrite(FAN_PIN, any_temp_high ? HIGH : LOW);
    digitalWrite(IMD_ERROR_PIN, g_bmsData.imd_status_ok ? LOW : HIGH);
    digitalWrite(AMS_ERROR_PIN, g_bmsData.overallFaultStatus ? HIGH : LOW);


    // =========================================================================
    // --- Refactored Balancing Status Monitoring using bqStackRead ---
    // =========================================================================
    
    // Buffers to hold the data for the entire stack
    uint8_t bal_stat_all[NUM_BQ79616_DEVICES];
    uint8_t cb_complete_all[NUM_BQ79616_DEVICES * 2]; // 2 bytes per device for CB_COMPLETE1 and CB_COMPLETE2

    // Read BAL_STAT for all stack devices in one command
    status = bqStackRead(BAL_STAT, bal_stat_all, 1);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Error reading BAL_STAT from stack.");
        current_cycle_comm_ok = false; // Mark comms as failed for this cycle
    }

    // Read CB_COMPLETE1 and CB_COMPLETE2 for all stack devices in one command
    // This reads 2 consecutive bytes starting from the CB_COMPLETE1 address (0x0333) for each device.
    status = bqStackRead(CB_COMPLETE1, cb_complete_all, 2);
    if (status != BMS_OK) {
        BMS_DEBUG_PRINTLN("Error reading CB_COMPLETE from stack.");
        current_cycle_comm_ok = false; // Mark comms as failed for this cycle
    }

    // If communication was okay, process and print the collected data
    if (current_cycle_comm_ok) {
        BMS_DEBUG_PRINTLN("--- Balancing Status ---");
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
            
            // Extract data for the current module from the buffers
            uint8_t bal_stat = bal_stat_all[mod_idx];
            uint8_t cb_complete1 = cb_complete_all[mod_idx * 2];
            uint8_t cb_complete2 = cb_complete_all[mod_idx * 2 + 1];

            // Print the status for the module in a compact format
            BMS_DEBUG_PRINTF("  Module %d: BAL_STAT=0x%02X, CB_COMPLETE=0x%02X%02X\n", 
                             mod_idx + 1, bal_stat, cb_complete1, cb_complete2);

            // Check if this module has finished balancing.
            // Balancing is complete for a module when all bits in its CB_COMPLETE registers are 1.
            // This logic correctly handles cases where not all 16 cells are used.
            uint16_t cells_in_use_mask = (uint16_t)((1ULL << CELLS_PER_SLAVE) - 1);
            uint16_t cb_complete_full_word = ((uint16_t)cb_complete1 << 8) | cb_complete2;

            if ((cb_complete_full_word & cells_in_use_mask) != cells_in_use_mask) {
                all_balancing_done = false; // If any module is not done, the whole pack is not done.
            }
        }
    } else {
        // If any of the stack reads failed, we cannot be sure of the status,
        // so we must assume balancing is not complete to be safe.
        all_balancing_done = false;
    }
    
    // Update the global data structure
    g_bmsData.balancing_cycle_active = !all_balancing_done;

    if (all_balancing_done) {
        BMS_DEBUG_PRINTLN("All modules report balancing is complete.");
        // Optionally, you could transition back to NORMAL_OPERATION or CHARGING state here
        // fsm_change_state(FSM_STATE_NORMAL_OPERATION);
    }
}
FSM_State_t transition_cell_balancing() {
    if (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
            if (g_bmsData.activeBalancingCells[mod_idx] != 0) {
                bqStopCellBalancing(&g_bmsData);
                g_bmsData.activeBalancingCells[mod_idx] = 0;
            }
        }
        g_bmsData.balancing_cycle_active = false; 
        g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_COMMUNICATION;
    }
    if (g_faultState.ovuv_fault_confirmed || g_faultState.otut_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) { /* Stop balancing */ }
        g_bmsData.balancing_cycle_active = false; 
        g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_MEASUREMENT;
    }
    if (g_faultState.unspecified_n_fault_confirmed) {
        for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) { /* Stop balancing */ }
        g_bmsData.balancing_cycle_active = false; 
        g_bmsData.balancing_request = false;
        return FSM_STATE_FAULT_CRITICAL;
    }

    // Remove all logic that checks voltage delta or balancing thresholds

    // If still requested (e.g. from normal_op) or a cycle is active, stay in balancing
    if(g_bmsData.balancing_request || g_bmsData.balancing_cycle_active) {
        return FSM_STATE_CELL_BALANCING; 
    }
    // Default to normal operation if no longer requested and no cycle active
    Serial.println("Exiting cell balancing as no longer requested or active.");
    g_bmsData.balancing_request = false;
    g_bmsData.balancing_cycle_active = false;
    return FSM_STATE_NORMAL_OPERATION;
}

void action_fault_communication() {
    Serial.println("Action: FAULT_COMMUNICATION");
    // No more retries or stack re-init here; just wait for transition
}

FSM_State_t transition_fault_communication() {
    static unsigned long last_attempt_time = 0;
    unsigned long now = millis();

    // Every 10 seconds, try to re-enter STARTUP to re-establish communications
    if (last_attempt_time == 0 || (now - last_attempt_time) > 10000) {
        last_attempt_time = now;
        Serial.println("Attempting to re-enter STARTUP from FAULT_COMMUNICATION to re-establish communications.");
        return FSM_STATE_STARTUP;
    }
    return FSM_STATE_FAULT_COMMUNICATION;
}

void action_fault_measurement() {
    Serial.println("Action: FAULT_MEASUREMENT (OV/UV/OT/UT)");
    if(g_faultState.ovuv_fault_confirmed) {
        Serial.println("Confirmed OV/UV Fault Active!");
    }
    if(g_faultState.ovuv_fault_confirmed) {
        Serial.println("Confirmed OV/UV Fault Active!");
    }
    send_pack_data(&g_bmsData); // Still print all cell data
}

FSM_State_t transition_fault_measurement() {
    // this might end up removed, because once NFAULT is triggered, it will latch until the entire system is reset. 
    if (!g_faultState.ovuv_fault_confirmed && !g_faultState.otut_fault_confirmed) {
        Serial.println("Measurement fault condition cleared.");
        return FSM_STATE_NORMAL_OPERATION;
    }
    return FSM_STATE_FAULT_MEASUREMENT; 
}

void action_fault_startup_error() {
    Serial.println("Action: FAULT_STARTUP_ERROR - System initialization failed.");
    delay(2000);
}

FSM_State_t transition_fault_startup_error() {
    unsigned long currentTime = millis();
    if (startup_attempt_timestamp == 0) startup_attempt_timestamp = currentTime; // Should have been set by action_startup

    // Retry startup after a longer delay, or go to critical
    if (currentTime - startup_attempt_timestamp > 5000) { // Retry every 30 seconds
        Serial.println("Attempting to re-enter STARTUP from FAULT_STARTUP_ERROR.");
        // Clear relevant fault flags to allow startup to proceed
        g_faultState.comm_fault_retries = 0; 
        g_faultState.comm_fault_confirmed = false; g_faultState.comm_fault_first_detected_ts = 0;
        g_faultState.bridge_comm_fault_confirmed = false; g_faultState.bridge_comm_first_detected_ts = 0;
        g_bmsData.communicationFault = false; // Reset immediate trigger
        if (g_bmsData.bridge.fault_summary_bridge == 0xFF) g_bmsData.bridge.fault_summary_bridge = 0; // Reset bridge read error flag
        return FSM_STATE_STARTUP;
    }
    return FSM_STATE_FAULT_STARTUP_ERROR; 
}

void action_fault_critical() {
    Serial.println("Action: FAULT_CRITICAL - System Halted. Manual intervention required.");
    processBMSFaults(&g_bmsData); // Ensure faults are processed
    printBQDump(); 
    static bool dump_done = false;
    if(!dump_done) {
        printBQDump();
        dump_done = true;
    }
    delay(10000);
}

FSM_State_t transition_fault_critical() {
    // Check for other fault conditions and transition accordingly
    if (g_faultState.comm_fault_confirmed || g_faultState.bridge_comm_fault_confirmed || g_faultState.stack_heartbeat_fault_confirmed) {
        Serial.println("Exiting CRITICAL FAULT: Communication fault detected, transitioning to FAULT_COMMUNICATION.");
        return FSM_STATE_FAULT_COMMUNICATION;
    }
    if (g_faultState.ovuv_fault_confirmed || g_faultState.otut_fault_confirmed) {
        Serial.println("Exiting CRITICAL FAULT: Measurement fault detected, transitioning to FAULT_MEASUREMENT.");
        return FSM_STATE_FAULT_MEASUREMENT;
    }
    if (g_faultState.comm_fault_retries >= 3) {
        Serial.println("Remaining in CRITICAL FAULT: Max communication retries reached.");
        return FSM_STATE_FAULT_CRITICAL;
    }
    // If no other faults, try to recover to startup
    if (!g_faultState.comm_fault_confirmed && !g_faultState.bridge_comm_fault_confirmed &&
        !g_faultState.stack_heartbeat_fault_confirmed &&
        !g_faultState.ovuv_fault_confirmed && !g_faultState.otut_fault_confirmed) {
        Serial.println("Exiting CRITICAL FAULT: No faults detected, transitioning to STARTUP.");
        return FSM_STATE_STARTUP;
    }
    // Default: remain in critical fault
    return FSM_STATE_FAULT_CRITICAL;
}



/* Removed unused shutdown/system off functions and all references to them */

