#include "can_bus.h" 
#include "config.h" // For BMS_DEBUG_PRINTLN and CAN IDs
#include "fault_handler.h" // For g_faultState

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;


void can_init() { 
    CanBus.begin();
    CanBus.setBaudRate(CAN_BAUDRATE); 
    BMS_DEBUG_PRINTLN("CAN Bus Initialized.");
}

void can_send_detailed_faults() {
    CAN_message_t fault_msg;
    fault_msg.id = CAN_ID_BMS_FAULTS; 
    fault_msg.flags.extended = 0;
    fault_msg.len = 8; 
    memset(fault_msg.buf, 0, 8); // Initialize buffer

    // Byte 0: Overall confirmed faults
    if (g_bmsData.overVoltageFault) fault_msg.buf[0] |= (1 << 0); // Based on bmsData flags from updatePackStatistics
    if (g_bmsData.underVoltageFault) fault_msg.buf[0] |= (1 << 1); 
    if (g_bmsData.overTemperatureFault) fault_msg.buf[0] |= (1 << 2); 
    if (g_bmsData.underTemperatureFault) fault_msg.buf[0] |= (1 << 3); 
    
    if (g_faultState.comm_fault_confirmed) fault_msg.buf[0] |= (1 << 4);
    if (g_faultState.bridge_comm_fault_confirmed) fault_msg.buf[0] |= (1 << 5);
    if (g_faultState.stack_heartbeat_fault_confirmed) fault_msg.buf[0] |= (1 << 6);
    if (g_faultState.n_fault_pin_raw_status) fault_msg.buf[0] |= (1 << 7); 

    // Byte 1: Aggregated BQ79616 FAULT_SUMMARY bits
    uint8_t aggregated_stack_fault_summary_bits = 0;
    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        aggregated_stack_fault_summary_bits |= g_bmsData.modules[i].fault_summary;
    }
    fault_msg.buf[1] = aggregated_stack_fault_summary_bits;

    // Bytes 2-5: First two faulting modules (device address 1-indexed)
    uint8_t first_faulting_module_id = 0xFF;
    uint8_t first_faulting_module_summary = 0;
    uint8_t second_faulting_module_id = 0xFF;
    uint8_t second_faulting_module_summary = 0;
    bool first_found = false;

    for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
        if (g_bmsData.modules[i].fault_summary != 0) {
            if (!first_found) {
                first_faulting_module_id = i + 1; 
                first_faulting_module_summary = g_bmsData.modules[i].fault_summary;
                first_found = true;
            } else if (second_faulting_module_id == 0xFF) {
                second_faulting_module_id = i + 1;
                second_faulting_module_summary = g_bmsData.modules[i].fault_summary;
                break; // Found two
            }
        }
    }
    fault_msg.buf[2] = first_faulting_module_id;
    fault_msg.buf[3] = first_faulting_module_summary;
    fault_msg.buf[4] = second_faulting_module_id;
    fault_msg.buf[5] = second_faulting_module_summary;
    
    // Bytes 6-7: Reserved
    fault_msg.buf[6] = 0; 
    fault_msg.buf[7] = 0; 

    if(!CanBus.write(fault_msg)) {
        // BMS_DEBUG_PRINTLN("CAN Send Detailed Faults Error!"); // Avoid too much serial print in CAN send
    }
}

void can_send_bms_data(const BMSOverallData_t *bmsData) { 
    if (bmsData == nullptr) return;
    CAN_message_t msg;
    msg.flags.extended = 0; 
    msg.len = 8;

    // --- Send General Status (CAN_ID_BMS_STATUS 0x6B0) ---
    msg.id = CAN_ID_BMS_STATUS; 
    memset(msg.buf, 0, 8);
    
    int16_t pack_current_can = 0;  // Placeholder
    msg.buf[0] = (pack_current_can >> 8) & 0xFF; msg.buf[1] = pack_current_can & 0xFF;

    uint16_t pack_voltage_can = bmsData->totalPackVoltage_mV / 100; 
    msg.buf[2] = (pack_voltage_can >> 8) & 0xFF; msg.buf[3] = pack_voltage_can & 0xFF;
    
    uint8_t soc_can = bmsData->stateOfCharge_pct; 
    msg.buf[4] = soc_can;

    uint8_t general_fault_byte = 0;
    if (g_faultState.ovuv_fault_confirmed) general_fault_byte |= (1 << 0);       
    if (g_faultState.otut_fault_confirmed) general_fault_byte |= (1 << 1);       
    if (g_faultState.comm_fault_confirmed || 
        g_faultState.bridge_comm_fault_confirmed || 
        g_faultState.stack_heartbeat_fault_confirmed) general_fault_byte |= (1 << 2); 
    if (g_faultState.unspecified_n_fault_confirmed) general_fault_byte |= (1 << 3); 
    msg.buf[5] = general_fault_byte;

    msg.buf[6] = (bmsData->maxCellVoltage_mV >> 8) & 0xFF; 
    msg.buf[7] = bmsData->maxCellVoltage_mV & 0xFF;       
    if(!CanBus.write(msg)) { /* BMS_DEBUG_PRINTLN("CAN Send Status Error!"); */ }

    // --- Send Detailed Faults (CAN_ID_BMS_FAULTS 0x6B2) ---
    can_send_detailed_faults(); 

    // --- Send Cell Voltages ---
    for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
        for (uint8_t group_idx = 0; group_idx < CAN_MSGS_PER_MODULE_VOLTS; ++group_idx) {
            msg.id = CAN_ID_CELL_VOLTS_BASE + (mod_idx * CAN_MSGS_PER_MODULE_VOLTS) + group_idx;
            memset(msg.buf, 0xFF, 8); // Default to 0xFFFF for unused slots
            uint8_t cell_start_idx = group_idx * 4;
            for (uint8_t c = 0; c < 4; ++c) {
                uint8_t current_cell_in_module = cell_start_idx + c;
                if (current_cell_in_module < CELLS_PER_SLAVE) {
                    uint16_t voltage_mv = bmsData->modules[mod_idx].cellVoltages[current_cell_in_module];
                    msg.buf[c * 2] = (voltage_mv >> 8) & 0xFF;     
                    msg.buf[c * 2 + 1] = voltage_mv & 0xFF;         
                } 
            }
            if(!CanBus.write(msg)) { /* BMS_DEBUG_PRINTF("CAN Send Voltages Error (Mod %d, Group %d)!\n", mod_idx, group_idx); */ }
        }
    }

    // --- Send Cell Temperatures ---
    for (uint8_t mod_idx = 0; mod_idx < NUM_BQ79616_DEVICES; ++mod_idx) {
        for (uint8_t group_idx = 0; group_idx < CAN_MSGS_PER_MODULE_TEMPS; ++group_idx) {
            msg.id = CAN_ID_CELL_TEMPS_BASE + (mod_idx * CAN_MSGS_PER_MODULE_TEMPS) + group_idx;
            memset(msg.buf, 0x7F, 8); // Default to 0x7FFF for unused/invalid slots
            uint8_t sensor_start_idx = group_idx * 4;
            for (uint8_t s = 0; s < 4; ++s) {
                uint8_t current_sensor_in_module = sensor_start_idx + s;
                if (current_sensor_in_module < TEMP_SENSORS_PER_SLAVE) {
                    int16_t temp_c10 = bmsData->modules[mod_idx].cellTemperatures[current_sensor_in_module];
                    msg.buf[s * 2] = (temp_c10 >> 8) & 0xFF;     
                    msg.buf[s * 2 + 1] = temp_c10 & 0xFF;         
                }
            }
             if(!CanBus.write(msg)) { /* BMS_DEBUG_PRINTF("CAN Send Temps Error (Mod %d, Group %d)!\n", mod_idx, group_idx); */ }
        }
    }
}


// the call to this is commented out. I don't think it will hurt anything to let the mailbox fill since it should be FIFO and the overhead shouldn't be terrible. 
void can_process_incoming_messages() { 
    CAN_message_t msg;
    if (CanBus.read(msg)) {
        BMS_DEBUG_PRINTF("CAN RX ID: 0x%03X, Len: %d, Data: ", msg.id, msg.len);
        for (int i = 0; i < msg.len; ++i) {
            BMS_DEBUG_PRINTF("%02X ", msg.buf[i]);
        }
        BMS_DEBUG_PRINTLN("");
    }
}