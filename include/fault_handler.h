#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include "bms_data_types.h"
#include "config.h"

typedef struct {
    bool comm_fault_confirmed;          
    bool ovuv_fault_confirmed;          
    bool otut_fault_confirmed;          
    bool bridge_comm_fault_confirmed;   
    bool stack_heartbeat_fault_confirmed;
    bool unspecified_n_fault_confirmed; 

    unsigned long comm_fault_first_detected_ts;
    unsigned long ovuv_first_detected_ts;
    unsigned long otut_first_detected_ts;
    unsigned long bridge_comm_first_detected_ts;
    unsigned long stack_heartbeat_first_detected_ts;
    unsigned long n_fault_pin_asserted_ts; 

    uint8_t comm_fault_retries; 

    bool n_fault_pin_raw_status; 

} FaultState_t;

extern FaultState_t g_faultState; // Declare g_faultState as extern here

void initFaultHandler();
void readNFaultPin(); 
void processBMSFaults(BMSOverallData_t *bmsData); 
void updateExternalFaultIndicator(); 
BMSErrorCode_t resetAllBQFaults(); 
void clearSingleFaultConfirmation(const char* faultType); 

#endif // FAULT_HANDLER_H