#ifndef FSM_H
#define FSM_H

#include "bms_data_types.h"

typedef enum {
    FSM_STATE_INITIAL,
    FSM_STATE_STARTUP,
    FSM_STATE_NORMAL_OPERATION,
    FSM_STATE_CELL_BALANCING,
    FSM_STATE_FAULT_COMMUNICATION,
    FSM_STATE_FAULT_MEASUREMENT,
    FSM_STATE_FAULT_STARTUP_ERROR,
    FSM_STATE_FAULT_CRITICAL,
} FSM_State_t;

extern FSM_State_t g_currentState;
// g_bmsData is now declared extern in bms_data_types.h and defined in bq_data.cpp

void fsm_init();
void fsm_run(); 
void action_initial();
FSM_State_t transition_initial();
void action_startup();
void configure_stack(uint8_t &reg_val_8bit, BMSErrorCode_t &status, bool &retFlag);
FSM_State_t transition_startup();
void action_normal_operation();
FSM_State_t transition_normal_operation();
void action_cell_balancing();
FSM_State_t transition_cell_balancing();
void action_fault_communication();
FSM_State_t transition_fault_communication();
void action_fault_measurement();
FSM_State_t transition_fault_measurement();
void action_fault_startup_error();
FSM_State_t transition_fault_startup_error();
void action_fault_critical();
FSM_State_t transition_fault_critical();


// Helper for NTC voltage conversion
float convertTemperatureToNtcVoltage(float temperatureC);


#endif // FSM_H