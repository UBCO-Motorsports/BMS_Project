#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <FlexCAN_T4.h> 
#include "bms_data_types.h"
#include "config.h"
#include "fault_handler.h" 

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;

void can_init();
void can_send_bms_data(const BMSOverallData_t *bmsData);
void can_send_detailed_faults(); // Make it available if called separately
void can_process_incoming_messages(); 

#endif // CAN_BUS_H