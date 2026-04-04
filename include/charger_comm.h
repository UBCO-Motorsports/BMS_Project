#ifndef CHARGER_COMM_H
#define CHARGER_COMM_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct ChargerStatus_t {
    uint16_t outputVoltage_mV;
    uint16_t outputCurrent_mA;
    uint8_t  internalTemp_C;
    uint16_t inputVoltage_V;
    uint16_t inputCurrent_A;
    
    // Status Flags 
    bool hardwareFailure;
    bool overTemp;
    bool inputVoltageAbnormal;
    bool batteryReversePolarity;
    bool commTimeout;
};

extern ChargerStatus_t g_chargerStatus;

void charger_send_command(bool charging_enabled);
void charger_process_message(const CAN_message_t &msg);

#endif