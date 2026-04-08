#include "charger_comm.h"
#include "can_bus.h"
#include "config.h"

ChargerStatus_t g_chargerStatus;

void charger_send_command(bool charging_enabled) {
    CAN_message_t msg;
    msg.id = CAN_ID_BMS_TO_CHARGER;
    msg.flags.extended = 1; // Elcon uses 29-bit IDs
    msg.len = 8;

    // Byte 1-2: Max Voltage (0.1V/bit) 
    uint16_t v_set = CHARGE_MAX_VOLTAGE_MV / 100; 
    msg.buf[0] = (v_set >> 8) & 0xFF;
    msg.buf[1] = v_set & 0xFF;

    // Byte 3-4: Max Current (0.1A/bit) 
    uint16_t i_set = CHARGE_MAX_CURRENT_MA / 100;
    msg.buf[2] = (i_set >> 8) & 0xFF;
    msg.buf[3] = i_set & 0xFF;

    // Byte 5: Control (0: Start, 1: Stop/Protection) 
    msg.buf[4] = charging_enabled ? 0 : 1;

    // Byte 6: Mode (0: Charging, 1: Heating) 
    msg.buf[5] = 0; 

    // Byte 7-8: Reserved 
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    // --- DEBUG: Print outgoing CAN message ---
    Serial.printf("TX CHG [0x%08X]: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                  msg.id, 
                  msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3], 
                  msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

    CanBus.write(msg);
}

void charger_process_message(const CAN_message_t &msg) {
    if (msg.id != CAN_ID_CHARGER_TO_BMS) return;

    // --- DEBUG: Print incoming CAN message ---
    Serial.printf("RX CHG [0x%08X]: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                  msg.id, 
                  msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3], 
                  msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

    // Byte 1-2: Output Voltage (0.1V/bit) 
    g_chargerStatus.outputVoltage_mV = ((msg.buf[0] << 8) | msg.buf[1]) * 100;

    // Byte 3-4: Output Current (0.1A/bit) 
    g_chargerStatus.outputCurrent_mA = ((msg.buf[2] << 8) | msg.buf[3]) * 100;

    // Byte 5: Status Flags 
    g_chargerStatus.hardwareFailure        = (msg.buf[4] & (1 << 0));
    g_chargerStatus.overTemp               = (msg.buf[4] & (1 << 1));
    g_chargerStatus.inputVoltageAbnormal   = (msg.buf[4] & (1 << 2));
    g_chargerStatus.batteryReversePolarity = (msg.buf[4] & (1 << 3));
    g_chargerStatus.commTimeout            = (msg.buf[4] & (1 << 4));

    // Byte 6: Temperature (offset 100) 
    g_chargerStatus.internalTemp_C = msg.buf[5] - 100;

    // Byte 7: Input Voltage (2V/bit) 
    g_chargerStatus.inputVoltage_V = msg.buf[6] * 2;

    // Byte 8: Input Current (1A/bit) 
    g_chargerStatus.inputCurrent_A = msg.buf[7];
}