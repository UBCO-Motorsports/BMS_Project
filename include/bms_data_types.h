#ifndef BMS_DATA_TYPES_H
#define BMS_DATA_TYPES_H

#include "config.h"

typedef enum {
    BMS_OK = 0,
    BMS_ERROR_COMM_TIMEOUT,
    BMS_ERROR_CRC,
    BMS_ERROR_INVALID_RESPONSE,
    BMS_ERROR_WAKEUP_FAILED,
    BMS_ERROR_AUTOADDRESS_FAILED,
    BMS_ERROR_CONFIG_FAILED,
    BMS_ERROR_FAULT_ACTIVE,
    BMS_ERROR_UNKNOWN
} BMSErrorCode_t;

typedef struct {
    uint16_t cellVoltages[CELLS_PER_SLAVE]; 
    int16_t cellTemperatures[TEMP_SENSORS_PER_SLAVE]; 
    uint16_t dieTemperature; 
    
    uint8_t fault_summary;
    uint8_t fault_comm1, fault_comm2, fault_comm3;
    uint8_t fault_ov1, fault_ov2;
    uint8_t fault_uv1, fault_uv2;
    uint8_t fault_ot, fault_ut;
    uint8_t fault_sys;  
    uint8_t fault_prot1; 
    uint8_t fault_prot2; 

    uint8_t bal_stat;      
    uint16_t cb_complete1; 
} BQ79616ModuleData_t;

typedef struct {
    uint8_t fault_summary_bridge;
    uint8_t fault_comm1_bridge;
    uint8_t fault_comm2_bridge;
    uint8_t fault_sys_bridge;
    uint8_t fault_pwr_bridge;
    bool stackHeartbeatOk;
    bool ringPathOk; 
} BQ79600BridgeData_t;


typedef struct {
    BQ79616ModuleData_t modules[NUM_BQ79616_DEVICES];
    BQ79600BridgeData_t bridge;
    
    uint32_t totalPackVoltage_mV;
    int16_t maxCellTemp_C10;
    int16_t minCellTemp_C10;
    uint16_t maxCellVoltage_mV;
    uint16_t minCellVoltage_mV;
    uint8_t stateOfCharge_pct; 

    bool overallFaultStatus; // True if any *confirmed* fault is active
    bool communicationFault; // Immediate flag for current cycle comms issue
    bool overVoltageFault;   // Immediate flag from current readings
    bool underVoltageFault;
    bool overTemperatureFault;
    bool underTemperatureFault;
    
    uint8_t activeBalancingCells[NUM_BQ79616_DEVICES]; 
    bool balancing_request; 
    bool balancing_cycle_active; 

    bool reset_pin_active;
    bool imd_status_ok;
    bool pos_air_closed;
    bool neg_air_closed;

} BMSOverallData_t;

extern BMSOverallData_t g_bmsData; // Declare g_bmsData as extern here

#endif // BMS_DATA_TYPES_H