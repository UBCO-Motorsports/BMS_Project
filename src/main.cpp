#include <Arduino.h>
#include "config.h"
#include "fsm.h"
#include "can_bus.h" // <-- ADDED THIS INCLUDE

void setup() {
    Serial.begin(115200);
    unsigned long startup_wait = millis();
    while (!Serial && (millis() - startup_wait < 5000)); 
    BMS_DEBUG_PRINTLN("\n\nBMS Controller Starting...");

    fsm_init(); 

    BMS_DEBUG_PRINTLN("Setup Complete. Starting FSM.");
}

void loop() {
    fsm_run();
    //can_process_incoming_messages(); // Now declared via can_bus.h
}