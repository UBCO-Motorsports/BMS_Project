#include "Arduino.h"

// Define the global Serial objects
MockSerial Serial;
MockSerial Serial5;

// Mock pin states to allow digitalRead/Write to function
static std::map<uint8_t, uint8_t> mockPins;

// Time tracking
static auto startTime = std::chrono::steady_clock::now();

unsigned long millis() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void pinMode(uint8_t pin, uint8_t mode) {
    // Just initialize the pin in our map if it doesn't exist
    if (mockPins.find(pin) == mockPins.end()) {
        mockPins[pin] = LOW; 
        
        // Emulate default states based on your config.h
        // Note: In a real test, you can change these states via the GUI to test faults
        if (pin == 19) mockPins[pin] = HIGH; // NFAULT_PIN (PULLUP)
        if (pin == 5) mockPins[pin] = HIGH;  // IMD_STATUS (OK)
    }
}

void digitalWrite(uint8_t pin, uint8_t val) {
    mockPins[pin] = val;
}

int digitalRead(uint8_t pin) {
    return mockPins[pin];
}

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) {
    // Dummy: Interrupts are not easily simulated without multithreading, ignored for basic FSM testing
}

uint8_t digitalPinToInterrupt(uint8_t pin) {
    return pin;
}