#ifndef ARDUINO_H
#define ARDUINO_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <iostream>
#include <iomanip> // Added for hex formatting
#include <chrono>
#include <thread>
#include <map>

// Arduino Constants
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3
#define FALLING 0x2
#define SERIAL_8N1 0x06

// Print Formatting Constants (THIS FIXES THE ERROR)
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define F(x) x

// Mocking Arduino functions
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
uint8_t digitalPinToInterrupt(uint8_t pin);

// Mocking the Hardware Serial Interface
class MockSerial {
public:
    void begin(unsigned long baud, uint8_t config = SERIAL_8N1) {}
    void end() {}
    void setTimeout(unsigned long timeout) {}
    
    // Print/Write functions
    void print(const char* str) { std::cout << str; }
    
    void print(int val, int format = DEC) { 
        if (format == HEX) std::cout << std::hex << std::uppercase << val << std::dec;
        else std::cout << val; 
    }
    
    void println(const char* str = "") { std::cout << str << std::endl; }
    
    void println(int val, int format = DEC) { 
        if (format == HEX) std::cout << std::hex << std::uppercase << val << std::dec << std::endl;
        else std::cout << val << std::endl; 
    }
    
    template<typename... Args>
    void printf(const char* format, Args... args) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), format, args...);
        std::cout << buffer;
    }
    
    size_t write(const uint8_t *buf, size_t size) { return size; } // Dummy write
    void flush() {}
    
    // Read functions
    int available() { return 0; } // Return 0 by default (no incoming data simulated yet)
    int read() { return -1; }
    
    operator bool() const { return true; }
};

// Global serial instances used by your project
extern MockSerial Serial;
extern MockSerial Serial5; // Used as BQ_UART_SERIAL

#endif // ARDUINO_H