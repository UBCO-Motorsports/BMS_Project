#ifndef FLEXCAN_T4_H
#define FLEXCAN_T4_H

#include <cstdint>
#include <cstring>

// Dummy enums and constants
enum CAN_BUS { CAN1, CAN2, CAN3 };
enum RX_SIZE { RX_SIZE_256 = 256 };
enum TX_SIZE { TX_SIZE_16 = 16 };

// Mock CAN Message Struct
struct CAN_message_t {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t buf[8] = {0};
    struct {
        uint8_t extended : 1;
    } flags;
    
    CAN_message_t() { flags.extended = 0; }
};

// Mock FlexCAN Class
template<CAN_BUS bus, int rx_size, int tx_size>
class FlexCAN_T4 {
public:
    void begin() {}
    void setBaudRate(uint32_t baud) {}
    bool write(const CAN_message_t& msg) { return true; } // Pretend write succeeded
    bool read(CAN_message_t& msg) { return false; }       // No messages in dummy queue
};

#endif // FLEXCAN_T4_H