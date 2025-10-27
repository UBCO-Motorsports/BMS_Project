#ifndef BQ_COMM_DEFS_H
#define BQ_COMM_DEFS_H

#include "config.h" // For NUM_BQ79616_DEVICES

// Frame Write Types for BQ796xx series
#define FRMWRT_SGL_R 0x00 // single device read
#define FRMWRT_SGL_W 0x10 // single device write
#define FRMWRT_STK_R 0x20 // stack read
#define FRMWRT_STK_W 0x30 // stack write
#define FRMWRT_ALL_R 0x40 // broadcast read
#define FRMWRT_ALL_W 0x50 // broadcast write
#define FRMWRT_REV_ALL_W 0xE0 // broadcast write with reverse order

// BQ79600 Bridge Device ID
#define BQ79600_BRIDGE_DEVICE_ID 0x00

// Max data bytes in a single write/read operation
#define MAX_WRITE_DATA_BYTES 8
#define MAX_READ_DATA_BYTES 128 

// Max response for a single device: 1(INIT) + 1(DEV_ADDR) + 2(REG_ADDR) + DATA(numBytesToRead) + 2(CRC)
#define SINGLE_DEVICE_RESPONSE_OVERHEAD 6 
#define MAX_SINGLE_DEVICE_RESPONSE_SIZE (MAX_READ_DATA_BYTES + SINGLE_DEVICE_RESPONSE_OVERHEAD)

// Buffer size for stack reads
#define MAX_STACK_RESPONSE_BUFFER_SIZE (MAX_SINGLE_DEVICE_RESPONSE_SIZE * NUM_BQ79616_DEVICES)

// Maximum length of a single command frame to be sent
#define MAX_COMMAND_FRAME_SIZE 16 

#endif // BQ_COMM_DEFS_H