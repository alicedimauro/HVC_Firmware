//
// Created by alice on 3/30/2025.
//

// adbms.h
#ifndef ADBMS_H
#define ADBMS_H

#include <stdint.h>
#include <stdbool.h>

// Number of BMS ICs in the daisy chain
#define NUM_BMS_ICS 1 // Update this based on your setup

// SPI timeout for communication
#define LTC_SPI_TIMEOUT 100

// Commands for ADBMS6830 (based on datasheet)
typedef enum {
    CMD_WRCFGA = 0x001,
    CMD_WRCFGB = 0x024,
    CMD_RDCVA = 0x0004,
    CMD_RDCVB = 0x0006,
    CMD_RDCVC = 0x0008,
    CMD_RDCVD = 0x000A,
    CMD_RDAUXB = 0x000C,
    CMD_RDAUXC = 0x000E,
    CMD_ADCV = 0x0260,
    CMD_ADAX = 0x0460,
    CMD_RDFCA = 0x0010,
    CMD_RDFCB = 0x0012,
    CMD_RDFCC = 0x0014,
    CMD_RDFCD = 0x0016,
    CMD_RDFCE = 0x0018,
    CMD_RDSTATB = 0x0002,
} ADBMS6830_Command_t;

// Error codes for ADBMS communication
typedef enum {
    ADBMS6830_OK = 0,
    ADBMS6830_SPI_ERROR = -1,
} ADBMS6830_Error_t;

// Default configuration arrays (defined in adbms.c)
extern uint8_t cfga_default[6];
extern uint8_t cfgb_default[6];

// Function prototypes
void adbms6830_wakeup(void); // Wake up all BMS chips
void adbms6830_wrcfga(uint8_t *cfga); // Write CFGA configuration
void adbms6830_wrcfgb(uint8_t *cfgb); // Write CFGB configuration
ADBMS6830_Error_t adbms6830_cmd_write(ADBMS6830_Command_t command, uint8_t *data_buf); // Write command and data
uint32_t adbms6830_cmd_read(ADBMS6830_Command_t command, uint8_t *data_buf); // Read data from BMS chips

#endif // ADBMS_H
