//
// Created by alice on 3/30/2025.
//

// adbms.c
#include "adbms.h"
#include "spi.h"
#include "gpio.h"

// Default configuration for CFGA and CFGB registers
uint8_t cfga_default[6] = {
    0x81, // REFON = 1 (Reference voltage enabled)
    0x00, // Reserved
    0x00, // Reserved
    0xFF, // Disabling pulldown for GPIOs 1-8
    0x03, // Disabling pulldown for GPIOs 9-10
    0x01  // IIR filter corner frequency set to 110 Hz
};

uint8_t cfgb_default[6] = {
    0x00, // Reserved
    0xF8, // Reserved
    0x7F, // Reserved
    0x00, // DCC (Discharge control bits)
    0x00, // DCC (Discharge control bits)
};

// PEC (Packet Error Code) calculation function
// This function computes a CRC15 checksum for error detection during SPI communication.
static uint16_t pec(const uint8_t *data, int len) {
    uint16_t remainder = 16; // PEC seed (initial value)
    for (int i = 0; i < len; i++) {
        uint16_t address = ((remainder >> 7) ^ data[i]) & 0xff; // Calculate PEC table address
        remainder = (remainder << 8) ^ pec15Table[address];     // Update remainder using lookup table
    }
    return (remainder * 2); // CRC15 has a zero in the LSB, so multiply by 2
}

// Wake up all BMS chips in the daisy chain by sending a dummy command over SPI.
// This ensures that all chips are ready to receive commands.
void adbms6830_wakeup() {
    uint8_t wakeup_cmd = 0xFF; // Dummy command to wake up chips
    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_RESET); // Select BMS SPI device
    HAL_SPI_Transmit(&hspi2, &wakeup_cmd, 1, HAL_MAX_DELAY);                 // Transmit wake-up command
    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);   // Deselect BMS SPI device
}

// Write configuration data to CFGA registers on all BMS chips in the daisy chain.
void adbms6830_wrcfga(uint8_t *cfga) {
    adbms6830_cmd_write(CMD_WRCFGA, cfga); // Send write command with CFGA data buffer
}

// Write configuration data to CFGB registers on all BMS chips in the daisy chain.
// This includes balancing commands for individual cells.
void adbms6830_wrcfgb(uint8_t *cfgb) {
    adbms6830_cmd_write(CMD_WRCFGB, cfgb); // Send write command with CFGB data buffer
}

// Send a command to the ADBMS IC and write data.
// This function handles both the command and data transmission over SPI.
ADBMS6830_Error_t adbms6830_cmd_write(ADBMS6830_Command_t command, uint8_t *data_buf) {
    uint16_t command_int = command;                // Convert command enum to integer value
    uint8_t cmd_buf[2] = {command_int >> 8, command_int & 0xFF}; // Split into high and low bytes

    uint16_t cmd_crc = pec(cmd_buf, sizeof(cmd_buf));            // Calculate PEC for command bytes
    uint8_t crc_buf[2] = {cmd_crc >> 8, cmd_crc & 0xFF};         // Split PEC into high and low bytes

    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_RESET); // Select BMS SPI device

    // Send command
    if (HAL_SPI_Transmit(&hspi2, cmd_buf, sizeof(cmd_buf), LTC_SPI_TIMEOUT) != HAL_OK ||
        HAL_SPI_Transmit(&hspi2, crc_buf, sizeof(crc_buf), LTC_SPI_TIMEOUT) != HAL_OK) {
        HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET); // Deselect BMS SPI device in case of error
        return ADBMS6830_SPI_ERROR;                                           // Return error status
    }

    // Iterate through BMS chips in reverse order (last to first)
    for (int i = NUM_BMS_ICS - 1; i >= 0; i--) {
        uint16_t data_crc = pec(data_buf + (i * 6), 6);
        crc_buf[0] = data_crc >> 8;
        crc_buf[1] = data_crc & 0xFF;

        if (HAL_SPI_Transmit(&hspi2, data_buf + (i * 6), 6, LTC_SPI_TIMEOUT) != HAL_OK ||
            HAL_SPI_Transmit(&hspi2, crc_buf, sizeof(crc_buf), LTC_SPI_TIMEOUT) != HAL_OK) {
            HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);
            return ADBMS6830_SPI_ERROR;
        }
    }

    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);     // Deselect BMS SPI device after successful transmission
    return ADBMS6830_OK;                                                      // Return success status
}

// Read data from ADBMS IC.
// This function sends a read command and retrieves response data from all BMS chips in the daisy chain.
uint32_t adbms6830_cmd_read(ADBMS6830_Command_t command, uint8_t *data_buf) {
    uint16_t command_int = command;                // Convert command enum to integer value
    uint8_t cmd_buf[2] = {command_int >> 8, command_int & 0xFF};              // Split into high and low bytes

    uint16_t cmd_crc = pec(cmd_buf, sizeof(cmd_buf));                         // Calculate PEC for command bytes
    uint8_t crc_buf[2] = {cmd_crc >> 8, cmd_crc & 0xFF};                      // Split PEC into high and low bytes

    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_RESET);   // Select BMS SPI device

    // Send command
    if (HAL_SPI_Transmit(&hspi2, cmd_buf, sizeof(cmd_buf), LTC_SPI_TIMEOUT) != HAL_OK ||
        HAL_SPI_Transmit(&hspi2, crc_buf, sizeof(crc_buf), LTC_SPI_TIMEOUT) != HAL_OK) {
        HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);
        return false;
    }

    uint32_t responsiveChips = NUM_BMS_ICS;                                   // Track number of responsive chips

    // Read from last BMS first
    for (int i = NUM_BMS_ICS - 1; i >= 0; i--) {
        if (HAL_SPI_Receive(&hspi2, data_buf + (i * 6), LTC_SPI_TIMEOUT) != HAL_OK) {
            responsiveChips--;                                               // Decrement responsive chip count if read fails
        }

        /* Verify PEC here if needed */
        /* Example: Compare received CRC with calculated CRC */

        /* Placeholder: Add CRC verification logic */

        /* If CRC verification fails: responsiveChips--; */

        /* End placeholder */

        /* Continue reading remaining chips */
    }

    HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);   // Deselect BMS SPI device
    return responsiveChips;                                                     // Return number of responsive chips
}
