//
// Created by alice on 3/30/2025.
//

#include "adbms.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

// Configuration
#define CMD_READ_REG 0x0A      // Read register command opcode
#define REG_STATUSA  0x0000     // STATUSA register address

// SPI Instance (Assumes SPI3 - Check your CubeIDE setup)
extern SPI_HandleTypeDef hspi3;

// Chip Select Pin Configuration (PD9)
#define CS_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_9

// PEC Calculation Function
uint8_t pec8_calc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x41;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Read STATUSA Register
uint32_t adbms_read_statusa(void) {
    uint8_t cmd[4], response[6]; // Response is always 6 bytes
    uint32_t status_data = 0;

    // Build command: [CMD | ADDR | PEC]
    cmd[0] = CMD_READ_REG;
    cmd[1] = (REG_STATUSA >> 8) & 0xFF; // High byte of address
    cmd[2] = REG_STATUSA & 0xFF;        // Low byte of address
    cmd[3] = pec8_calc(cmd, 3);           // Calculate PEC

    // SPI Transaction
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, response, sizeof(response), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);   // CS High

    // Verify PEC
    if (pec8_calc(response, sizeof(response) - 1) != response[5]) {
        return 0xFFFFFFFF; // PEC Error
    }

    // Combine response bytes (STATUSA is 4 bytes, starting at response[1])
    status_data = (response[1] << 24) | (response[2] << 16) | (response[3] << 8) | response[4];

    return status_data;
}
