//
// Created by alice on 3/30/2025.
//

#ifndef ADBMS_H
#define ADBMS_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

// Chip Select Pin Configuration (PD9)
#define CS_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_9

// Function Prototype
uint32_t adbms_read_statusa(void);

#endif // ADBMS_H

