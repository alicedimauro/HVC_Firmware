//
// Created by alice on 3/30/2025.
//
// cells.h

#ifndef CELLS_H
#define CELLS_H

#include <stdint.h>
#include <stdbool.h>
#include "adbms.h"

// Voltage and temperature thresholds (adjust as needed)
#define PACK_OVER_VOLTAGE 546.0f
#define PACK_UNDER_VOLTAGE 390.0f
#define CELL_OVER_VOLTAGE 4.20f
#define CELL_UNDER_VOLTAGE 3.00f
#define OVER_TEMP 60.0f
#define UNDER_TEMP 0.0f

// Number of cells and temperatures per BMS IC
#define NUM_CELLS_PER_IC 14
#define NUM_TEMPS_PER_IC 5

// Function prototypes
void cellsInit(void); // Initialize cell monitoring system
void cellsPeriodic(int state); // Periodically read cell voltages/temperatures and process them

extern bool balanceCommands[NUM_BMS_ICS * 14]; // balancing commands

//Helper functions
float convertVoltage(uint16_t value);
float convertTemp(float voltage, float referenceVoltage);
void checkMinMaxCells(int cellIndex);
void checkMinMaxTemps(int tempIndex);
void doChecks(int state);
void updateBalanceCommands();
void setDeadCells();

#endif // CELLS_H
