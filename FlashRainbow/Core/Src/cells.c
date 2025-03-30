//
// Created by alice on 3/29/2025.
//

// Reads cell voltages and temperatures, performing checks, and managing CAN communication.


// cells.c
#include "cells.h"
#include "adbms.h"
#include "night_can.h" // Include CAN communication library

// Voltage and temperature thresholds (adjust as needed)
float PACK_OVER_VOLTAGE = 546.0f;   // Pack over voltage threshold
float PACK_UNDER_VOLTAGE = 390.0f;  // Pack under voltage threshold
float CELL_OVER_VOLTAGE = 4.20f;    // Cell over voltage threshold
float CELL_UNDER_VOLTAGE = 3.00f;   // Cell under voltage threshold
float OVER_TEMP = 60.0f;            // Over temperature threshold
float UNDER_TEMP = 0.0f;            // Under temperature threshold

// Commands for ADBMS6830 to read different voltage and temperature registers
static ADBMS6830_Command_t CMD_RDCs[9] = {
    CMD_RDFCA, CMD_RDFCB, CMD_RDFCC, CMD_RDFCD, CMD_RDFCE, // Voltage registers
    CMD_RDAUXB, CMD_RDAUXC, CMD_RDAUXD                      // Temperature registers
};

// CAN outboxes to store cell voltage and temperature data
static CanOutbox cellVoltages[35]; // CAN outboxes for cell voltages
static CanOutbox cellTemps[13];   // CAN outboxes for cell temperatures

static uint32_t responsiveChips = 0; // Number of ADBMS chips responding

bool balanceCommands[NUM_BMS_ICS * 14] = {false}; // balancing commands

static float voltageData[NUM_BMS_ICS * 14]; // holds voltages
static float tempData[NUM_BMS_ICS * 5]; // holds temps

float statVreg[NUM_BMS_ICS];

/**
 * @brief Reads voltage and temperature data from BMS.
 *        Converts and writes data into respective CanOutboxes.
 * @param state Current state of the state machine.
 */
void cellsPeriodic(int state) {
    static int divider = 0; // Divider to control the frequency of execution

    divider++;

    // Only execute the following code every 10 iterations
    if (divider == 10) {
        divider = 0; // Reset divider
    } else {
        return; // Skip execution
    }

    static int cmd_ID = -1;      // Tracks current ADBMS command being executed
    static uint16_t value = 0;   // Temporary storage for ADC value
    static uint8_t rawData[8 * 6]; // Buffer to hold raw data from ADBMS

    // Wake up the ADBMS6830 chips to ensure they are ready for communication
    adbms6830_wakeup();

    // Initialize ADBMS commands
    if (cmd_ID == -1) {
        // Write default configuration to CFGA register on all chips
        uint8_t cfga[NUM_BMS_ICS * 6];
        for (int i = 0; i < NUM_BMS_ICS; i++) {
            for (int j = 0; j < 6; j++) {
                cfga[i * 6 + j] = cfga_default[j];
            }
        }
        adbms6830_wrcfga(cfga);

        // Write balancing commands based on balanceCommands array
        uint8_t cfgb[NUM_BMS_ICS * 6];
        for (int i = 0; i < NUM_BMS_ICS; i++) {
            for (int j = 0; j < 6; j++) {
                cfgb[i * 6 + j] = cfgb_default[j];
            }
        }
        adbms6830_wrcfgb(cfgb);
        cmd_ID = 0; // Set command ID to start at the beginning of data collection
    }

        // Poll status registers
    else if (cmd_ID == 5) {
        // Start ADC conversion for auxiliary registers (temperature)
        adbms6830_adax();

        // Read status register B to get temperature readings
        responsiveChips = adbms6830_cmd_read(CMD_RDSTATB, rawData);

    }

    // Start ADC conversion for cell voltages
    if (cmd_ID >= 0 && cmd_ID <= 4) {
        // Start ADC conversion for cell voltages
        adbms6830_adcv();

        // Read cell voltage data from ADBMS chips
        responsiveChips = adbms6830_cmd_read(CMD_RDCs[cmd_ID], rawData);

        for (int j = 0; j < responsiveChips; j++) {
            if (cmd_ID == 4) {
                for (int k = 0; k < 2; k++) {
                    // Combine high and low bytes to form the ADC value
                    value = (rawData[j * 6 + k * 2 + 1] << 8) | rawData[j * 6 + k * 2];
                    // Convert ADC value to voltage and store
                    voltageData[cmd_ID * 3 + j * 14 + k] = convertVoltage(value);
                }
            } else {
                for (int k = 0; k < 3; k++) {
                    // Combine high and low bytes to form the ADC value
                    value = (rawData[j * 6 + k * 2 + 1] << 8) | rawData[j * 6 + k * 2];
                    // Convert ADC value to voltage and store
                    voltageData[cmd_ID * 3 + j * 14 + k] = convertVoltage(value);
                }
            }
        }
    }

    // Temperature readings
    if (cmd_ID >= 5 && cmd_ID <= 7) {
        // Read auxiliary data (temperature) from ADBMS chips
        responsiveChips = adbms6830_cmd_read(CMD_RDCs[cmd_ID], rawData);

        // Process temperature data from each chip
        for (int j = 0; j < responsiveChips; j++) {
            if (cmd_ID == 5) {
                // Read temperature value from Auxiliary Register Group B
                value = (rawData[j * 6 + 5] << 8) | rawData[j * 6 + 4];
                tempData[j * 5] = convertTemp(convertVoltage(value), statVreg[j]);
            }

            if (cmd_ID == 6) {
                for (int k = 0; k < 3; k++) {
                    // Read temperature values from Auxiliary Register Group C
                    value = (rawData[j * 6 + k * 2 + 1] << 8) | rawData[j * 6 + k * 2];
                    tempData[j * 5 + k + 1] = convertTemp(convertVoltage(value), statVreg[j]);
                }
            }

            if (cmd_ID == 7) {
                // Read temperature value from Auxiliary Register Group D
                value = (rawData[j * 6 + 1] << 8) | rawData[j * 6];
                tempData[j * 5 + 4] = convertTemp(convertVoltage(value), statVreg[j]);
            }
        }
    }

    // Send CAN messages after all temperature readings are processed
    if (cmd_ID == 7) {
        // Writes voltage values into CanOutboxes
        for (int i = 0; i < 35; i++) {
            cellVoltages[i].dlc = 8; // Set DLC (Data Length Code) to 8 bytes
            for (int j = 0; j < 4; j++) {
                // Get voltage data
                float v = voltageData[i * 4 + j];
                if (v < 0.1f)
                    v = 0; // Set to 0 if too low

                // Write voltage data into CAN outbox
                can_writeFloat(&cellVoltages[i].data, &cellVoltages[i], j * 2, v, 0.0001f);

                // Check for voltage limits and update min/max values
                //checkMinMaxCells(i * 4 + j);
            }
        }

        // Writes temperature values into CanOutboxes
        for (int i = 0; i < 13; i++) {
            cellTemps[i].dlc = 8; // Set DLC (Data Length Code) to 8 bytes
            for (int j = 0; j < 4; j++) {
                // Get temperature data
                float t = tempData[i * 4 + j];
                if (t < 0.1f)
                    t = 0; // Set to 0 if too low

                // Break the loop if we are at the end of the array
                if (i == 12 && j > 1)
                    break;

                // Write temperature data into CAN outbox
                can_writeFloat(&cellTemps[i].data, &cellTemps[i], j * 2, t, 0.1f);

                // Check for temperature limits and update min/max values
                //checkMinMaxTemps(i * 4 + j);
            }
        }
    }

    // Cycle through commands for each iteration
    cmd_ID++;

    // Reset command ID when all commands are processed
    if (cmd_ID == 8)
        cmd_ID = -1;
}

/**
 * @brief Initializes CAN outboxes to send voltage and temperature data.
 */
void cellsInit() {
    //setDeadCells();      // Identify and set dead cells
}

float convertVoltage(uint16_t value) {
    return value * 0.0001f; // Example conversion factor, adjust as needed
}

// Convert voltage to temperature using thermistor equation
float convertTemp(float voltage, float referenceVoltage) {
    return (voltage / referenceVoltage) * OVER_TEMP; // Example calculation, adjust as needed
}
