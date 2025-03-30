//
// Created by alice on 3/30/2025.
//

#include "night_can.h"
#include "faults.h"
#include <stdlib.h> // Required for malloc and free
#include <math.h>   // Required for fmodf?

// Define hash table structures
typedef struct {
    uint32_t key;      // CAN ID
    CanInbox *inbox;    // Pointer to the inbox
    CanOutbox *outbox;  // Pointer to the outbox
    struct HashEntry *next; // Pointer to the next entry in case of collision
} HashEntry;

#define HASH_TABLE_SIZE 128  // Size of the hash table, can be adjusted based on the number of CAN IDs

// Hash table for inboxes (receives CAN messages)
static HashEntry *inboxTable[HASH_TABLE_SIZE] = {NULL};

// Hash table for outboxes (sends CAN messages)
static HashEntry *outboxTable[HASH_TABLE_SIZE] = {NULL};

// CAN handle (a pointer to the CAN peripheral's data structure)
static CAN_HANDLE *canHandleTypeDef;

// Simple hash function to compute the index in the hash table
static uint32_t hash(uint32_t key) {
    return key % HASH_TABLE_SIZE;  // Modulo operation to fit the key within the table size
}

// Function to get an Inbox from the hash table
static CanInbox *can_getInbox(uint32_t id) {
    uint32_t index = hash(id);    // Compute the index using the hash function
    HashEntry *entry = inboxTable[index]; // Get the first entry at this index

    // Traverse the linked list to find the entry with the matching ID
    while (entry != NULL) {
        if (entry->key == id) {   // If the ID matches, return the inbox
            return entry->inbox;
        }
        entry = entry->next;      // Move to the next entry in the linked list
    }

    return NULL;  // If no matching entry is found, return NULL
}

// Function to add an Inbox to the hash table
static void can_addInboxToTable(uint32_t id, CanInbox *inbox) {
    uint32_t index = hash(id);  // Compute the index using the hash function
    HashEntry *newEntry = (HashEntry *)malloc(sizeof(HashEntry)); // Allocate memory for the new entry

    if (newEntry == NULL) {
        // Handle memory allocation failure, potentially by returning an error code
        return;
    }
    newEntry->key = id;          // Set the CAN ID
    newEntry->inbox = inbox;      // Set the inbox pointer
    newEntry->outbox = NULL;     // Not an outbox entry
    newEntry->next = inboxTable[index]; // Add the new entry to the beginning of the linked list
    inboxTable[index] = newEntry;    // Update the table to point to the new entry
}

// Function to add an Outbox to the hash table
static void can_addOutboxToTable(uint32_t id, CanOutbox *outbox) {
    uint32_t index = hash(id);  // Compute the index using the hash function
    HashEntry *newEntry = (HashEntry *)malloc(sizeof(HashEntry)); // Allocate memory for the new entry

    if (newEntry == NULL) {
        // Handle memory allocation failure, potentially by returning an error code
        return;
    }
    newEntry->key = id;          // Set the CAN ID
    newEntry->inbox = NULL;      // Not an inbox entry
    newEntry->outbox = outbox;    // Set the outbox pointer
    newEntry->next = outboxTable[index]; // Add the new entry to the beginning of the linked list
    outboxTable[index] = newEntry;    // Update the table to point to the new entry
}

//H7 Specific code
#ifdef H7_SERIES

// Function to convert DLC (Data Length Code) to number of bytes
static uint8_t dlc_to_num(uint32_t dlc) {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    default:                   return 0;
  }
}

// Function to convert number of bytes to DLC (Data Length Code)
static uint32_t num_to_dlc(uint8_t num) {
  switch (num) {
    case 0:  return FDCAN_DLC_BYTES_0;
    case 1:  return FDCAN_DLC_BYTES_1;
    case 2:  return FDCAN_DLC_BYTES_2;
    case 3:  return FDCAN_DLC_BYTES_3;
    case 4:  return FDCAN_DLC_BYTES_4;
    case 5:  return FDCAN_DLC_BYTES_5;
    case 6:  return FDCAN_DLC_BYTES_6;
    case 7:  return FDCAN_DLC_BYTES_7;
    case 8:  return FDCAN_DLC_BYTES_8;
    default:   return FDCAN_DLC_BYTES_0;
  }
}

#endif

/**
 * Put a CAN packet in the Tx FIFO, which is guaranteed to be pushed onto the CAN BUS
 * @param id    ID of the CAN packet
 * @param dlc   Length of the CAN packet (number of bytes)
 * @param data  Data of the CAN packet
 * @return 0 if successful, 1 if unsuccessful
 */
uint32_t can_send(uint32_t id, uint8_t dlc, uint8_t *data) {
#ifdef H7_SERIES
  static FDCAN_TxHeaderTypeDef TxHeader; // Static transmit header
  TxHeader.Identifier = id;                // Set the CAN ID
  TxHeader.IdType = (id > 0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID; // Determine if it's standard or extended ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // Set the frame type to data frame
  TxHeader.DataLength = num_to_dlc(dlc);     // Set the data length code
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // Set error state indicator
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // Disable bit rate switching
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // Set format to classic CAN
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Disable transmit event FIFO
  TxHeader.MessageMarker = 0;

  // Add message to the FDCAN transmit FIFO
  uint32_t error = HAL_FDCAN_AddMessageToTxFifoQ(canHandleTypeDef, &TxHeader, data);
  if (error != HAL_OK) {
    if(error & HAL_FDCAN_ERROR_FIFO_FULL) {   // If the FIFO is full
      FAULT_SET(&faultVector, FAULT_VCU_CAN_BAD_TX); // Set a fault
    } else if(error & 0xFF) {
      return canHandleTypeDef->ErrorCode;       // Return the error code
    }
  }
#endif
#ifdef STM32L431xx
  static CAN_TxHeaderTypeDef TxHeader;   // Static transmit header
  if(id > 0x7FF) {
      TxHeader.IDE = CAN_ID_EXT;         // Set to extended ID if ID is greater than 0x7FF
      TxHeader.ExtId = id;                // Set the extended ID
  } else {
      TxHeader.IDE = CAN_ID_STD;         // Set to standard ID
      TxHeader.StdId = id;                // Set the standard ID
  }
  TxHeader.DLC = dlc;                    // Set the data length code
  TxHeader.RTR = CAN_RTR_DATA;            // Set to data frame

  uint32_t TxMailbox; // Variable to store the mailbox number
  // Add message to the CAN transmit mailbox
  volatile uint32_t error = HAL_CAN_AddTxMessage(canHandleTypeDef, &TxHeader, data, &TxMailbox);
  if (error != HAL_OK) {
    if(canHandleTypeDef->ErrorCode & 0x200000) { // If the FIFO is full
      HAL_CAN_AbortTxRequest(canHandleTypeDef, 0x7); // Abort the transmit request
      FAULT_SET(&faultVector, FAULT_VCU_CAN_BAD_TX); // Set a fault
    } else {
      return canHandleTypeDef->ErrorCode;       // Return the error code
    }
  }
#endif

  return 0; // Return 0 if successful
}

/**
 * Initialize the CAN interface
 * @param handle A pointer to the CAN peripheral's data structure
 */
void can_init(CAN_HANDLE *handle) {
  canHandleTypeDef = handle; // Set the CAN handle

#ifdef H7_SERIES
  HAL_FDCAN_Start(canHandleTypeDef); // Start the FDCAN peripheral
#endif
#ifdef STM32L431xx
  HAL_CAN_Start(canHandleTypeDef);   // Start the CAN peripheral
#endif
}

/**
 * Add a CAN outbox to the system
 * @param id     ID of the CAN message
 * @param period Transmission period in seconds
 * @param outbox Pointer to the CanOutbox structure
 */
void can_addOutbox(uint32_t id, float period, CanOutbox *outbox) {
  outbox->period = period;            // Set the transmission period
  outbox->_timer = 0.0f;
  can_addOutboxToTable(id, outbox); // Add the outbox to the hash table
}

/**
 * Add multiple CAN outboxes to the system with a staggered transmission
 * @param idLow     Starting ID of the CAN messages
 * @param idHigh    Ending ID of the CAN messages
 * @param period    Total transmission period in seconds
 * @param outboxes  Pointer to the array of CanOutbox structures
 */
void can_addOutboxes(uint32_t idLow, uint32_t idHigh, float period, CanOutbox *outboxes) {
  float staggerInterval = period / ((float)(idHigh - idLow + 1)); // Calculate the stagger interval
  float stagger = 0; // Initialize the stagger time
  // Loop through the IDs and add the outboxes
  for (uint32_t i = idLow; i <= idHigh; i++, outboxes++) {
    outboxes->_timer = stagger;         // Set the initial timer value
    can_addOutbox(i, period, outboxes);  // Add the outbox to the system
    stagger += staggerInterval;          // Increment the stagger time
  }
}

/**
 * Add a CAN inbox to the system
 * @param id          ID of the CAN message
 * @param mailbox     Pointer to the CanInbox structure
 * @param timeoutLimit Timeout limit in seconds
 */
void can_addInbox(uint32_t id, CanInbox *mailbox, float timeoutLimit) {
  mailbox->timeLimit = timeoutLimit;       // Set the timeout limit
  mailbox->ageSinceRx = 0.0f;
  mailbox->isRecent = false;
  mailbox->dlc = 0;
  mailbox->isTimeout = false;
  can_addInboxToTable(id, mailbox);      // Add the inbox to the hash table
}

/**
 * Add multiple CAN inboxes to the system
 * @param idLow       Starting ID of the CAN messages
 * @param idHigh      Ending ID of the CAN messages
 * @param mailboxes   Pointer to the array of CanInbox structures
 * @param timeoutLimit Timeout limit in seconds
 */
void can_addInboxes(uint32_t idLow, uint32_t idHigh, CanInbox *mailboxes, float timeoutLimit) {
  // Loop through the IDs and add the inboxes
  for (uint32_t i = idLow; i <= idHigh; i++) {
    can_addInbox(i, &mailboxes[i - idLow], timeoutLimit); // Add the inbox to the system
  }
}

/**
 * Process the CAN Rx FIFO to receive incoming messages
 * @return HAL_OK if successful, error code otherwise
 */
static uint32_t can_processRxFifo() {
#ifdef H7_SERIES
  static FDCAN_RxHeaderTypeDef RxHeader; // Static receive header
  static uint8_t RxData[8];             // Static receive data buffer

  // Loop to process all messages in the FIFO
  while (HAL_FDCAN_GetRxMessage(canHandleTypeDef, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    uint32_t id = RxHeader.Identifier;      // Get the CAN ID
    uint8_t dlc = dlc_to_num(RxHeader.DataLength); // Get the data length code
    CanInbox *this_mailbox = can_getInbox(id);  // Get the inbox associated with this ID

    // If an inbox is found for this ID
    if (this_mailbox != NULL) {
        for(int i = 0; i < dlc; i++)
        {
          this_mailbox->data[i] = RxData[i];
        }

        this_mailbox->isRecent = true;       // Set the message as recent
        this_mailbox->dlc = dlc;              // Set the data length code
        this_mailbox->ageSinceRx = 0;         // Reset the age since last receive
        this_mailbox->isTimeout = false;       // Clear the timeout flag
    }
  }
  // If an error occurred
  if ((canHandleTypeDef->ErrorCode & 0xFF) != HAL_FDCAN_ERROR_NONE) {
    FAULT_SET(&faultVector, FAULT_VCU_CAN_BAD_RX); // Set a fault
    return canHandleTypeDef->ErrorCode;             // Return the error code
  }
#endif
#ifdef STM32L431xx
  static CAN_RxHeaderTypeDef RxHeader;   // Static receive header
    static uint8_t RxData[8];               // Static receive data buffer
    // Loop to process all messages in the FIFO
    while(HAL_CAN_GetRxFifoFillLevel(canHandleTypeDef, CAN_RX_FIFO0)) {
        // If a message is received
        if(HAL_CAN_GetRxMessage(canHandleTypeDef, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            uint32_t id = (RxHeader.IDE == CAN_ID_EXT) ? RxHeader.ExtId : RxHeader.StdId; // Get the CAN ID
            uint32_t dlc = RxHeader.DLC;              // Get the data length code
            CanInbox *this_mailbox = can_getInbox(id);  // Get the inbox associated with this ID

            // If an inbox is found for this ID
            if (this_mailbox != NULL) {
                for(int i = 0; i < dlc; i++)
                {
                  this_mailbox->data[i] = RxData[i];
                }
                this_mailbox->isRecent = true;       // Set the message as recent
                this_mailbox->dlc = dlc;              // Set the data length code
                this_mailbox->ageSinceRx = 0;         // Reset the age since last receive
                this_mailbox->isTimeout = false;       // Clear the timeout flag
            }
        } else {
            return canHandleTypeDef->ErrorCode;         // Return the error code
        }
    }
    // If an error occurred
    if ((canHandleTypeDef->ErrorCode & 0xFF) != HAL_CAN_ERROR_NONE) {
      FAULT_SET(&faultVector, FAULT_VCU_CAN_BAD_RX); // Set a fault
      return canHandleTypeDef->ErrorCode;             // Return the error code
  }
#endif
  return HAL_OK; // Return HAL_OK if successful
}

/**
 * Send all CAN outboxes and update the status of CAN inboxes
 * @param deltaTime Time elapsed since the last call in seconds
 * @return HAL_OK if successful, error code otherwise
 */
static uint32_t can_sendAll(float deltaTime) {
  uint32_t error = HAL_OK; // Initialize error to HAL_OK

  // Iterate over outboxes in the hash table
  for (int i = 0; i < HASH_TABLE_SIZE; i++) {
      HashEntry *entry = outboxTable[i]; // Get the entry at this index
      // Loop through the linked list
      while (entry != NULL) {
          CanOutbox *outbox = entry->outbox; // Get the outbox
          uint32_t id = entry->key;            // Get the CAN ID

          outbox->_timer += deltaTime;          // Increment the timer
          // If the timer is greater than the period
          if (outbox->_timer >= outbox->period) {
              outbox->_timer = fmodf(outbox->_timer, outbox->period); // Reset the timer
              error = can_send(id, outbox->dlc, outbox->data);        // Send the CAN message
              // If an error occurred
              if (error != HAL_OK) {
                  return error;                      // Return the error code
              }
          }
          entry = entry->next; // Move to the next entry
      }
  }

  // Iterate over inboxes in the hash table
  for (int i = 0; i < HASH_TABLE_SIZE; i++) {
      HashEntry *entry = inboxTable[i]; // Get the entry at this index
      // Loop through the linked list
      while (entry != NULL) {
          CanInbox *inbox = entry->inbox; // Get the inbox

          inbox->ageSinceRx += deltaTime;    // Increment the age since last receive
          // If the timeout limit is set and the age since last receive is greater than the timeout limit
          if (inbox->timeLimit != 0 && inbox->timeLimit < inbox->ageSinceRx) {
              inbox->isTimeout = true;          // Set the timeout flag
              inbox->isRecent = false;         // Clear the recent flag
          }
          entry = entry->next; // Move to the next entry
      }
  }

  return HAL_OK; // Return HAL_OK if successful
}

/**
 * Periodic function to process CAN messages (both Rx and Tx)
 * @param deltaTime Time elapsed since the last call in seconds
 * @return HAL_OK if successful, error code otherwise
 */
uint32_t can_periodic(float deltaTime) {
  uint32_t error = can_processRxFifo(); // Process the receive FIFO
  if (error != HAL_OK) {
    return error; // 0x300              // Return the error code
  }

  error = can_sendAll(deltaTime);        // Send all CAN outboxes
  if (error != HAL_OK) {
    return error;                      // Return the error code
  }

  return HAL_OK; // Return HAL_OK if successful
}

/**
 * Writes a float value into the CAN outbox data buffer, scaling if needed
 * @param dest     pointer to the data buffer (uint16_t*)
 * @param outbox   Pointer to the CanOutbox structure
 * @param offset   Offset in the data buffer (in bytes)
 * @param value    Float value to be written
 * @param scale    Scaling factor (value is divided by scale before writing)
 * @return 0 if successful
 */
uint32_t can_writeFloat(uint16_t* dest, CanOutbox *outbox, uint8_t offset, float value, float scale)
{
    int16_t scaled_value = (int16_t)(value / scale);  // Scale the float value to an int16_t

    // Write the scaled value into the data buffer
    outbox->data[offset] = (uint8_t)(scaled_value & 0xFF);         // Lower byte
    outbox->data[offset + 1] = (uint8_t)((scaled_value >> 8) & 0xFF);  // Upper byte
    return 0; // Return 0 if successful
}
