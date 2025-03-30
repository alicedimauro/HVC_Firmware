//
// Created by alice on 3/30/2025.
//


#ifndef NIGHT_CAN_H
#define NIGHT_CAN_H

#include <stdint.h>
#include <stdbool.h>

// Define a generic CAN handle type
typedef void* CAN_HANDLE;

// Structure for CAN inbox
typedef struct {
    uint8_t data[8];
    uint8_t dlc;
    float timeLimit;
    float ageSinceRx;
    bool isRecent;
    bool isTimeout;
} CanInbox;

// Structure for CAN outbox
typedef struct {
    uint8_t data[8];
    uint8_t dlc;
    float period;
    float _timer;
} CanOutbox;

// Initialization function
void can_init(CAN_HANDLE *handle);

// Adding outboxes
void can_addOutbox(uint32_t id, float period, CanOutbox *outbox);
void can_addOutboxes(uint32_t idLow, uint32_t idHigh, float period, CanOutbox *outboxes);

// Adding inboxes
void can_addInbox(uint32_t id, CanInbox *mailbox, float timeoutLimit);
void can_addInboxes(uint32_t idLow, uint32_t idHigh, CanInbox *mailboxes, float timeoutLimit);

// Periodic function to process CAN messages
uint32_t can_periodic(float deltaTime);

// Send CAN message function
uint32_t can_send(uint32_t id, uint8_t dlc, uint8_t *data);

// Helper function to write float data into CAN message
uint32_t can_writeFloat(uint16_t* dest, CanOutbox *outbox, uint8_t offset, float value, float scale);

#endif //NIGHT_CAN_H
