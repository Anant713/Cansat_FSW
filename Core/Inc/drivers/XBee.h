#ifndef XBEE_H
#define XBEE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    XBEE_IDLE = 0,
    XBEE_TX_BUSY,
    XBEE_RX_READY,
    XBEE_ERROR
} XBeeStatus;


/* Ground commands */

typedef enum
{
    CMD_NONE = 0,
    CMD_SEND_TELEMETRY,
    CMD_CALIBRATE,
    CMD_TEST,
    CMD_RESET
} XBeeCommand;


typedef struct
{
    XBeeCommand command;

    uint8_t payload[32];
    uint16_t payloadLength;

    bool valid;

} XBeeCommandData;


bool XBee_Init(void);


/* TX */

bool XBee_Send(const uint8_t *data,
               uint16_t length);

bool XBee_SendString(const char *data);


/* RX */

bool XBee_IsCommandAvailable(void);

bool XBee_GetCommand(XBeeCommandData *command);

void XBee_ProcessReceivedData(void);


/* UART/DMA event */

void XBee_RxEventCallback(uint16_t size);

void XBee_TxCompleteCallback(void);

void XBee_ErrorCallback(void);


/* Status */

bool XBee_IsTxBusy(void);

XBeeStatus XBee_GetStatus(void);

#endif