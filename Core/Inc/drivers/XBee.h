#ifndef XBEE_H
#define XBEE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Byte transport only. Call public functions from one main-loop context.
 * DMA memory must stay in normal SRAM (not STM32F405 CCM RAM). */
#define XBEE_TX_CAPACITY       256U
#define XBEE_RX_DMA_SIZE       256U
#define XBEE_RX_QUEUE_SIZE     2048U
#define XBEE_RESET_PULSE_MS    2U
#define XBEE_BOOT_GUARD_MS     500U /* Local guard, NOT proof of network joining. */

typedef enum {
    XBEE_OK = 0, XBEE_BUSY, XBEE_NOT_READY, XBEE_INVALID_ARGUMENT,
    XBEE_IO_ERROR, XBEE_RX_LOST
} XBeeResult;

typedef enum {
    XBEE_UNINITIALIZED = 0, XBEE_BOOTING, XBEE_ASLEEP,
    XBEE_READY, XBEE_TRANSMITTING, XBEE_RECOVERING, XBEE_FAULT
} XBeeStatus;

typedef struct {
    uint32_t receivedBytes, droppedBytes, rxLossEvents;
    uint32_t txStarted, txCompleted, txAborted;
    uint32_t uartErrors, lastUartError;
    uint16_t rxAvailable;
    bool receiving, txBusy;
} XBeeDiagnostics;

/* Uses the baud configured in CubeMX; does NOT reconfigure the radio.
 * UART must be 8N1, RX circular DMA, TX normal DMA, byte alignment.
 * PA2 must be open drain, released high. PA3 is ON/SLEEP status input.
 * UART/DMA IRQs must be enabled with equal preemption priority. */
XBeeResult XBee_Init(UART_HandleTypeDef *uart);
void XBee_Process(void); /* Advances reset/recovery; never waits for hardware. */

/* Copies 1..XBEE_TX_CAPACITY bytes before returning. Caller may immediately reuse data.
 * Busy means nothing was queued; retry later. Success is UART acceptance,
 * txCompleted means UART wire completion, neither confirms RF delivery. */
XBeeResult XBee_Send(const uint8_t *data, uint16_t length);
XBeeResult XBee_SendString(const char *text);
bool XBee_IsTxBusy(void);

/* Reads currently queued bytes without waiting; zero length means none.
 * Chunks are NOT messages. On XBEE_RX_LOST, queue is flushed and length=0:
 * application must discard its partial command and resynchronize framing. */
XBeeResult XBee_Read(uint8_t *data, uint16_t capacity, uint16_t *length);
bool XBee_IsAwake(void); /* Raw ON/SLEEP indication; not RF association/CTS. */
XBeeStatus XBee_GetStatus(void);
void XBee_GetDiagnostics(XBeeDiagnostics *out);

/* Reset refuses while TX busy. Recovery can abort a faulted transfer; it
 * never retransmits it. Both discard RX and report XBEE_RX_LOST on Read.
 * Call Process repeatedly until no longer BOOTING/RECOVERING. */
XBeeResult XBee_Reset(void);
XBeeResult XBee_Recover(void);

/* Called only by the shared HAL callback dispatcher. */
void XBee_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void XBee_TxCompleteCallback(UART_HandleTypeDef *uart);
void XBee_ErrorCallback(UART_HandleTypeDef *uart);
void XBee_AbortCompleteCallback(UART_HandleTypeDef *uart);

#endif
