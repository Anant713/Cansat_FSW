#ifndef GNSS_H
#define GNSS_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Main-loop parser, interrupt-driven circular RX DMA. Buffers must be in SRAM,
 * never CCM. Queue holds bytes plus their arrival ticks (about 20 KiB total). */
#define GNSS_DMA_SIZE 512U
#define GNSS_QUEUE_SIZE 4096U
#define GNSS_PROCESS_BUDGET 1024U
#define GNSS_FRESH_MS 3000U

typedef struct {
    uint32_t utcTime;              /* HHMMSS, UTC time-of-day; GGA has no date */
    uint16_t utcMilliseconds;
    double latitude, longitude;  /* signed decimal degrees */
    char latitudeHemisphere, longitudeHemisphere;
    float altitude;               /* metres above mean sea level, from GGA */
    float speed;                  /* metres/second over ground, from VTG */
    float course;                 /* degrees true over ground, NOT attitude */
    float hdop;
    uint8_t satellites;           /* used in fix, NOT satellites in view */
    uint8_t fixQuality;
    uint32_t ggaReceivedMs, vtgReceivedMs;
    bool utcValid, altitudeValid, speedValid, courseValid, vtgValid;
    bool fixValid;                /* fresh satellite fix: GGA quality 1 or 2 */
    bool valid;                   /* fresh, well-formed GGA, including NO FIX */
} GNSSData;

typedef enum {
    GNSS_CONFIG_IDLE = 0, GNSS_CONFIG_BUSY, GNSS_CONFIG_DONE,
    GNSS_CONFIG_NAK, GNSS_CONFIG_TIMEOUT, GNSS_CONFIG_IO_ERROR,
    GNSS_CONFIG_BAUD_UNCONFIRMED
} GNSSConfigState;
typedef struct {
    uint32_t receivedBytes, queueDrops, streamResets, uartErrors;
    uint32_t goodGga, goodVtg, badChecksum, malformed, ignored;
    uint32_t lastByteMs, lastUartError, queuedBytes;
    uint32_t baudRate;
    bool receiving;
    GNSSConfigState configState;
    uint8_t configStep;
} GNSSDiagnostics;

/* After CubeMX UART/DMA init. currentBaud must match the receiver NOW.
 * Only one GNSS instance; dedicated UART. Init configures MCU baud only. */
bool GNSS_Init(UART_HandleTypeDef *uart, uint32_t currentBaud);
bool GNSS_StartReception(void);   /* explicit recovery after UART/DMA error */
bool GNSS_StopReception(void);
void GNSS_Process(void);          /* finite work, no waiting for bytes/ACKs */
bool GNSS_GetData(GNSSData *data); /* copies snapshot, false if GGA is stale */
bool GNSS_IsDataReady(void);      /* fresh GGA exists; not a consumable flag */
bool GNSS_IsFixValid(void);
void GNSS_GetDiagnostics(GNSSDiagnostics *diagnostics);

/* Asynchronous receiver configuration, advanced by Process():
 * GGA+VTG each solution, disable other standard NMEA/TXT, set 1/2/5 Hz,
 * optionally change both module and MCU baud, then verify port readback.
 * periodMs: 1000, 500, 200. targetBaud: 9600..115200 standard rates.
 * Poll diagnostics.configState; true means STARTED, not acknowledged.
 * Volatile receiver settings only; no flash/BBR save command is sent.
 * Call Process regularly during configuration (preferably <=100 ms). */
bool GNSS_Configure(uint16_t periodMs, uint32_t targetBaud);

/* Forward the matching HAL callbacks here; implementations supplied in
 * GNSS_HAL.c. RX callback copies bytes only. Keep DMA and UART IRQs at
 * the same preemption priority. Do not disable DMA half-transfer IRQ. */
void GNSS_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void GNSS_ErrorCallback(UART_HandleTypeDef *uart);
void GNSS_TxCompleteCallback(UART_HandleTypeDef *uart);
#endif
