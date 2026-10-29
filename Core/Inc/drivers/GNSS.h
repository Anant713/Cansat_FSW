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
#define GNSS_FRESH_MS 500U
#define GNSS_EXPECTED_BAUD 115200U
#define GNSS_EXPECTED_PERIOD_MS 100U

/* Quectel L89HA-S90; archived u-blox profiles are not supported here. */
#if defined(GNSS_ENABLE_NAV) || defined(GNSS_ENABLE_GSV)
#error "NEO-6M profiles are archived in Legacy/NEO6M; use TestGNSS for L89HA"
#endif
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
    bool hdopValid;
} GNSSData;

typedef enum {
    GNSS_CONFIG_IDLE = 0, GNSS_CONFIG_BUSY, GNSS_CONFIG_DONE,
    GNSS_CONFIG_NAK, GNSS_CONFIG_TIMEOUT, GNSS_CONFIG_IO_ERROR,
    GNSS_CONFIG_BAUD_UNCONFIRMED, GNSS_CONFIG_MISMATCH
} GNSSConfigState;
typedef struct {
    uint32_t receivedBytes, queueDrops, streamResets, uartErrors;
    uint32_t goodGga, goodVtg, badChecksum, malformed, ignored;
    uint32_t lastByteMs, lastUartError, queuedBytes;
    uint32_t baudRate;
    bool receiving;
    GNSSConfigState configState;
    uint8_t configStep;
    uint32_t configuredBaud, configuredPeriodMs, observedPeriodMs, rateMatches;
    uint16_t messageReplyMask;
    uint8_t messageRates[10], constellationMask;
    bool baudVerified, rateVerified, sentencesVerified, constellationsVerified;
    bool seenGnGga, seenGnVtg;
} GNSSDiagnostics;

/* After CubeMX UART/DMA init. currentBaud must match the receiver NOW.
 * Only one GNSS instance; dedicated UART. Init requires 115200 baud, then
 * asynchronously sets GGA/VTG-only output and verifies the module settings. */
bool GNSS_Init(UART_HandleTypeDef *uart, uint32_t currentBaud);
bool GNSS_StartReception(void);   /* explicit recovery after UART/DMA error */
bool GNSS_StopReception(void);
void GNSS_Process(void);          /* finite work, no waiting for bytes/ACKs */
bool GNSS_GetData(GNSSData *data); /* copies snapshot, false if GGA is stale */
bool GNSS_IsDataReady(void);      /* fresh GGA exists; not a consumable flag */
bool GNSS_IsFixValid(void);
void GNSS_GetDiagnostics(GNSSDiagnostics *diagnostics);

/* L89HA-S90: read-only recheck (PAIR865/051/063/067).
 * Init first sets sentence rates with PAIR062, then starts these checks.
 * Process advances them; DONE means queries
 * matched AND GNGGA/GNVTG + five 100 ms GGA UTC intervals were observed.
 * This recheck never changes settings. Neither Init nor recheck reboots or
 * writes flash; Init changes ONLY the standard NMEA output rates. */
bool GNSS_VerifyConfiguration(void);
/* Compatibility entry point: only accepts 100 ms / 115200 and rechecks.
 * It does NOT configure the receiver. Prefer VerifyConfiguration(). */
bool GNSS_Configure(uint16_t periodMs, uint32_t targetBaud);
/* Forward the matching HAL callbacks here; implementations supplied in
 * GNSS_HAL.c. RX callback copies bytes only. Keep DMA and UART IRQs at
 * the same preemption priority. Do not disable DMA half-transfer IRQ. */
void GNSS_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void GNSS_ErrorCallback(UART_HandleTypeDef *uart);
void GNSS_TxCompleteCallback(UART_HandleTypeDef *uart);
#endif
