#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "apps/Telemetry.h"

typedef struct Cansat Cansat;


#define COMMUNICATION_TELEMETRY_CAPACITY TELEMETRY_BINARY_SIZE
#define COMMUNICATION_TELEMETRY_FIELDS TELEMETRY_FIELD_COUNT
#define COMMUNICATION_TELEMETRY_PERIOD_MS 500U
#define COMMUNICATION_RX_BUDGET 256U

typedef enum {
    COMMAND_NONE = 0,
    COMMAND_TELEMETRY_ON = 1,
    COMMAND_TELEMETRY_OFF = 2,
    COMMAND_CALIBRATE_SENSORS = 3
} GroundCommand;

/* Optional main-loop observer for console diagnostics; never called in an ISR.
 * RECEIVED precedes execution; PENDING means asynchronous calibration started;
 * FINISHED reports success/failure, including later calibration completion. */
typedef enum { COMMAND_RECEIVED, COMMAND_PENDING, COMMAND_FINISHED } CommandEvent;
typedef void (*CommandObserver)(GroundCommand id, CommandEvent event, bool success);
void Communication_SetCommandObserver(CommandObserver observer);
/* Optional raw RX tap, called in main-loop context before parsing. The bytes
 * are borrowed for this call only; the observer must not consume XBee RX. */
typedef void (*CommunicationRxObserver)(const uint8_t *bytes, size_t length);
void Communication_SetRxObserver(CommunicationRxObserver observer);

/* Once after CubeMX UART4/DMA initialization, in main-loop context. */
bool Communication_Init(void);


/* Services XBee and a bounded RX chunk each pass. Transmission scheduling and
 * telemetry.enabled gating belong to FlightApp. No automatic radio recovery. */

void Communication_Update(Cansat *cansat);


/* Binary serialization: 107 data bytes, 43 comma bytes, final CR = 151 bytes.
 * Multi-byte integers are little-endian; signed fields use two's complement.
 * Team ID occupies 27 bytes, padded with zero bytes. No extra string terminator.
 * Measurement sentinels: unsigned maximum / signed minimum; invalid hemisphere
 * is 0xff. Sentinels are reserved, so those endpoints cannot represent readings.
 * Counters/booleans retain their full ranges (no sentinel).
 * Rounds scaled native units to nearest integer, halfway away from zero.
 * Returns 151, or zero for invalid arguments/team ID/insufficient capacity.
 * Parse fixed widths: binary payload can contain comma, CR, and NUL bytes.
 * Never use strlen, CSV splitting, or XBee_SendString on this buffer. */
size_t Communication_FormatTelemetry(const Cansat *cansat, uint8_t *buffer, size_t capacity);

/* Sends a snapshot with packetCount+1 (first packet is 1, modulo 65536).
 * Count advances only if XBee accepts the complete row, not on busy/error.
 * True means UART acceptance, not wire completion or confirmed RF delivery.
 * Explicit sends bypass the scheduled telemetry.enabled gate. */
bool Communication_SendTelemetry(Cansat *cansat);

/* Assemble binary A5 <ID 1..3> 5A packets across arbitrary UART chunks.
 * Main-loop only. Unknown IDs/malformed frames do not update bookkeeping.
 * Each recognized packet counts, including repeated commands. */
void Communication_ProcessCommand(Cansat *cansat, const uint8_t *bytes, size_t length);

#endif
