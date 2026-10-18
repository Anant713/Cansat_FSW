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

/* Once after CubeMX UART4/DMA initialization, in main-loop context. */
bool Communication_Init(void);


/* Services XBee and a bounded RX chunk each pass. Transmission scheduling and
 * telemetry.enabled gating belong to FlightApp. No automatic radio recovery. */

void Communication_Update(Cansat *cansat);


/* Binary serialization: 108 data bytes, 44 comma bytes, final CR = 153 bytes.
 * Multi-byte integers are little-endian; signed fields use two's complement.
 * Team ID occupies 27 bytes, padded with zero bytes. No extra string terminator.
 * Measurement sentinels: unsigned maximum / signed minimum; invalid hemisphere
 * is 0xff. Sentinels are reserved, so those endpoints cannot represent readings.
 * Counters/command ID/booleans retain their full ranges (no sentinel).
 * Rounds scaled native units to nearest integer, halfway away from zero.
 * Returns 153, or zero for invalid arguments/team ID/insufficient capacity.
 * Parse fixed widths: binary payload can contain comma, CR, and NUL bytes.
 * Never use strlen, CSV splitting, or XBee_SendString on this buffer. */
size_t Communication_FormatTelemetry(const Cansat *cansat, uint8_t *buffer, size_t capacity);

/* Sends a snapshot with packetCount+1 (first packet is 1, modulo 65536).
 * Count advances only if XBee accepts the complete row, not on busy/error.
 * True means UART acceptance, not wire completion or confirmed RF delivery.
 * Explicit sends bypass the scheduled telemetry.enabled gate. */
bool Communication_SendTelemetry(Cansat *cansat);

/* Ground-command placeholder. Input is a byte chunk, NOT a NUL-terminated
 * string or necessarily a complete command. Currently changes no state.
 * Define framing/reassembly and exact matching here later. */
void Communication_ProcessCommand(Cansat *cansat, const uint8_t *bytes, size_t length);

#endif
