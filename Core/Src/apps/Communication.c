#include "apps/Communication.h"
#include "apps/Cansat.h"
#include "drivers/XBee.h"

extern UART_HandleTypeDef huart4;
static uint8_t telemetryBuffer[COMMUNICATION_TELEMETRY_CAPACITY];
_Static_assert(COMMUNICATION_TELEMETRY_CAPACITY <= XBEE_TX_CAPACITY,
               "Complete telemetry packet must fit XBee TX");

size_t Communication_FormatTelemetry(const Cansat *cansat, uint8_t *buffer, size_t capacity)
{
    return Telemetry_Encode(cansat, buffer, capacity);
}

bool Communication_Init(void)
{
    return XBee_Init(&huart4) == XBEE_OK;
}

bool Communication_SendTelemetry(Cansat *cansat)
{
    if (cansat == NULL || XBee_IsTxBusy()) return false;
    Cansat snapshot = *cansat;
    snapshot.telemetry.packetCount = (uint16_t)(snapshot.telemetry.packetCount + 1U);
    size_t length = Communication_FormatTelemetry(&snapshot, telemetryBuffer, sizeof(telemetryBuffer));
    if (length == 0U || XBee_Send(telemetryBuffer, (uint16_t)length) != XBEE_OK)
        return false;
    cansat->telemetry.packetCount = snapshot.telemetry.packetCount;
    return true;
}

void Communication_ProcessCommand(Cansat *cansat, const uint8_t *bytes, size_t length)
{
    /* TODO: Define ground-command framing and exact strings, assemble complete
     * commands across chunks, then compare exact lengths/bytes and execute.
     * Only complete commands may update command IDs/execution/RX packet count.
     * Input is not necessarily terminated and can contain embedded zero bytes. */
    (void)cansat;
    (void)bytes;
    (void)length;
}

void Communication_Update(Cansat *cansat)
{
    if (cansat == NULL) return;
    XBee_Process();
    uint8_t bytes[COMMUNICATION_RX_BUDGET];
    uint16_t length;
    XBeeResult result = XBee_Read(bytes, sizeof(bytes), &length);
    if (result == XBEE_OK && length != 0U)
        Communication_ProcessCommand(cansat, bytes, length);
    /* TODO: When command reassembly is added, discard partial commands and
     * resynchronize on XBEE_RX_LOST/IO_ERROR. No parser state exists yet. */
}
