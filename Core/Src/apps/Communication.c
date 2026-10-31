#include "apps/Communication.h"
#include "apps/Cansat.h"
#include "apps/SensorAcquisition.h"
#include "apps/StateMachine.h"
#include "drivers/XBee.h"

extern UART_HandleTypeDef huart4;
static uint8_t telemetryBuffer[COMMUNICATION_TELEMETRY_CAPACITY];
static uint8_t commandPosition, commandId;
static bool calibrationPending, calibrationIsLatest;
static CommandObserver commandObserver;
static CommunicationRxObserver rxObserver;
void Communication_SetRxObserver(CommunicationRxObserver observer) { rxObserver = observer; }
void Communication_SetCommandObserver(CommandObserver observer) { commandObserver = observer; }
static void notifyCommand(uint8_t id, CommandEvent event, bool success)
{
    if (commandObserver) commandObserver((GroundCommand)id, event, success);
}
_Static_assert(COMMUNICATION_TELEMETRY_CAPACITY <= XBEE_TX_CAPACITY,
               "Complete telemetry packet must fit XBee TX");

size_t Communication_FormatTelemetry(const Cansat *cansat, uint8_t *buffer, size_t capacity)
{
    return Telemetry_Encode(cansat, buffer, capacity);
}

bool Communication_Init(void)
{
    commandPosition = commandId = 0;
    calibrationPending = calibrationIsLatest = false;
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

static void executeCommand(Cansat *cansat, uint8_t id)
{
    cansat->telemetry.lastReceivedCommand = id;
    ++cansat->telemetry.receivedPacketCount;
    cansat->telemetry.commandExecuted = false;
    calibrationIsLatest = false;
    notifyCommand(id, COMMAND_RECEIVED, false);
    switch (id) {
    case COMMAND_TELEMETRY_ON:
    case COMMAND_TELEMETRY_OFF:
        cansat->telemetry.enabled = id == COMMAND_TELEMETRY_ON;
        cansat->telemetry.commandExecuted = true;
        break;
    case COMMAND_CALIBRATE_SENSORS:
        if (cansat->missionState == CANSAT_STATE_LAUNCH_PAD &&
            StateMachine_GetStatus()->persistenceOK) {
            if (calibrationPending || SensorAcquisition_StartCalibration(cansat)) {
                calibrationPending = calibrationIsLatest = true;
            }
        }
        break;
    default: break;
    }
    if (cansat->telemetry.commandExecuted)
        cansat->telemetry.lastExecutedCommand = id;
    notifyCommand(id, calibrationIsLatest ? COMMAND_PENDING : COMMAND_FINISHED,
                  cansat->telemetry.commandExecuted);
}

void Communication_ProcessCommand(Cansat *cansat, const uint8_t *bytes, size_t length)
{
    if (cansat == NULL || bytes == NULL) return;
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = bytes[i];
        /* A new start byte always resynchronizes, including A5 A5 01 5A. */
        if (byte == 0xa5U) {
            commandPosition = 1;
        } else if (commandPosition == 1 && byte >= COMMAND_TELEMETRY_ON &&
                   byte <= COMMAND_CALIBRATE_SENSORS) {
            commandId = byte;
            commandPosition = 2;
        } else {
            bool complete = commandPosition == 2 && byte == 0x5aU;
            commandPosition = 0;
            if (complete) executeCommand(cansat, commandId);
        }
    }
}

void Communication_Update(Cansat *cansat)
{
    if (cansat == NULL) return;
    if (calibrationPending) {
        SensorCalibrationStatus result = SensorAcquisition_GetCalibrationStatus();
        if (result != SENSOR_CAL_RUNNING) {
            bool success = result == SENSOR_CAL_SUCCESS; /* Acquisition already committed both values. */
            if (success) cansat->telemetry.lastExecutedCommand = COMMAND_CALIBRATE_SENSORS;
            if (calibrationIsLatest) cansat->telemetry.commandExecuted = success;
            calibrationPending = calibrationIsLatest = false;
            notifyCommand(COMMAND_CALIBRATE_SENSORS, COMMAND_FINISHED, success);
        }
    }
    XBee_Process();
    uint8_t bytes[COMMUNICATION_RX_BUDGET];
    uint16_t length;
    XBeeResult result = XBee_Read(bytes, sizeof(bytes), &length);
    if (result == XBEE_OK && length != 0U) {
        if (rxObserver) rxObserver(bytes, length);
        Communication_ProcessCommand(cansat, bytes, length);
    }
    else if (result != XBEE_OK)
        commandPosition = 0; /* Never join fragments across known RX loss/errors. */
}
