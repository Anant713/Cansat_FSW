#include "App.h"
#include "main.h"
#include "Console.h"
#include "apps/Cansat.h"
#include "apps/SensorAcquisition.h"
#include "apps/Communication.h"
#include "apps/Logging.h"
#include "apps/StateMachine.h"
#include "drivers/MissionStore.h"
#include "drivers/XBee.h"
#include <string.h>
#include <stdio.h>

/* Ground bench only: no mission transitions or actuator entry actions. The
 * production command handler queries this test context for calibration policy.
 * Do not link StateMachine.c into this target. Preserve existing mission records. */
static StateMachineStatus benchMission;
const StateMachineStatus *StateMachine_GetStatus(void) { return &benchMission; }

Cansat test_telecommands_data;
bool test_telecommands_sensors_ok, test_telecommands_xbee_ok;
uint32_t test_telecommands_logs, test_telecommands_log_errors;
uint32_t test_telecommands_tx_accepted, test_telecommands_tx_errors;
static uint32_t lastAcquisition, lastTelemetry, lastLog, lastReport;
static bool stream = true;
static bool rawRx = true;
static char input[CONSOLE_LINE_MAX + 1U];
#define TELECOMMAND_REPORT_MS 5000U

static void rawRxEvent(const uint8_t *bytes, size_t length)
{
    if (!rawRx) return;
    /* Bounded diagnostic output even if a radio delivers a continuous stream. */
    char text[160];
    size_t shown = length > 32U ? 32U : length;
    size_t used = (size_t)snprintf(text, sizeof(text), "XBee RAW RX (%u bytes):", (unsigned)length);
    for (size_t i = 0; i < shown; ++i)
        used += (size_t)snprintf(text + used, sizeof(text) - used, " %02X", (unsigned)bytes[i]);
    (void)snprintf(text + used, sizeof(text) - used, "%s\r\n", shown < length ? " ..." : "");
    Console_Write(text);
}

static const char *commandName(GroundCommand id)
{
    switch (id) {
    case COMMAND_TELEMETRY_ON: return "TELEMETRY_ON";
    case COMMAND_TELEMETRY_OFF: return "TELEMETRY_OFF";
    case COMMAND_CALIBRATE_SENSORS: return "CALIBRATE_SENSORS";
    default: return "NONE";
    }
}

static void commandEvent(GroundCommand id, CommandEvent event, bool success)
{
    if (event == COMMAND_RECEIVED) {
        Console_Printf("RX command %u: %s\r\n", (unsigned)id, commandName(id));
        if (id == COMMAND_CALIBRATE_SENSORS)
            Console_Write("Calibration requested: keep stationary; initialization/ground sampling may take several seconds.\r\n");
    } else if (event == COMMAND_PENDING) {
        Console_Write("Calibration IN PROGRESS: collecting gyro samples.\r\n");
    } else if (id == COMMAND_CALIBRATE_SENSORS) {
        if (success)
            Console_Printf("Calibration SUCCESS: gyro biases and ground pressure saved to flash; ground=%.2f Pa.\r\n",
                           (double)SensorAcquisition_GetGroundPressure());
        else
            Console_Write("Calibration FAILED/rejected: check sensor motion/I/O, timeout or flash status.\r\n");
    } else {
        Console_Printf("Command %u %s: telemetry %s. Logging continues.\r\n", (unsigned)id,
                       success ? "SUCCESS" : "FAILED",
                       test_telecommands_data.telemetry.enabled ? "ON" : "OFF");
    }
}

static void report(void)
{
    const TelemetryState *t = &test_telecommands_data.telemetry;
    XBeeDiagnostics d;
    XBee_GetDiagnostics(&d);
    Console_Printf("Telemetry=%s TX accepted=%lu errors=%lu | SD rows=%lu errors=%lu ready=%u\r\n",
        t->enabled ? "ON" : "OFF", (unsigned long)test_telecommands_tx_accepted,
        (unsigned long)test_telecommands_tx_errors, (unsigned long)test_telecommands_logs,
        (unsigned long)test_telecommands_log_errors, (unsigned)Logging_IsReady());
    Console_Printf("Commands=%u last RX=%u last successful=%u latest success=%u | calibration=%u saved=%u ground=%.2f Pa\r\n",
        (unsigned)t->receivedPacketCount, (unsigned)t->lastReceivedCommand,
        (unsigned)t->lastExecutedCommand, (unsigned)t->commandExecuted,
        (unsigned)SensorAcquisition_GetCalibrationStatus(), (unsigned)SensorAcquisition_HasCalibration(),
        (double)SensorAcquisition_GetGroundPressure());
    Console_Printf("XBee RX bytes=%lu loss events=%lu UART errors=%lu status=%u receiving=%u queued=%u lastError=0x%08lX\r\n",
        (unsigned long)d.receivedBytes, (unsigned long)d.rxLossEvents,
        (unsigned long)d.uartErrors, (unsigned)XBee_GetStatus(), (unsigned)d.receiving,
        (unsigned)d.rxAvailable, (unsigned long)d.lastUartError);
}

void App_Init(void)
{
    (void)Console_Init();
    Cansat_Init(&test_telecommands_data); /* Bench context stays LAUNCH_PAD. */
    uint32_t missionPayload;
    MissionStoreResult mission = MissionStore_Load(&missionPayload);
    benchMission.persistenceOK = mission == MISSION_STORE_VALID ||
        (mission == MISSION_STORE_EMPTY && MissionStore_Save(CANSAT_STATE_LAUNCH_PAD));
    if (!benchMission.persistenceOK)
        Console_Write("Mission journal invalid/full: calibration commands will be rejected.\r\n");
    if (!Logging_Init()) Console_Write("SD logging initialization FAILED.\r\n");
    test_telecommands_sensors_ok = SensorAcquisition_Init();
    Console_Printf("Sensors=%s calibration restored=%u ground=%.2f Pa\r\n",
        test_telecommands_sensors_ok ? "OK" : "ERROR", (unsigned)SensorAcquisition_HasCalibration(),
        (double)SensorAcquisition_GetGroundPressure());
    test_telecommands_xbee_ok = Communication_Init();
    Communication_SetCommandObserver(commandEvent);
    Communication_SetRxObserver(rawRxEvent);
    Console_Printf("XBee=%s CSV=%s\r\n", test_telecommands_xbee_ok ? "OK" : "ERROR", Logging_GetFilename());
    Console_Write("Ground bench: telemetry starts OFF. Send binary A5 01 5A / A5 02 5A / A5 03 5A over XBee.\r\n");
    Console_Write("Console: status, stream on, stream off. Calibration status: 0 idle, 1 running, 2 success, 3 failed.\r\n");
    Console_Write("Raw hex RX enabled (including invalid commands). Console: rx on, rx off. XBee status 3 = READY, not RF delivery confirmation.\r\n");
    lastTelemetry = lastLog = lastReport = HAL_GetTick();
    lastAcquisition = lastReport - ACQUISITION_PERIOD_MS;
}

void App_RunOnce(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastAcquisition) >= ACQUISITION_PERIOD_MS) {
        lastAcquisition = now;
        SensorAcquisition_Update(&test_telecommands_data);
    }
    test_telecommands_data.telemetry.uptimeMs = HAL_GetTick();
    if (test_telecommands_xbee_ok) Communication_Update(&test_telecommands_data);
    now = HAL_GetTick(); /* Calibration may have blocked during command handling. */
    test_telecommands_data.telemetry.uptimeMs = now;
    if ((uint32_t)(now - lastTelemetry) >= COMMUNICATION_TELEMETRY_PERIOD_MS) {
        lastTelemetry = now;
        if (test_telecommands_xbee_ok && test_telecommands_data.telemetry.enabled) {
            if (Communication_SendTelemetry(&test_telecommands_data)) ++test_telecommands_tx_accepted;
            else ++test_telecommands_tx_errors;
        }
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - lastLog) >= LOGGING_PERIOD_MS) {
        lastLog = now;
        if (Logging_IsReady()) {
            if (Logging_Write(&test_telecommands_data)) ++test_telecommands_logs;
            else { ++test_telecommands_log_errors; Console_Write("SD row write FAILED.\r\n"); }
        }
    }
    Console_Process();
    ConsoleLineResult result = Console_ReadLine(input, sizeof(input));
    if (result == CONSOLE_LINE_READY) {
        if (!strcmp(input, "status")) report();
        else if (!strcmp(input, "stream on")) stream = true;
        else if (!strcmp(input, "stream off")) stream = false;
        else if (!strcmp(input, "rx on")) rawRx = true;
        else if (!strcmp(input, "rx off")) rawRx = false;
        else Console_Write("Console: status, stream on/off, rx on/off. Mission commands must arrive over XBee.\r\n");
    } else if (result < 0) Console_Write("Console input lost/rejected; press Enter and retry.\r\n");
    now = HAL_GetTick();
    if (stream && (uint32_t)(now - lastReport) >= TELECOMMAND_REPORT_MS) {
        lastReport = now;
        report();
    }
}
