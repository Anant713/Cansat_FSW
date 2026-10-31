/* Reuse the flight scheduler verbatim; only BMP I/O is substituted at link time. */
#define App_Init Hils_FlightInit
#define App_RunOnce Hils_FlightRunOnce
#include "../Core/Src/apps/FlightApp.c"
#undef App_Init
#undef App_RunOnce
#include "Console.h"
#include "hils/HilsInput.h"
#include <stdio.h>

#define HILS_REPORT_PERIOD_MS 200U
static uint32_t lastReport, logRows, logErrors, txErrors, badFrames;
static bool reportNow;

bool __real_Logging_Write(const Cansat *c);
bool __wrap_Logging_Write(const Cansat *c)
{
    bool ok = __real_Logging_Write(c);
    if (ok) ++logRows; else ++logErrors;
    return ok;
}
bool __real_Communication_SendTelemetry(Cansat *c);
bool __wrap_Communication_SendTelemetry(Cansat *c)
{
    bool ok = __real_Communication_SendTelemetry(c);
    if (!ok) ++txErrors;
    return ok;
}
static void commandEvent(GroundCommand id, CommandEvent event, bool success)
{
    Console_Printf("{\"type\":\"command\",\"id\":%u,\"event\":%u,\"ok\":%u}\r\n",
                   (unsigned)id, (unsigned)event, (unsigned)success);
    reportNow = true;
}
static void report(void)
{
    const Cansat *c = &flight_cansat;
    const StateMachineStatus *s = StateMachine_GetStatus();
    const HilsSample *input = HilsInput_Latest();
    uint32_t now = HAL_GetTick();
    bool fresh = c->bmp.valid && c->bmp.heightValid &&
        (uint32_t)(now - c->bmp.timestamp_ms) <= STATE_ALTITUDE_MAX_AGE_MS;
    static char json[1280];
    int n = snprintf(json, sizeof(json),
        "{\"type\":\"status\",\"ms\":%lu,\"seq\":%lu,\"input\":%u,\"age\":%lu,"
        "\"pressure\":%.2f,\"ground\":%.2f,\"height\":%.2f,\"fresh\":%u,"
        "\"state\":%u,\"peak\":%.2f,\"confirm\":%lu,\"recovered\":%u,\"flash\":%u,"
        "\"camera\":%u,\"recording\":%u,\"phase\":%u,\"hdrm\":%u,\"deployed\":%u,\"buzzer\":%u,"
        "\"telemetry\":%u,\"tx\":%u,\"tx_errors\":%lu,\"rx\":%u,\"command\":%u,\"executed\":%u,\"last_ok\":%u,"
        "\"cal\":%u,\"saved\":%u,\"logging\":%u,\"rows\":%lu,\"log_errors\":%lu,\"file\":\"%s\","
        "\"bad_frames\":%lu,\"rx_dropped\":%lu,\"rx_errors\":%lu,\"sensor_init\":%u,\"radio_init\":%u,\"state_init\":%u}\r\n",
        (unsigned long)now, (unsigned long)input->sequence, (unsigned)input->present,
        (unsigned long)(input->present ? now - input->receivedMs : 0),
        (double)c->bmp.pressure, (double)SensorAcquisition_GetGroundPressure(), (double)c->bmp.relativeHeightM, (unsigned)fresh,
        (unsigned)c->missionState, (double)s->maxHeightM, (unsigned long)s->heightConfirmCount,
        (unsigned)s->recovered, (unsigned)s->persistenceOK,
        (unsigned)c->camera.isOn, (unsigned)c->camera.isRecording, (unsigned)c->camera.phase,
        (unsigned)c->switches.hdrm, (unsigned)c->actuators.parafoilDeployed, (unsigned)c->switches.buzzer,
        (unsigned)c->telemetry.enabled, (unsigned)c->telemetry.packetCount, (unsigned long)txErrors,
        (unsigned)c->telemetry.receivedPacketCount, (unsigned)c->telemetry.lastReceivedCommand,
        (unsigned)c->telemetry.commandExecuted, (unsigned)c->telemetry.lastExecutedCommand,
        (unsigned)SensorAcquisition_GetCalibrationStatus(), (unsigned)SensorAcquisition_HasCalibration(),
        (unsigned)Logging_IsReady(), (unsigned long)logRows, (unsigned long)logErrors, Logging_GetFilename(),
        (unsigned long)badFrames, (unsigned long)console_diagnostics.rx_dropped_bytes,
        (unsigned long)console_diagnostics.rx_errors, (unsigned)flight_sensor_init_ok,
        (unsigned)flight_communication_init_ok, (unsigned)flight_state_init_ok);
    if (n > 0 && (size_t)n < sizeof(json)) {
        for (size_t at = 0; at < (size_t)n;) {
            size_t chunk = (size_t)n - at;
            if (chunk > CONSOLE_TX_SIZE) chunk = CONSOLE_TX_SIZE;
            if (!Console_WriteBytes(json + at, chunk)) break;
            at += chunk;
        }
    }
}
void App_Init(void)
{
    HilsInput_Init();
    /* Open console after blocking startup to avoid accumulating old PC samples. */
    Hils_FlightInit();
    (void)Console_Init();
    Communication_SetCommandObserver(commandEvent);
    Console_Write("{\"type\":\"boot\",\"protocol\":1,\"bmp\":\"simulated\",\"outputs\":\"REAL\"}\r\n");
    reportNow = true;
}
void App_RunOnce(void)
{
    char line[CONSOLE_LINE_MAX + 1];
    /* Bound serial work; never let a PC flood starve actuator timers. */
    for (unsigned i = 0; i < 4; ++i) {
        ConsoleLineResult result = Console_ReadLine(line, sizeof(line));
        if (result == CONSOLE_NO_LINE) break;
        if (result != CONSOLE_LINE_READY) { ++badFrames; continue; }
        uint8_t id = 0;
        switch (HilsInput_Parse(line, HAL_GetTick(), &id)) {
        case HILS_PRESSURE: break;
        case HILS_COMMAND: {
            uint8_t packet[] = {0xa5, id, 0x5a};
            Communication_ProcessCommand(&flight_cansat, packet, sizeof(packet));
            break;
        }
        case HILS_STATUS: reportNow = true; break;
        case HILS_NEW_MISSION: {
            /* Test-console operation, not a fourth flight telecommand. */
            bool ok = SensorAcquisition_GetCalibrationStatus() != SENSOR_CAL_RUNNING &&
                      StateMachine_BeginNewMission(&flight_cansat);
            if (ok) {
                flight_cansat.bmp.valid = flight_cansat.bmp.heightValid = false;
                HilsInput_Init();
                ok = Logging_Stop() && Logging_Init();
                logRows = 0;
            }
            Console_Printf("{\"type\":\"new\",\"ok\":%u}\r\n", (unsigned)ok);
            reportNow = true;
            break;
        }
        case HILS_RESET:
            Console_Write("{\"type\":\"reset\"}\r\n");
            NVIC_SystemReset(); /* No orderly shutdown: exercise genuine reset recovery. */
            break;
        default: ++badFrames; break;
        }
    }
    Hils_FlightRunOnce();
    if (reportNow || (uint32_t)(HAL_GetTick() - lastReport) >= HILS_REPORT_PERIOD_MS) {
        reportNow = false; report(); lastReport = HAL_GetTick();
    }
}
