#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#ifdef TEST_DRONE_NO_COMMANDS
#include "../../HDRM_drone_no_commands_test.c"
#else
#include "../../test_acquisition_logging.c"
#endif
MockDWT mockDwt;
MockCoreDebug mockDebug;
uint32_t SystemCoreClock = 1000000;
static uint32_t tick, acquireCalls, logCalls;
static bool ready;
static const char *pendingCommand;
static char consoleOutput[4096];
static void advance(uint32_t ms);
#ifndef TEST_DRONE_NO_COMMANDS
static ServoData startupServo;
static bool startupLed;
bool Switches_Init(void) { startupLed = false; return true; }
void Switches_LED(bool on) { startupLed = on; }
void Switches_Update(SwitchState *s) { s->led = startupLed; }
bool Servo_Init(void) { startupServo = (ServoData){0}; return true; }
void Servo_SetAngles(float a, float b) { assert(a == SERVO_MIN_ANGLE_DEG && b == SERVO_MIN_ANGLE_DEG); startupServo.servo1Angle = a; startupServo.servo2Angle = b; }
void Servo_Enable(void) { startupServo.enabled = true; }
bool Servo_GetData(ServoData *s) { *s = startupServo; return true; }
#endif
#if TEST_WITH_TELEMETRY
static unsigned telemetryCalls, communicationUpdates;
bool Communication_Init(void) { return true; }
void Communication_Update(Cansat *c) { (void)c; ++communicationUpdates; }
bool Communication_SendTelemetry(Cansat *c)
{
    ++telemetryCalls; ++c->telemetry.packetCount; advance(1); return true;
}
#endif
uint32_t HAL_GetTick(void) { return tick; }
static void advance(uint32_t ms) { tick += ms; mockDwt.CYCCNT += ms * 1000; }
bool Console_Init(void) { return true; }
void Console_Process(void) {}
bool Console_Write(const char *s) { (void)s; return true; }
bool Console_Printf(const char *s, ...)
{
    va_list args;
    va_start(args, s);
    size_t used = strlen(consoleOutput);
    int written = vsnprintf(consoleOutput + used, sizeof(consoleOutput) - used, s, args);
    va_end(args);
    assert(written >= 0 && (size_t)written < sizeof(consoleOutput) - used);
    return true;
}
ConsoleLineResult Console_ReadLine(char *s, size_t capacity)
{
    if (!pendingCommand) return CONSOLE_NO_LINE;
    assert(strlen(pendingCommand) < capacity);
    strcpy(s, pendingCommand); pendingCommand = NULL; return CONSOLE_LINE_READY;
}
bool SensorAcquisition_Init(void) { return true; }
#ifndef TEST_DRONE_NO_COMMANDS
bool SensorAcquisition_CalibrateAtStartup(Cansat *c) { assert(c); return true; }
float SensorAcquisition_GetGroundPressure(void) { return 101325.0f; }
#endif
void SensorAcquisition_Update(Cansat *c)
{
    ++acquireCalls; advance(2);
    c->bmp = (BMP280Data){.valid=true,.heightValid=true,.relativeHeightM=12.5f};
}
bool Logging_Init(void) { ready = true; return true; }
bool Logging_IsReady(void) { return ready; }
const char *Logging_GetFilename(void) { return "0:/LOG0000.CSV"; }
bool Logging_Stop(void) { ready = false; return true; }
FRESULT SDCard_GetLastResult(void) { return FR_OK; }
bool Logging_Write(const Cansat *c)
{
    assert(c->bmp.heightValid && c->bmp.relativeHeightM == 12.5f);
        assert(!c->attitude.quaternionValid);
#if TEST_WITH_TELEMETRY
    assert(c->telemetry.packetCount == telemetryCalls && telemetryCalls > 0);
#else
    assert(c->telemetry.packetCount == 0);
#endif
    ++logCalls; advance(5); return true;
}
int main(void)
{
    App_Init(); assert(test_timing_clock_ok);
    App_RunOnce(); assert(acquireCalls == 1 && logCalls == 0);
    advance(7); App_RunOnce(); assert(acquireCalls == 1);
    advance(1); App_RunOnce(); assert(acquireCalls == 2);
    advance(988); App_RunOnce();
    assert(acquireCalls == 3 && logCalls == 1);
    assert(test_acquisition_time.lastCycles == 2000 && test_logging_time.lastCycles == 5000);
    #if TEST_WITH_TELEMETRY
    assert(test_loop_work.lastCycles == 8000 && test_log_successes == 1);
    assert(telemetryCalls == 1 && test_telemetry_time.lastCycles == 1000);
#else
    assert(test_loop_work.lastCycles == 7000 && test_log_successes == 1);
#endif
        assert(test_acquisition_time.lastStartMs == 1000);
#if TEST_WITH_TELEMETRY
    assert(test_logging_time.lastStartMs == 1003);
#else
    assert(test_logging_time.lastStartMs == 1002);
#endif
    App_RunOnce(); assert(acquireCalls == 3 && logCalls == 1); /* no catch-up burst */
    consoleOutput[0] = 0;
    report();
    assert(strstr(consoleOutput, "Avg sensor acquisition: 2000.00 us"));
    assert(strstr(consoleOutput, "Avg logging: 5000.00 us"));
#if TEST_WITH_TELEMETRY
    assert(strstr(consoleOutput, "Avg telemetry: 1000.00 us"));
    assert(strstr(consoleOutput, "Avg main loop (task sum): 8000.00 us"));
#else
    assert(!strstr(consoleOutput, "Avg telemetry"));
    assert(strstr(consoleOutput, "Avg main loop (task sum): 7000.00 us"));
#endif
    assert(!strstr(consoleOutput, "start_ms") && !strstr(consoleOutput, "max="));
    pendingCommand = "stream off"; App_RunOnce(); assert(!stream);
    pendingCommand = "stop"; App_RunOnce(); assert(stopped && !ready);
    advance(2000); App_RunOnce(); assert(acquireCalls == 3 && logCalls == 1);
    #if TEST_WITH_TELEMETRY
    assert(telemetryCalls == 1 && communicationUpdates > 0);
#endif
    puts("Combined test scheduling and timing passed: task periods, measured durations, defaults and stop.");
    return 0;
}


