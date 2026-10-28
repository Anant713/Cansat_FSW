#include "App.h"
#include "main.h"
#include "Console.h"
#include "apps/Cansat.h"
#include "apps/SensorAcquisition.h"
#include "apps/Logging.h"
#include "drivers/SDCard.h"
#include <string.h>
#ifndef TEST_WITH_TELEMETRY
#define TEST_WITH_TELEMETRY 0
#endif
#if TEST_WITH_TELEMETRY
#include "apps/Communication.h"

#endif

#define TIMING_REPORT_PERIOD_MS 5000U
/* Same task periods as flight. Durations include interrupt service time. */
typedef struct {
    uint32_t count, lastStartMs, lastCycles, minCycles, maxCycles;
    uint64_t totalCycles;
} TimingStats;
Cansat test_acquisition_logging_data;
TimingStats test_acquisition_time, test_logging_time, test_loop_period, test_loop_work;
bool test_acquisition_init_ok, test_logging_init_ok, test_timing_clock_ok;
uint32_t test_acquisition_init_ms, test_logging_init_ms;
uint32_t test_log_successes, test_log_failures, test_acquisition_overruns, test_logging_overruns;
uint32_t test_timing_invalid_intervals;
static uint32_t lastAcquisition, lastLog, lastReport, previousLoopCycles, previousLoopMs;
static bool havePreviousLoop, stream = true, stopped;
static char command[CONSOLE_LINE_MAX + 1U];

static void measure(TimingStats *s, uint32_t startCycles, uint32_t startMs)
{
    uint32_t cycles = DWT->CYCCNT - startCycles;
    /* Do not silently accept measurements spanning a full counter wrap.
     * At 168 MHz the counter wraps every ~25.6 seconds. */
    if (!test_timing_clock_ok || (uint32_t)(HAL_GetTick() - startMs) >= UINT32_MAX / (SystemCoreClock / 1000U)) {
        ++test_timing_invalid_intervals;
        return;
    }
    s->lastStartMs = startMs; s->lastCycles = cycles;
    if (!s->count || cycles < s->minCycles) s->minCycles = cycles;
    if (cycles > s->maxCycles) s->maxCycles = cycles;
    s->totalCycles += cycles;
    ++s->count;
}
static double us(double cycles) { return cycles * 1000000.0 / SystemCoreClock; }
static double average_us(const TimingStats *s)
{
    return s->count ? us((double)s->totalCycles / s->count) : 0.0;
}
#if TEST_WITH_TELEMETRY
TimingStats test_telemetry_time;
bool test_telemetry_init_ok;
uint32_t test_telemetry_successes, test_telemetry_failures;
static uint32_t lastTelemetry;
#endif
static void report(void)
{
    if (!test_timing_clock_ok) { Console_Write("Timing unavailable: cycle counter is not running.\r\n"); return; }
    double acq = average_us(&test_acquisition_time);
    double log = average_us(&test_logging_time);
    Console_Printf("Avg sensor acquisition: %.2f us\r\n", acq);
    Console_Printf("Avg logging: %.2f us\r\n", log);
#if TEST_WITH_TELEMETRY
    double telemetry = average_us(&test_telemetry_time);
    Console_Printf("Avg telemetry: %.2f us\r\n", telemetry);
    Console_Printf("Avg main loop (task sum): %.2f us\r\n\r\n", acq + log + telemetry);
#else
    Console_Printf("Avg main loop (task sum): %.2f us\r\n\r\n", acq + log);
#endif
}
void App_Init(void)
{
    (void)Console_Init();
    Cansat_Init(&test_acquisition_logging_data);
    uint32_t start = HAL_GetTick();
    test_logging_init_ok = Logging_Init(); /* SD startup before GNSS reception. */
    test_logging_init_ms = HAL_GetTick() - start;
    start = HAL_GetTick();
    test_acquisition_init_ok = SensorAcquisition_Init();
    test_acquisition_init_ms = HAL_GetTick() - start;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP();
    test_timing_clock_ok = DWT->CYCCNT != before;
#if TEST_WITH_TELEMETRY
    test_telemetry_init_ok = Communication_Init();
    test_acquisition_logging_data.telemetry.enabled = test_telemetry_init_ok;
    if (!test_telemetry_init_ok) Console_Write("XBee initialization failed.\r\n");
#endif
    if (!test_acquisition_init_ok) Console_Write("Some sensors failed initialization.\r\n");
    if (!test_logging_init_ok) Console_Write("SD logging initialization failed.\r\n");
    Console_Printf("CSV: %s\r\n", Logging_GetFilename());
    Console_Write("Commands: status, stream on, stream off, stop.\r\n");
    lastLog = lastReport = HAL_GetTick();
    lastAcquisition = lastLog - ACQUISITION_PERIOD_MS;
#if TEST_WITH_TELEMETRY
    lastTelemetry = lastLog;
#endif
}
void App_RunOnce(void)
{
    uint32_t loopCycles = DWT->CYCCNT, loopMs = HAL_GetTick();
    if (havePreviousLoop) measure(&test_loop_period, previousLoopCycles, previousLoopMs);
    previousLoopCycles = loopCycles; previousLoopMs = loopMs; havePreviousLoop = true;
    uint32_t now = HAL_GetTick();
    if (!stopped && (uint32_t)(now - lastAcquisition) >= ACQUISITION_PERIOD_MS) {
        lastAcquisition = now;
        uint32_t cycles = DWT->CYCCNT;
        SensorAcquisition_Update(&test_acquisition_logging_data);
        measure(&test_acquisition_time, cycles, now);
        if ((uint32_t)(HAL_GetTick() - now) > ACQUISITION_PERIOD_MS) ++test_acquisition_overruns;
    }
    now = HAL_GetTick();
    test_acquisition_logging_data.telemetry.uptimeMs = now;
#if TEST_WITH_TELEMETRY
    if (test_telemetry_init_ok) Communication_Update(&test_acquisition_logging_data);
    now = HAL_GetTick();
    test_acquisition_logging_data.telemetry.uptimeMs = now;
    if (!stopped && test_telemetry_init_ok &&
        (uint32_t)(now - lastTelemetry) >= COMMUNICATION_TELEMETRY_PERIOD_MS) {
        lastTelemetry = now;
        uint32_t cycles = DWT->CYCCNT;
        bool ok = Communication_SendTelemetry(&test_acquisition_logging_data);
        measure(&test_telemetry_time, cycles, now);
        if (ok) ++test_telemetry_successes; else ++test_telemetry_failures;
    }
    /* TX first: the accepted packet count is reflected in a simultaneous log. */
    now = HAL_GetTick();
#endif
    if (!stopped && Logging_IsReady() && (uint32_t)(now - lastLog) >= LOGGING_PERIOD_MS) {
        lastLog = now;
        uint32_t cycles = DWT->CYCCNT;
        bool ok = Logging_Write(&test_acquisition_logging_data);
        measure(&test_logging_time, cycles, now);
        if ((uint32_t)(HAL_GetTick() - now) > LOGGING_PERIOD_MS) ++test_logging_overruns;
        if (ok) ++test_log_successes; else ++test_log_failures;
    }
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) report();
        else if (!strcmp(command, "stream on")) stream = true;
        else if (!strcmp(command, "stream off")) stream = false;
        else if (!strcmp(command, "stop")) {
            stopped = true;
            bool ok = Logging_Stop();
            Console_Write(ok ? "Stopped; file closed. Reset to start a new run.\r\n" : "Close failed; retry stop.\r\n");
            report();
        } else Console_Write("Commands: status, stream on, stream off, stop.\r\n");
    } else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    now = HAL_GetTick();
    if (stream && (uint32_t)(now - lastReport) >= TIMING_REPORT_PERIOD_MS) { lastReport = now; report(); }
    measure(&test_loop_work, loopCycles, loopMs);
}

