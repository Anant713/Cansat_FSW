#include "App.h"
#include "Console.h"
#include "main.h"
#include "drivers/GNSS.h"
#include <string.h>

#define GNSS_TEST_CURRENT_BAUD 9600U
#define GNSS_TEST_TARGET_BAUD  9600U
#define GNSS_TEST_PERIOD_MS    1000U /* 500=2 Hz, 200=5 Hz */
extern UART_HandleTypeDef huart6;
typedef enum {
    GNSS_TEST_INIT_ERROR = 0, GNSS_TEST_WAITING_FOR_BYTES, GNSS_TEST_WAITING_FOR_GGA,
    GNSS_TEST_NO_FIX, GNSS_TEST_FIX, GNSS_TEST_STALE, GNSS_TEST_RX_ERROR
} GNSSTestState;
volatile GNSSTestState gnss_test_state;
volatile GNSSData gnss_test_data;
volatile GNSSDiagnostics gnss_test_diagnostics;
volatile bool gnss_test_fresh;
volatile uint32_t gnss_test_reports;
static bool initialized, configuration_started;
static uint32_t started_ms, last_report, last_recovery;
static bool stream = true;
static char command[CONSOLE_LINE_MAX + 1U];

static void report(void)
{
    GNSSData d = gnss_test_data;
    Console_Printf("GNSS state=%u %s satellites=%u quality=%u bytes=%lu drops=%lu config=%u\r\n",
        (unsigned)gnss_test_state, !d.valid ? "STALE/NO DATA" : d.fixValid ? "FIX" : "NO FIX",
        (unsigned)d.satellites, (unsigned)d.fixQuality,
        (unsigned long)gnss_test_diagnostics.receivedBytes,
        (unsigned long)gnss_test_diagnostics.queueDrops, (unsigned)gnss_test_diagnostics.configState);
    /* A fresh GGA does not imply fresh VTG: respect each validity flag. */
    if (d.utcValid) Console_Printf("UTC=%02lu:%02lu:%02lu.%03u ",
        (unsigned long)(d.utcTime / 10000U), (unsigned long)(d.utcTime / 100U % 100U),
        (unsigned long)(d.utcTime % 100U), (unsigned)d.utcMilliseconds);
    else Console_Write("UTC=N/A ");
    if (d.fixValid) Console_Printf("lat=%.6f deg lon=%.6f deg ", d.latitude, d.longitude);
    else Console_Write("lat=N/A lon=N/A ");
    if (d.speedValid) Console_Printf("speed=%.3f m/s ", (double)d.speed);
    else Console_Write("speed=N/A ");
    if (d.courseValid) Console_Printf("course=%.2f deg true\r\n", (double)d.course);
    else Console_Write("course=N/A\r\n");
}

void App_Init(void)
{
    (void)Console_Init();
    initialized = GNSS_Init(&huart6, GNSS_TEST_CURRENT_BAUD);
    gnss_test_state = initialized ? GNSS_TEST_WAITING_FOR_BYTES : GNSS_TEST_INIT_ERROR;
    started_ms = last_report = last_recovery = HAL_GetTick();
    Console_Write(initialized ? "GNSS test ready. Commands: status, stream on, stream off, help.\r\n" :
        "GNSS initialization FAILED. Commands: status, help.\r\n");
}

void App_RunOnce(void)
{
    Console_Process();
    bool requested = false;
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) requested = true;
        else if (!strcmp(command, "stream on")) { stream = true; Console_Write("Streaming on.\r\n"); }
        else if (!strcmp(command, "stream off")) { stream = false; Console_Write("Streaming off.\r\n"); }
        else Console_Write("Commands: status, stream on, stream off, help.\r\n");
    } else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    if (!initialized) { if (requested) report(); return; }
    GNSS_Process(); /* Run frequently; get a snapshot when acquisition needs it. */
    uint32_t now = HAL_GetTick();
    GNSSDiagnostics diagnostics;
    GNSS_GetDiagnostics(&diagnostics);
    if (!diagnostics.receiving && (uint32_t)(now-last_recovery) >= 1000U) {
        last_recovery = now;
        (void)GNSS_StartReception();
    }
    /* Wait for a complete NMEA sentence to prove the initial baud is correct. */
    if (!configuration_started && (diagnostics.goodGga || diagnostics.goodVtg) &&
        (uint32_t)(now-started_ms) >= 1000U) {
        configuration_started = GNSS_Configure(GNSS_TEST_PERIOD_MS, GNSS_TEST_TARGET_BAUD);
    }
    if (!requested && (uint32_t)(now-last_report) < 1000U) return;
    last_report = now;
    GNSSData data;
    gnss_test_fresh = GNSS_GetData(&data);
    GNSS_GetDiagnostics(&diagnostics);
    gnss_test_data = data;
    gnss_test_diagnostics = diagnostics;
    if (!diagnostics.receiving) gnss_test_state = GNSS_TEST_RX_ERROR;
    else if (!diagnostics.receivedBytes) gnss_test_state = GNSS_TEST_WAITING_FOR_BYTES;
    else if (!diagnostics.goodGga) gnss_test_state = GNSS_TEST_WAITING_FOR_GGA;
    else if (!data.valid) gnss_test_state = GNSS_TEST_STALE;
    else gnss_test_state = data.fixValid ? GNSS_TEST_FIX : GNSS_TEST_NO_FIX;
    ++gnss_test_reports;
    if (stream || requested) report();
}
