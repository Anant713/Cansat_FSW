#include "App.h"
#include "Console.h"
#include "drivers/GNSS.h"
#include <string.h>

extern UART_HandleTypeDef huart6;
volatile GNSSSatInView gnss_view_data;
volatile GNSSDiagnostics gnss_view_diagnostics;
static bool initialized, configured, stream = true;
static uint32_t last_report, last_recovery;
static char command[CONSOLE_LINE_MAX + 1U];

void App_Init(void)
{
    (void)Console_Init();
    initialized = GNSS_Init(&huart6, 9600);
    Console_Write(initialized ? "GNSS satellites-in-view test: USART6 initial 9600 baud; target 38400 baud, 5 Hz.\r\n" :
        "GNSS init failed.\r\n");
    Console_Write("Commands: status, stream on, stream off, configure, help. Console: 115200, newline enabled.\r\n");
}

void App_RunOnce(void)
{
    bool requested = false, retry = false;
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) requested = true;
        else if (!strcmp(command, "stream on")) stream = true;
        else if (!strcmp(command, "stream off")) stream = false;
        else if (!strcmp(command, "configure")) retry = true;
        else Console_Write("status, stream on, stream off, configure, help\r\n");
    } else if (input < 0) Console_Write("Input lost/rejected. Press Enter and resend.\r\n");
    if (!initialized) return;
    GNSS_Process();
    GNSSDiagnostics diag;
    GNSSSatInView view;
    GNSSData fix;
    GNSS_GetDiagnostics(&diag);
    (void)GNSS_GetSatInView(&view);
    uint32_t now = HAL_GetTick();
    if (!diag.receiving && (uint32_t)(now-last_recovery) >= 1000U) {
        last_recovery = now;
        (void)GNSS_StartReception();
    }
    if (retry || (!configured && (diag.goodGga || diag.goodVtg || view.goodSentences))) {
        bool ok = GNSS_Configure(200, 38400);
        if (ok) configured = true;
        Console_Write(ok ? "Configuring GGA + VTG + GPGSV at 5 Hz / 38400 baud.\r\n" : "Configuration not started; check RX/config state.\r\n");
    }
    gnss_view_data = view;
    gnss_view_diagnostics = diag;
    if (!requested && (!stream || (uint32_t)(now-last_report) < 1000U)) return;
    last_report = now;
    (void)GNSS_GetData(&fix);
    if (view.valid) Console_Printf("In view=%u GSV part=%u/%u ",
        (unsigned)view.satellitesInView, (unsigned)view.messageNumber, (unsigned)view.totalMessages);
    else Console_Write("In view=N/A (waiting/stale) ");
    if (fix.valid) Console_Printf("used in fix=%u %s ", (unsigned)fix.satellites, fix.fixValid ? "FIX" : "NO FIX");
    else Console_Write("GGA=N/A ");
    Console_Printf("GSV=%lu bytes=%lu drops=%lu malformed=%lu checksum=%lu uart_errors=%lu config=%u\r\n",
        (unsigned long)view.goodSentences, (unsigned long)diag.receivedBytes,
        (unsigned long)diag.queueDrops, (unsigned long)diag.malformed,
        (unsigned long)diag.badChecksum, (unsigned long)diag.uartErrors, (unsigned)diag.configState);
}
