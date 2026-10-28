#include "App.h"
#include "Console.h"
#include "drivers/GNSS.h"
#include <string.h>

#define CURRENT_BAUD 9600U
#define TARGET_BAUD 38400U
#define PERIOD_MS 200U
extern UART_HandleTypeDef huart6;
volatile GNSSData gnss_nav_data;
volatile GNSSDiagnostics gnss_nav_diagnostics;
static bool initialized, configured, stream = true;
static uint32_t last_report, last_recovery;
static char command[CONSOLE_LINE_MAX + 1U];

static void report(const GNSSData *d, const GNSSDiagnostics *s)
{
    Console_Printf("GNSS %s sats=%u quality=%u config=%u step=%u GGA/VEL/UTC=%lu/%lu/%lu drops=%lu checksum=%lu malformed=%lu UART=%lu last=0x%lX\r\n",
        !d->valid ? "STALE/NO GGA" : d->fixValid ? "FIX" : "NO FIX",
        (unsigned)d->satellites, (unsigned)d->fixQuality, (unsigned)s->configState, (unsigned)s->configStep,
        (unsigned long)s->goodGga, (unsigned long)s->goodVelned, (unsigned long)s->goodTimeutc,
        (unsigned long)s->queueDrops, (unsigned long)s->badChecksum, (unsigned long)s->malformed,
        (unsigned long)s->uartErrors, (unsigned long)s->lastUartError);
    if (d->fixValid) Console_Printf("Lat=%.7f (%c) Lon=%.7f (%c) ",
        d->latitude, d->latitudeHemisphere, d->longitude, d->longitudeHemisphere);
    else Console_Write("Lat/Lon=N/A ");
    if (d->altitudeValid) Console_Printf("MSL=%.2f m ", (double)d->altitude);
    else Console_Write("MSL=N/A ");
    if (d->hdopValid) Console_Printf("HDOP=%.2f\r\n", (double)d->hdop);
    else Console_Write("HDOP=N/A\r\n");
    const GNSSNavData *n = &d->nav;
    if (n->velnedFresh) {
        Console_Printf("VELNED %s iTOW=%lu N/E/D=%.2f/%.2f/%.2f m/s sAcc=%.2f m/s cAcc=%.5f deg\r\n",
            n->velocityValid ? "fresh GGA fix" : "UNVALIDATED (no fresh GGA fix)",
            (unsigned long)n->velocityTowMs, (double)n->northVelocity, (double)n->eastVelocity,
            (double)n->downVelocity, (double)n->speedAccuracy, (double)n->courseAccuracy);
        Console_Printf("3D speed=%.2f ground speed=%.2f m/s course=%.5f deg; UBX epochs match=%u\r\n",
            (double)n->speed3D, (double)n->groundSpeed, (double)n->course, (unsigned)n->navEpochMatched);
    } else Console_Write("VELNED=N/A (waiting/stale)\r\n");
    if (n->timeutcFresh) {
        Console_Printf("TIMEUTC flags=0x%02X (TOW/WEEK/UTC=%u/%u/%u) iTOW=%lu tAcc=%lu ns\r\n",
            (unsigned)n->timeValidityFlags, (unsigned)(n->timeValidityFlags & 1U),
            (unsigned)((n->timeValidityFlags >> 1) & 1U), (unsigned)((n->timeValidityFlags >> 2) & 1U),
            (unsigned long)n->timeTowMs, (unsigned long)n->timeAccuracyNs);
        if (n->utcDateTimeValid) Console_Printf("UTC=%04u-%02u-%02u %02u:%02u:%02u plus signed nano=%ld ns\r\n",
            (unsigned)n->year, (unsigned)n->month, (unsigned)n->day,
            (unsigned)n->hour, (unsigned)n->minute, (unsigned)n->second, (long)n->nanosecondCorrection);
        else Console_Write("UTC date/time INVALID\r\n");
    } else Console_Write("TIMEUTC=N/A (waiting/stale)\r\n");
}
void App_Init(void)
{
    (void)Console_Init();
    initialized = GNSS_Init(&huart6, CURRENT_BAUD);
    Console_Write(initialized ? "GNSS NAV test ready: GGA + NAV-VELNED + NAV-TIMEUTC, target 5 Hz / 38400 baud.\r\n" : "GNSS init FAILED.\r\n");
    Console_Write("Commands: status, stream on, stream off, configure, help.\r\n");
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
    GNSSData data;
    GNSS_GetDiagnostics(&diag);
    uint32_t now = HAL_GetTick();
    if (!diag.receiving && (uint32_t)(now-last_recovery) >= 1000U) {
        last_recovery = now; (void)GNSS_StartReception();
    }
    if (retry || (!configured && (diag.goodGga || diag.goodVtg || diag.goodVelned || diag.goodTimeutc))) {
        bool ok = GNSS_Configure(PERIOD_MS, TARGET_BAUD);
        if (ok) configured = true;
        Console_Write(ok ? "Configuration started.\r\n" : "Configuration not started; check status.\r\n");
    }
    (void)GNSS_GetData(&data); /* NAV validity is independent of the return value. */
    GNSS_GetDiagnostics(&diag);
    gnss_nav_data = data; gnss_nav_diagnostics = diag;
    if (requested || (stream && (uint32_t)(now-last_report) >= 1000U)) {
        last_report = now; report(&data, &diag);
    }
}
