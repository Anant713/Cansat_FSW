#include "App.h"
#include "Console.h"
#include "drivers/Switches.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

static char line[CONSOLE_LINE_MAX + 1U];
static bool ready;
static void report(void)
{
    SwitchState state;
    Switches_Update(&state);
    Console_Printf("Output latches: HDRM=%s LED=%s BUZZER=%s\r\n",
        state.hdrm ? "ON" : "OFF", state.led ? "ON" : "OFF", state.buzzer ? "ON" : "OFF");
}
void App_Init(void)
{
    (void)Console_Init();
    ready = Switches_Init();
    Console_Write(ready ? "Switch test ready; all switches OFF.\r\n" : "Switch init failed.\r\n");
    Console_Write("Commands: hdrm on/off, led on/off, buzzer on/off, all off, status, help.\r\n");
}
void App_RunOnce(void)
{
    ConsoleLineResult input = Console_ReadLine(line, sizeof(line));
    if (input < 0) { Console_Write("Input lost/rejected. Press Enter and resend.\r\n"); return; }
    if (input != CONSOLE_LINE_READY) return;
    for (char *p = line; *p; ++p) *p = (char)tolower((unsigned char)*p);
    if (!strcmp(line, "help")) {
        Console_Write("hdrm on/off, led on/off, buzzer on/off, all off, status. ON stays high until OFF/reset.\r\n"); return;
    }
    if (!ready) { Console_Write("Switch initialization failed.\r\n"); return; }
    if (!strcmp(line, "status")) { report(); return; }
    char name[16], value[8], extra;
    if (sscanf(line, "%15s %7s %c", name, value, &extra) != 2 ||
        (strcmp(value, "on") && strcmp(value, "off"))) {
        Console_Write("Invalid command. Type help.\r\n"); return;
    }
    bool on = !strcmp(value, "on");
    if (!strcmp(name, "hdrm")) Switches_HDRM(on);
    else if (!strcmp(name, "led")) Switches_LED(on);
    else if (!strcmp(name, "buzzer")) Switches_Buzzer(on);
    else if (!strcmp(name, "all") && !on) {
        Switches_HDRM(false); Switches_LED(false); Switches_Buzzer(false);
    } else { Console_Write("Unknown switch/command. Type help.\r\n"); return; }
    report();
}
