#include "App.h"
#include "Console.h"
#include "drivers/Servo.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

static char line[CONSOLE_LINE_MAX + 1U];
static bool ready;

static void report(void)
{
    ServoData data;
    if (!Servo_GetData(&data)) { Console_Write("Servo driver not initialized.\r\n"); return; }
    Console_Printf("PWM=%s requested angles: servo1=%.2f servo2=%.2f deg\r\n",
        data.enabled ? "ON" : "OFF", (double)data.servo1Angle, (double)data.servo2Angle);
}

void App_Init(void)
{
    (void)Console_Init();
    ready = Servo_Init();
    Console_Write(ready ? "Servo test ready; PWM disabled, saved angles 5 deg.\r\n" :
        "Servo init failed: check TIM4 1 MHz counter / 20 ms period.\r\n");
    Console_Write("Commands: servo1 <5..45>, servo2 <5..45>, enable, disable, status, help.\r\n");
}

void App_RunOnce(void)
{
    ConsoleLineResult input = Console_ReadLine(line, sizeof(line));
    if (input < 0) { Console_Write("Input lost/rejected. Press Enter and resend.\r\n"); return; }
    if (input != CONSOLE_LINE_READY) return;
    if (!strcmp(line, "help")) {
        Console_Write("servo1 45 or servo2 30 sets angle; enable starts BOTH servos; disable stops BOTH; status.\r\n");
        return;
    }
    if (!ready) { Console_Write("Servo initialization failed.\r\n"); return; }
    if (!strcmp(line, "enable")) Servo_Enable();
    else if (!strcmp(line, "disable")) Servo_Disable();
    else if (!strcmp(line, "status")) { report(); return; }
    else {
        bool first = !strncmp(line, "servo1 ", 7);
        if (!first && strncmp(line, "servo2 ", 7)) {
            Console_Write("Unknown command. Type help.\r\n"); return;
        }
        char *end;
        float angle = strtof(line + 7, &end);
        bool parsed = end != line + 7;
        while (isspace((unsigned char)*end)) ++end;
        if (!parsed || *end || !isfinite(angle) ||
            angle < SERVO_MIN_ANGLE_DEG || angle > SERVO_MAX_ANGLE_DEG) {
            Console_Write("Enter a finite angle from 5 to 45 degrees.\r\n"); return;
        }
        if (first) Servo1_SetAngle(angle); else Servo2_SetAngle(angle);
    }
    report();
}
