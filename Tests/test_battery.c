#include "App.h"
#include "Console.h"
#include "main.h"
#include "drivers/Battery.h"
#include <string.h>

volatile BatteryData battery_test_data;
volatile bool battery_test_initialized;
volatile uint32_t battery_test_samples, battery_test_failures;
static uint32_t last_sample;
static bool stream = true;
static char command[CONSOLE_LINE_MAX + 1U];

void App_Init(void)
{
    (void)Console_Init();
    battery_test_initialized = Battery_Init();
    Console_Write(battery_test_initialized ? "Battery test ready: ADC1 channel 12 / PC2.\r\n" :
        "Battery initialization FAILED; check ADC1 configuration.\r\n");
    Console_Printf("ADC reference=%.3f V divider ratio=%.3f. Commands: status, stream on, stream off, help.\r\n",
        (double)BATTERY_ADC_REFERENCE_V, (double)BATTERY_DIVIDER_RATIO);
    last_sample = HAL_GetTick() - 1000U;
}

void App_RunOnce(void)
{
    bool requested = false;
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) requested = true;
        else if (!strcmp(command, "stream on")) { stream = true; requested = true; }
        else if (!strcmp(command, "stream off")) { stream = false; Console_Write("Streaming off.\r\n"); }
        else Console_Write("status: measure now; stream on/off: periodic output; help.\r\n");
    } else if (input < 0) Console_Write("Input lost/rejected. Press Enter and resend.\r\n");

    uint32_t now = HAL_GetTick();
    if (!requested && (uint32_t)(now - last_sample) < 1000U) return;
    last_sample = now;
    BatteryData sample = {0};
    bool ok = battery_test_initialized && Battery_Read(&sample);
    battery_test_data = sample;
    if (ok) ++battery_test_samples;
    else ++battery_test_failures;
    if (!stream && !requested) return;
    if (ok) Console_Printf("Battery=%.3f V ADC raw=%u samples=%lu failures=%lu\r\n",
        (double)sample.voltage, (unsigned)sample.adcRaw,
        (unsigned long)battery_test_samples, (unsigned long)battery_test_failures);
    else Console_Printf("Battery: no valid measurement (initialized=%u failures=%lu).\r\n",
        (unsigned)battery_test_initialized, (unsigned long)battery_test_failures);
}
