# Battery voltage measurement

`Battery.c` reads PC2 / ADC1 channel 12 through the configured 4:1 resistor
divider. `BatteryData.voltage` is the battery voltage, not the voltage at PC2:

`battery volts = adcRaw / 4095 * BATTERY_ADC_REFERENCE_V * BATTERY_DIVIDER_RATIO`

Defaults are 3.3 V reference and a divider ratio of 4.0. Use the actual VREF+/VDDA
and resistor ratio for better accuracy. The constants in Battery.h can also be
overridden through compiler definitions. No percentage or cutoff is inferred.

The flight application's App_Init initializes the driver after CubeMX initializes
ADC1. Other applications/tests must call Battery_Init after peripheral setup.

```c
#include "drivers/Battery.h"

BatteryData battery;
if (Battery_Read(&battery)) {
    // Log battery.voltage (volts) and battery.adcRaw (0..4095).
} else {
    // battery.valid is false; do not use this reading.
}
```

Each call performs one software-triggered 12-bit, right-aligned conversion,
waits for completion, reads it and stops the ADC. It is a short blocking call;
there is no DMA/IRQ dependency beyond the HAL timebase. The conversion wait has
a 2 ms HAL timeout with tick granularity. Call only from main context with the
tick/interrupts running. Caller scheduling determines the measurement rate.
Failures clear the entire supplied data structure; NULL returns false.

Sampling time is set to 480 cycles in the IOC/generated ADC setup and reasserted
by Battery_Init. At the current ADC clock of 8 MHz (16 MHz PCLK2 / 2), sampling
plus 12-bit conversion takes approximately 61.5 microseconds, excluding HAL
startup/software overhead. This longer acquisition period allows more settling
time through a resistor divider. Actual accuracy still depends on resistor
values, input capacitance, reference accuracy and board noise. See
[ST's ADC accuracy guidance](https://www.st.com/resource/en/application_note/dm00050879.pdf).

A 12.6 V pack produces 3.15 V at PC2 with an exact 4:1 divider. With a 3.3 V
reference the nominal measurement range ends at 13.2 V battery voltage.
Full-scale raw 4095 is a successful conversion but may represent saturation;
the driver cannot distinguish higher voltages. `valid` does not imply that the
battery is healthy, connected, or within an operating range. Keep the physical
ADC input within its permitted voltage range; software cannot protect it.

Verify against a multimeter at several battery voltages. No physical ADC or
divider measurements have been performed during implementation.

Host regression test (MinGW GCC, from project root):

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ITests/host/battery -ICore/Inc Tests/host/battery/test_battery.c Core/Src/drivers/Battery.c -o build/driver-tools/test_battery.exe
./build/driver-tools/test_battery.exe
```
