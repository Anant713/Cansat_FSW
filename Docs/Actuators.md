# Switch and servo drivers

CubeMX owns peripheral initialization. The flight `App_Init()` calls
`Switches_Init()` and `Servo_Init()` afterward. Both sources are registered in
the top-level CMake target. Test applications call these init functions themselves.

Switch outputs are active high: PA12 LED, PA11 buzzer, PA10 DVR power, PA8 HDRM.
Init resets all four plus PA9 DVR control to LOW without modifying unrelated pins.
Setters are ignored before initialization. `Switches_Update(&state)` reads the
output latches; it does not measure whether the attached load actually turned on.
`Switch_Capture(bool)` sets PA9's level, not a timed capture pulse.

TIM4 CH1/PB6 and CH2/PB7 generate the servo waveforms in hardware. Current settings:
16 MHz timer input, PSC=15, ARR=19999, up counting, PWM1, active high. The counter
ticks every microsecond and repeats at 50 Hz. Init checks the clock, prescaler,
period and PWM configuration; it returns false on a mismatch rather than silently
generating the wrong pulse widths. Keep TIM4 dedicated to the two servos.

`Servo_Init()` leaves PWM disabled with both requested angles at 90 degrees.
Set the desired positions before enabling. `Servo_Disable()` stops both outputs
but retains requested positions; it does not switch off servo power. Enabling
again uses the retained angles. A failed HAL channel start stops both channels
and leaves `Servo_GetData().enabled` false.

```c
/* In application code, after successful initialization: */
Switches_LED(true);
Switches_Buzzer(false);

Servo_SetAngles(0.0f, 90.0f);
Servo_Enable();

/* Later, without blocking the main loop: */
Servo1_SetAngle(45.0f);
Servo2_SetAngle(180.0f);
```

Pulse width is `500 + angle * 1900 / 180` microseconds, rounded to the nearest
counter tick. Thus 0/90/180 degrees map to 500/1450/2400 microseconds. Finite
out-of-range values are clamped. NaN/infinity are ignored; the paired setter
rejects both values if either is invalid. GetData reports requested angles, not
position feedback. Changes use compare preloads and take effect together on the
next timer update (up to 20 ms), without resetting a running PWM frame.

Use these APIs from one main-loop caller, not concurrently from interrupts.
No delays, DMA or timer interrupts are used. No further IOC changes are needed
for the current clock settings. Reinitialize after changing the timer clock.

Host regression test (MinGW GCC, from project root):

```powershell
gcc -std=c11 -Wall -Wextra -Werror -ITests/host/actuators -ICore/Inc Tests/host/actuators/test_actuators.c Core/Src/drivers/Servo.c Core/Src/drivers/Switches.c -o build/driver-tools/test_actuators.exe
./build/driver-tools/test_actuators.exe
```

Hardware verification remains: measure PB6/PB7 periods of 20 ms and pulses of
500, 1450 and 2400 microseconds, and check switch GPIOs start LOW. No board was
flashed or attached actuators operated during implementation.
