# Flight and subunit test firmware

Every firmware uses the generated `Core/Src/main.c` for HAL, clock, and peripheral
initialization. It calls `App_Init()` once, then calls `App_RunOnce()` repeatedly.
CMake selects exactly one source file implementing these hooks. Test files do not
define their own `main()` and do not run the flight application's loop.

The flight application runs sensor acquisition, communication, and logging; its other mission
operations remain a scaffold. See [acquisition behavior and host tests](../Docs/SensorAcquisition.md)
and [telemetry schema and communication tests](../Docs/Communication.md).
See [CSV logging and scheduling](../Docs/Logging.md); run its host regression tests
with `powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_logging_tests.ps1`.
The test template demonstrates the serial console but does not perform a device test.
Persistent mission transitions and the `TestCamera` ten-second recording test are
described in [StateMachine.md](../Docs/StateMachine.md). State-machine host tests
run with `Tests/host/run_state_tests.ps1`.

## Build an application

In VS Code, select the CMake configure preset and build its `Documents` target.
The internal target retains its CubeMX name; the ELF filename identifies the
selected application. Configure and build presets are paired:

| Preset | Selected source | Output |
| --- | --- | --- |
| Debug | `Core/Src/apps/FlightApp.c` | `build/Debug/flight.elf` |
| Release | `Core/Src/apps/FlightApp.c` | `build/Release/flight.elf` |
| TestTemplate | `Tests/test_template.c` | `build/TestTemplate/test_template.elf` |
| TestSD | `Tests/test_sd.c` | `build/TestSD/test_sd.elf` |
| TestSensors | `Tests/test_sensors.c` | `build/TestSensors/test_sensors.elf` |
| TestGNSS | `Tests/test_gnss.c` | `build/TestGNSS/test_gnss.elf` |
| TestXBee | `Tests/test_xbee.c` | `build/TestXBee/test_xbee.elf` |

`TestSD` creates a new file, writes and appends a binary pattern, remounts, and
verifies the contents. It prints PASS/FAIL, stage, FatFs result, and filename;
`status` repeats the result without rerunning the test. Debugger variables remain
available. It does not overwrite existing files.
See [SD integration and test instructions](../Docs/SDCard.md).

`TestSensors` polls BMP280, AHT20 (humidity only), BMI270, and IIS2MDC on I2C2.
It prints temperature/pressure, humidity, acceleration/angular rate, and magnetic
field once per second, with units, sample timestamps, initialization status and
counts. `status` prints on demand; `stream off`/`stream on` control periodic output.
`cal gyro` starts gyro calibration: keep the board stationary. Debugger variables
remain available. See [sensor APIs, calibration, and board checks](../Docs/Sensors.md).
Host tests are in `Tests/host/run_sensor_tests.ps1`.

`TestGNSS` receives NEO-6M GGA/VTG over USART6 circular RX DMA and reports fix/no-fix,
satellites used, freshness, queue errors, and configuration status in debugger
variables. USART3 output is enabled by default: UTC time, signed latitude/longitude,
speed in m/s, and course in degrees true print once per second. Invalid or stale
fields print `N/A`. Commands: `status`, `stream on`, `stream off`, `help`.
See [GNSS setup and APIs](../Docs/GNSS.md).
Host tests are in `Tests/host/run_gnss_tests.ps1`.

`TestXBee` transmits a fixed 153-byte message and lines entered on the USART3
serial monitor, and displays incoming XBee bytes there. XBee uses UART4 normal
TX DMA and circular RX DMA. See [wiring, test commands and driver APIs](../Docs/XBee.md).
Host tests are in `Tests/host/run_xbee_tests.ps1`.

The shared SD/FatFs sources are already registered in the top-level CMake target;
do not add those sources again to individual test applications. Host regression
tests are available in `Tests/host/run_sd_tests.ps1`.

From a terminal with the STM32 compiler, Ninja, and CMake available:

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
cmake --preset TestTemplate
cmake --build --preset TestTemplate
```

If your STM32 VS Code environment exposes `cube-cmake` instead of `cmake`, use
`cube-cmake` for these commands. Each preset uses a separate build directory.
Select the matching ELF when configuring flashing or debugging. The linker map
retains the generated name `Documents.map` inside each build directory.

## Add a subunit test

1. Copy `Tests/test_template.c` to, for example, `Tests/test_led.c`.
   Implement `App_Init()` and `App_RunOnce()` and include `App.h`. Initialize only
   the required drivers/modules. Keep the template's console initialization and
   input processing; use `Console_Write`/`Console_Printf` to report results.
2. Register the application in the top-level `CMakeLists.txt`: add `test_led` to
   the cache option's `STRINGS` list and error message, and insert a branch before
   the final `else()`:

   ```cmake
   elseif(FIRMWARE_APP STREQUAL "test_led")
       set(FIRMWARE_SOURCES
           Tests/test_led.c
           Core/Src/drivers/Switches.c
       )
   ```

   Implement the required `Switches` functions before this test calls them.
   List any additional dependencies explicitly; do not include `FlightApp.c`,
   `test_template.c`, or another test's hook implementation in the same build.
3. Add a configure preset to `CMakePresets.json`:

   ```json
   {
       "name": "TestLED",
       "inherits": "default",
       "cacheVariables": {
           "CMAKE_BUILD_TYPE": "Debug",
           "FIRMWARE_APP": "test_led"
       }
   }
   ```

   Add its matching build preset:

   ```json
   { "name": "TestLED", "configurePreset": "TestLED" }
   ```

4. Configure and build `TestLED`. Its output is `build/TestLED/test_led.elf`.
   Flash/debug that ELF to run the LED test on the STM32.

Repeat for `test_switches.c` or individual sensor tests, registering each source,
its driver/module dependencies, and its own preset. Estimator tests can supply
synthetic sensor inputs; acquisition tests can use physical sensor drivers.

Both flight and test builds initialize all peripherals configured by CubeMX.
Only the selected hook implementation starts device drivers and application
processing. Keep the main hook calls inside CubeMX `USER CODE` sections and keep
test registration in the top-level CMake file, outside generated build files.

## Shared test console

`Tests/Console.h` and `Tests/Console.c` are included only in test firmware by the
top-level CMake selection. No driver, flight application, generated main, or IOC
change is needed. Test builds also enable newlib-nano floating-point formatting
(`-u _printf_float`); flight builds do not inherit this option or console code.

Open the Nucleo ST-LINK virtual COM port at **115200 baud, 8N1, no flow control**.
Connect CanSat PC10 (USART3 TX) to the ST-LINK UART RX, PC11 (USART3 RX) to its TX,
and common ground. Use the Nucleo's external-target SWD/UART isolation setup;
SWD alone does not carry these console messages. No USB firmware runs on CanSat.
Send CR, LF, or CRLF when pressing Enter. Local echo can be enabled in the terminal.

```c
#include "Console.h"

/* App_Init(), after generated peripheral initialization: */
(void)Console_Init();
Console_Write("Test ready\r\n");

/* App_RunOnce(): */
Console_Process();
/* Use a timer or command to print; do not flood every loop iteration. */
Console_Printf("pressure=%.2f Pa count=%lu\r\n", (double)pressure, (unsigned long)count);
```

Call `Console_ReadLine(command, sizeof(command))` each pass with a buffer of
`CONSOLE_LINE_MAX + 1` bytes. Only execute a command on `CONSOLE_LINE_READY`.
`CONSOLE_NO_LINE` means no complete command yet. Negative results indicate loss,
invalid input, or oversize input; prompt for Enter followed by a resend. See the
template for an echo example. Use `Console_WriteBytes(bytes, length)` for binary
data: the XBee test uses it to preserve incoming NUL bytes, although terminals
will not visibly render every binary value.

The fixed storage is a 512-byte TX formatting buffer, 512-byte RX ring, and a
255-byte line buffer (254 text characters plus terminator). These buffers never
grow. There is no TX queue or helper-managed heap allocation. `Console_Printf`
uses the C library's `vsnprintf`; floating-point conversion may use library heap
internally, so this is not a guarantee of heap-free C-library formatting.
Standard `printf`/`scanf` are not retargeted; tests should use the console API.

TX is blocking with a 100 ms timeout per call; RX uses one-byte interrupts and
continues during TX. Maximum raw output is 512 bytes per call; formatted output
is at most 511 characters. Oversized formatted messages are rejected without
sending a truncated line. TX failures return false and can leave partial output.
Call only from the main loop with interrupts enabled, never from an ISR. For
timing/throughput measurements disable streaming and print results afterward.

Empty lines are ignored; backspace/delete edit the pending command. Overflow,
embedded NUL, or a UART error invalidates the affected command. Recovery discards
through a newline; after RX loss, send Enter before resending. Counters are visible
in `console_diagnostics`. Continuous input beyond processing capacity may be lost;
there is no hardware flow control. Commands can queue during startup but are
handled only once the test's main loop runs.

The helper owns the test-only `USART3_IRQHandler` and `HAL_UART_RxCpltCallback`.
Do not add duplicate handlers in a test or generate USART3's handler in CubeMX
without reconciling ownership. Other UART error/TX/DMA callbacks remain in the
existing driver dispatcher, with no console dependency.

## Console validation

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_console_tests.ps1
```

Host checks cover formatting and size limits, binary TX, TX timeout, RX during TX,
partial lines, CR/LF/CRLF, editing, ring wrap, overflow, invalid input, and UART
recovery. On hardware, flash each test ELF and verify the reported readings,
`status` commands, and XBee communication in both directions. Check GNSS outdoors,
then disconnect receiver TX to confirm stale fields become `N/A`. Use the template
to try long commands and recover with Enter; inspect console error counters.
