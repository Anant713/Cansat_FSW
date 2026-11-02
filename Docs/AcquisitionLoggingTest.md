# Combined sensor, logging and telemetry tests

Choose a configure/build preset:

| Preset | Tasks | ELF |
| --- | --- | --- |
| TestAcquisitionLogging | Sensors + CSV logging | build/TestAcquisitionLogging/test_acquisition_logging.elf |
| TestAcquisitionLoggingTelemetry | Sensors + CSV logging + XBee telemetry | build/TestAcquisitionLoggingTelemetry/test_acquisition_logging_telemetry.elf |

For example:

```powershell
cmake --preset TestAcquisitionLoggingTelemetry
cmake --build --preset TestAcquisitionLoggingTelemetry
```

Flash the matching ELF and open USART3 at 115200 baud, 8N1. Preserve sector 11:
both variants calibrate ground pressure and gyro during LAUNCH_PAD initialization
and save the pair to flash before normal tasks start. Keep stationary until
`Calibration saved/restored` appears. Airborne/touchdown boots retain the saved
reference instead of recalibrating. TestSensors uses the same startup policy.
The telemetry variant uses UART4/XBee and enables transmission automatically.
Both tests turn the LED on and enable both servos at 0 degrees before sensor/SD
startup. PWM remains enabled to hold that angle. No mission transitions run.
An empty mission journal
is seeded with LAUNCH_PAD before saving calibration; existing mission state is preserved.

Both tests share the acquisition/logging harness; the telemetry entry point enables
its additional compile-time task. CMake links only one entry point. The shared
periods remain ACQUISITION_PERIOD_MS (10 ms), LOGGING_PERIOD_MS (1000 ms), and
COMMUNICATION_TELEMETRY_PERIOD_MS (500 ms). There is no catch-up burst after a slow
operation. The telemetry variant services XBee each loop and sends before logging
when both are due, so the accepted packet count appears in the row.

## Simple timing report

Every five seconds (TIMING_REPORT_PERIOD_MS), the telemetry variant prints only:

```text
Avg sensor acquisition: ... us
Avg logging: ... us
Avg telemetry: ... us
Avg main loop (task sum): ... us
```

The acquisition/logging variant omits the telemetry line and sums only those two
task averages. Values are cumulative averages per task invocation. Before any
invocations, an average is displayed as zero. All durations are in microseconds.

The requested main-loop figure is the sum of task averages, not the measured mean
of every fast idle loop pass. Tasks run at different frequencies. Console output,
continuous XBee receive/service work, and scheduling overhead are excluded from
that sum. Initialization durations, minima/maxima, timestamps, counts and CPU-load
estimates are no longer printed. Detailed counters remain available in the debugger.

Acquisition measures the complete sensor update. Logging measures CSV formatting,
append-open, write and flushing close. Telemetry measures formatting and queuing
Communication_SendTelemetry, including failed/busy attempts. UART TX DMA finishes
asynchronously; the reported time is not total airtime or an RF delivery latency.
Accepted and failed send counts remain in test_telemetry_successes/failures.

DWT CYCCNT measures execution with interrupt time included. Very long intervals
spanning a full counter wrap (~25.6 s at 168 MHz) are rejected. Avoid debugger
halts and clock changes during measurement. Host timings are not board timings.

## CSV and console

Logging follows test_sd_text.c: create/write header/close, then open-append/write
one CRLF row/close for each log. The agreed 44 fields, scales and sentinels stay
unchanged. Missing estimates/actuator values retain initialized defaults and
unavailable sentinels. ALTITUDE comes from bmp.relativeHeightM with heightValid;
GNSS ALTITUDE comes from gnss.altitude. The driver height is relative to its most
recent initialization, unlike mission navigation's persisted reference.

The startup prints the selected LOGnnnn.CSV filename and command help. Startup
failures are reported without extra timing details. Commands:

- status: print the same simple averages.
- stream off / stream on: disable/enable periodic reports.
- stop: stop scheduled tasks and close logging; reset to start a new run/file.

An already queued UART transmission may finish after stop. XBee service remains
active. SD failure disables further logging; sensor acquisition and telemetry
continue. SD and sensor initialization happen before XBee reception starts.

Tests/host/run_acquisition_logging_tests.ps1 exercises both variants using mock
execution durations. Existing communication/logging suites cover binary fields,
CSV equality and SD failures. Validate actual files, RF reception and durations
on the board.
# Startup calibration in both variants

The shared harness enables startup calibration for both variants and its drone wrapper.
After sensor initialization, it calls `SensorAcquisition_CalibrateAtStartup()`
which uses the existing BMP driver's ground-calibration routine: settle for 1 s,
then average 16 real pressure samples 110 ms apart (about 2.7 s total).
Keep the board stationary at the starting location until initialization completes.
The measured P0 is printed once. Subsequent driver reads calculate
`44330 * (1 - (pressurePa / P0)^0.1903)` in `bmp.relativeHeightM`, used by both
telemetry ALTITUDE and SD CSV. Height is now relative to the startup location,
approximately zero at startup, rather than using fixed sea-level pressure.
On LAUNCH_PAD, startup also collects stationary gyro samples and saves ground
pressure plus all three gyro biases together in flash. Airborne and touchdown
boots retain the restored pair instead of recalibrating. Keep stationary until
startup completes; failures are reported on the console.
Valid negative barometric heights encode as zero in telemetry and CSV logging.
The original signed height remains available for flight processing. Invalid or
non-finite heights still encode as the unavailable sentinel.
