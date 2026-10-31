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
initialization restores existing gyro/ground calibration without resampling zero.
Without a saved calibration pair, relative altitude is unavailable; calibrate
using flight firmware first. The standalone TestSensors bench test explicitly
samples its own temporary ground zero and does not persist it.
The telemetry variant uses UART4/XBee and enables transmission automatically.
No mission transitions, actuator commands or mission-flash writes run in either test.

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
