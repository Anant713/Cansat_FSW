# SD CSV logging

`Logging_Write(&flight_cansat)` collects the current Cansat values and writes one
CSV row through the SDCard driver. Rows contain the same 44 fields, in the same
order, with the same scaling, rounding, validity checks, and reserved sentinels as
[binary XBee telemetry](Communication.md). Both use the shared `Telemetry` encoder.
Logging does not modify Cansat or increment the telemetry packet counter.

For example, acceleration stored as 72.25 m/s^2 becomes integer 7225: XBee sends
the two bytes `39 1C`, whereas the CSV contains the text `7225`. Signed fields
retain their sign. Invalid signed 16-bit measurements are written as `-32768`;
unsigned invalid measurements are `255`, `65535`, or `16777215`, according to
their width. Valid hemispheres are N/S/E/W; invalid hemispheres are `255`.
The team ID has no binary padding in the CSV. Rows end in CRLF, with no written
NUL terminator. The column definitions and scales are in Communication.md.

## File lifecycle

`Logging_Init()` mounts the card and creates the first unused filename from
`0:/LOG0000.CSV` through `0:/LOG9999.CSV`. Existing files are never overwritten.
Each new file starts with a 44-column header written with `SDCard_WriteString`,
then closed. Each log calls `SDCard_OpenAppend`, writes exactly one CRLF CSV row
with `SDCard_WriteString`, and calls `SDCard_CloseFile` to flush it, matching
`Tests/test_sd_text.c`. A 512-byte buffer holds each formatted data row.

`Logging_IsReady()` reports whether rows can be written; `Logging_GetFilename()`
exposes the selected filename. An open, write or close failure disables further logging.
There is no automatic retry or remount in the loop. `Logging_Stop()` closes the
file; after resolving the SD error, an explicit new initialization can start a
new file. Driver recovery may also require `SDCard_DeInit()`. A failed close can
be retried. Calling Init while the file remains open does not restart logging.
`SDCard_GetLastResult()` exposes the latest FatFs result (a cleanup close can
replace an earlier write error; a false Logging_Write still indicates failure).

## Superloop timing

FlightApp initializes logging before starting sensor/GNSS and XBee reception.
It services XBee reception on every pass and independently schedules:

- Telemetry attempts every 500 ms when `telemetry.enabled` is true.
- Logging every 1000 ms when logging is ready, independent of telemetry enable.

When both are due, transmission runs first. An accepted transmission updates the
packet count before that same snapshot is logged. Otherwise a row represents the
current Cansat snapshot and the last accepted packet count; it is not proof that
the row was transmitted or received on the ground. Failed/busy sends do not
increment the count or prevent logging.

Elapsed-time checks handle HAL tick rollover and do not produce catch-up bursts.
These are nominal 2 Hz/1 Hz schedules, not guaranteed minimum rates: the existing
sensor and SD open/write/close APIs block, so a slow operation can delay the next pass.
The state machine calls `Logging_Stop()` on touchdown, including recovered touchdown.

## Verification

Run `Tests/host/run_logging_tests.ps1`, `run_communication_tests.ps1`, and
`run_acquisition_tests.ps1` from PowerShell. Tests check all 44 CSV values against
decoded binary fields, signed values/sentinels, buffer bounds, headers, filename
collisions, SD failures, independent scheduling, tick rollover, and packet count
ordering. Debug and Release firmware builds include Telemetry and Logging.
Actual card timing, persisted contents, and power interruption require board tests.
