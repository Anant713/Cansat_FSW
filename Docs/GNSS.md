# Quectel L89HA-S90 GNSS over USART6

The active driver targets the L89HA-S90 on the 7Semi CM-10119 breakout,
firmware L89HANR01A07S, as identified by the supplied configuration record.
The requested profile is 115200 baud, 10 Hz, GPS + Galileo + NavIC, with
GNGGA and GNVTG output. The record's earlier GGA/RMC profile is not the target.
Initialization explicitly selects GGA and VTG and disables the other documented
standard NMEA outputs, then checks the resulting configuration.
GNRMC (not GNRMS) is the RMC sentence name; it is ignored by this driver.

## Files and integration

- Active driver: Core/Src/drivers/GNSS.c and Core/Inc/drivers/GNSS.h.
- Hardware test: Tests/test_gnss.c, preset TestGNSS.
- Unused u-blox NEO-6M source, header, tests and documentation: Legacy/NEO6M.
- The old GNSS_Nav and GNSS_SatInView presets are removed. Their source is
  archived and is not built into flight firmware or the Quectel test.
- SensorAcquisition uses GNSS_EXPECTED_BAUD (115200). Generated main.c and
  Documents.ioc also set USART6 to 115200. No separate baud switch is performed.
- PC6 is MCU TX to module RX; PC7 is MCU RX from module TX; common ground.
  Both UART wires are required for queries. There is no UART clock wire.

GNSS_Init(&huart6, GNSS_EXPECTED_BAUD) starts reception and schedules sentence
configuration followed by checks.
A true return means the MCU receiver started, not that the module's settings
have already been verified. Call GNSS_Process frequently from main context.
GNSS_GetDiagnostics exposes the eventual result. Flight acquisition currently
calls Process on its nominal 10 ms acquisition schedule. Delays from other
sensors and storage still count against the queue's available capacity.

## Startup sentence configuration and verification

Initialization first sends PAIR062,id,rate for documented IDs 0 through 8.
GGA (0) and VTG (5) get rate 1 (every navigation update); GLL, GSA, GSV,
RMC, ZDA, GRS and GST get rate 0 (disabled). Each command must receive a
successful acknowledgement before the next command is sent. The command
timeout is 2.5 seconds. This sequence is asynchronous and requires GNSS_Process.

The driver then sends these query bodies, with computed NMEA checksums and CRLF:

1. PAIR865,0,0: check configured UART baud is 115200.
2. PAIR051: check navigation interval is 100 ms.
3. PAIR063,-1: gather rates for IDs 0 through 9, matching the record's response.
   GGA (0) and VTG (5) must have rate 1; every other listed ID must have rate 0.
4. PAIR067: check GPS/Galileo/NavIC enabled and GLONASS/BDS disabled. QZSS may
   be enabled because the protocol documents it as a firmware default.

The driver requires successful ACK and query data, not ACK alone. Timeout is
2.5 seconds per query. Once settings match, it allows a further 3 seconds to
observe GNGGA and GNVTG and five consecutive 100 ms GGA UTC increments.
The UTC check measures navigation timestamps, not UART interrupt timing. If
UTC is unavailable before acquisition, query flags may pass but verification
can time out at step 4. Use verify after timestamps become available.

Baud, navigation rate and constellation settings are only checked. No
stop/start engine, reboot or flash-save commands are sent. The sentence mask
is applied on every initialization, so it does not depend on being saved in
the module. The firmware-specific ID 9 is checked as disabled in the readback;
only documented IDs 0 through 8 are written.

Diagnostics configState (historical name retained for compatibility):

| Value | Meaning |
| --- | --- |
| 0 | Idle |
| 1 | Sentence configuration or verification in progress |
| 2 | Query settings and observed output verified |
| 3 | Command rejected by module |
| 4 | Missing ACK, response, or output-rate evidence timed out |
| 5 | UART/transmit/stream error interrupted verification |
| 6 | Reserved old u-blox baud-unconfirmed status; not generated here |
| 7 | A returned setting differs from the expected profile |

configStep 10 through 18 means setting sentence IDs 0 through 8.
configStep 0/1/2/3 means baud/rate/sentence/constellation query; 4 means output
observation. Individual flags and returned values distinguish mismatch from
no reply. constellationMask bits 0..5 are GPS, GLONASS, Galileo, BDS, QZSS,
NavIC. expected mask is 0x25 or 0x35. messageReplyMask=0x3FF means all ten
message-rate replies arrived. The parser never treats other talkers' GGA/VTG
as corruption, but the expected multi-constellation GN talker must be observed
for verification to complete. GN alone does not prove which constellations
are enabled; that is checked by PAIR067.

GNSS_VerifyConfiguration() explicitly retries read-only checks after a failure;
it does not resend the sentence-setting commands. Reinitialize to apply the
sentence mask again.
GNSS_Configure(100,115200) remains only as a compatibility alias for this
read-only recheck. Other values are rejected; it no longer sets module values.
A configuration result is a startup snapshot, not continuous monitoring.

## Data and validity

The standard GNSSData layout and units remain unchanged: GGA gives UTC time
of day, latitude/longitude, hemisphere, MSL altitude, used-satellite count,
fix quality and HDOP; VTG gives ground speed in m/s and true course in degrees.
The parser accepts GN and GP talkers and preserves empty fields. All sentences
are checksum checked. Unknown NMEA and proprietary antenna-status sentences
are ignored. Receiver-specific UBX NAV fields do not exist in this profile;
they remain available only in the archived u-blox version.

No extra data fields are required for GGA + VTG. Configuration/readback fields
are in GNSSDiagnostics instead. GGA and VTG carry no calendar date or NED
velocity/accuracy estimates, so these cannot be recovered by renaming fields.
If date is later required, add RMC or ZDA and explicit date fields then.

GNSS_FRESH_MS is 500 ms (five nominal 10 Hz periods), replacing the previous
3-second window. A last value may remain in memory after it becomes stale;
check valid/fixValid/altitudeValid/hdopValid/speedValid/courseValid before use.
GGA and VTG freshness are tracked separately. VTG has no timestamp, so data is
latest-message based and is not an exact same-epoch fused solution.
DMA loss or UART error invalidates the stream. TestGNSS restarts RX after an
error; use verify to repeat configuration checks. The flight layer currently
does not automatically restart a failed GNSS receiver.

## DMA capacity at 115200 baud and 10 Hz

Existing hardware configuration is suitable: USART6 RX uses DMA2 Stream1,
channel 5, circular mode, byte alignment, memory increment, high DMA priority.
USART6 and DMA2 Stream1 IRQs both use preemption priority 1. Keep half-transfer,
transfer-complete, and UART IDLE events enabled. Buffers remain in normal SRAM,
not CCM. There is no need to align sentences with DMA boundaries.

At 115200 baud, 8N1 transmits at most 11520 bytes/s:

| Buffer/service condition | Time at saturated wire rate |
| --- | --- |
| 512-byte DMA buffer | 44.4 ms |
| 256-byte half buffer | 22.2 ms |
| 4096-byte software queue | 355.6 ms |

IRQ service must remain well below 22.2 ms with margin, even though average
traffic is lower. GNSS_Process consumes at most 1024 bytes per call. For an
illustrative 80-byte GGA + 50-byte VTG pair at 10 Hz, traffic is 1300 bytes/s
(~11.3% UART occupancy), and 4096 bytes represent about 3.15 seconds of that
traffic before considering existing backlog, query bursts or other messages.
These are sizing calculations, not measured lossless guarantees. The software
queue stores arrival ticks as well as bytes and uses about 20 KiB RAM.

No buffer-size increase is needed for the requested two-sentence stream.
Keep parsing frequently, especially around synchronous SD writes. Measure
queueDrops, queuedBytes, uartErrors and streamResets with actual logging.
Flash operations or debugger pauses that prevent servicing DMA can also lose
data. Increasing the software queue cannot recover overwritten DMA bytes.

## Test procedure

Select TestGNSS, build and flash build/TestGNSS/test_gnss.elf. Console USART3
is 115200 8N1, with Newline or CRLF. Commands: status, stream on, stream off,
verify, help. Debugger snapshots: gnss_test_data and gnss_test_diagnostics.
The test prints query readback, verification flags, message mask and observed
UTC interval. config=2 proves the checks completed; a position fix is separate.
A healthy no-fix receiver may still pass verification if UTC is available.

Host checks: powershell -ExecutionPolicy Bypass -File
Tests/host/run_gnss_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe

Sources: supplied L89HA-S90_Firmware_Configuration_Record.docx (29 October
2026), and Quectel L89 R2.0 GNSS Protocol Specification V1.0:
https://quectel.com/content/uploads/2024/02/Quectel_L89_R2.0_GNSS_Protocol_Specification_V1.0.pdf
