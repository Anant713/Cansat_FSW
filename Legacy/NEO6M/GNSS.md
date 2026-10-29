# NEO-6M GNSS over USART6 asynchronous UART

All three standalone GNSS tests now request 5 Hz (200 ms) at 38400 baud.
They start at 9600 baud, then switch the module and MCU together. Console reports
remain at 1 Hz; parsing runs each loop. If the module retains 38400 baud across
an MCU reset, change the test's initial baud to 38400 (the literal in
GNSS_SatInView's GNSS_Init call, or CURRENT_BAUD/GNSS_TEST_CURRENT_BAUD).
Use config=2 to confirm configuration completed. At 38400 8N1, a DMA half-buffer
fills in about 67 ms and the software queue holds about 1.07 s at full wire rate.

## GGA + NAV-VELNED + NAV-TIMEUTC test

Select CMake preset `GNSS_Nav`, build, and flash
`build/GNSS_Nav/GNSS_Nav.elf`. Source: `Tests/GNSS_Nav.c`.
USART6 starts at 9600 baud and configures 38400 baud / 5 Hz by default; change
`CURRENT_BAUD`, `TARGET_BAUD`, `PERIOD_MS` in that test if needed.
CURRENT_BAUD must match the receiver before configuration starts.
Console is USART3 at 115200 8N1 with newline/CRLF. Commands:
`status`, `stream on`, `stream off`, `configure` (retry), `help`.
`config=2` indicates acknowledged configuration; it does not mean a fix.
Both GNSS TX and RX wires are required. Settings are volatile, not saved to flash.

This is a compile-time variant of the existing driver, not a second live UART
instance. CMake defines `GNSS_ENABLE_NAV=1` for the whole target, including all
users of GNSS.h. GNSS_Init/Process/GetData and the HAL callbacks are unchanged.
The same 512-byte circular DMA buffer and 4096-byte queue carry both protocols.
The binary parser stores up to 36 payload bytes and validates CK_A and CK_B
before decoding NAV class 01, IDs 12 (VELNED) and 21 (TIMEUTC). Embedded binary
commas, dollar signs and newlines are not interpreted as NMEA. Split packets
and DMA wraps are supported. Unknown UBX payloads are consumed without decoding.

Configuration uses CFG-MSG to enable GGA, NAV-VELNED and NAV-TIMEUTC each
solution, and disable the other standard NMEA sentences (including VTG/GSV).
Normal/GSV profiles now explicitly disable these two NAV messages when they
configure the receiver, allowing a return from this test without power cycling.
This does not erase arbitrary unrelated receiver settings made by other tools.

Debugger snapshots: `gnss_nav_data`, `gnss_nav_diagnostics`.
The green requested fields map as follows:

| Field | GNSSData member | Unit/source |
| --- | --- | --- |
| Position | latitude, longitude | signed degrees, GGA |
| Hemisphere | latitudeHemisphere, longitudeHemisphere | N/S/E/W |
| MSL altitude | altitude / altitudeValid | metres, GGA |
| Satellites used, fix quality | satellites, fixQuality | GGA |
| HDOP | hdop / hdopValid | dimensionless, GGA |
| N/E/D velocity | nav.northVelocity, nav.eastVelocity, nav.downVelocity | m/s; Down positive |
| Speed accuracy | nav.speedAccuracy | m/s estimate |
| Course accuracy | nav.courseAccuracy | degrees estimate |
| UTC calendar | nav.year/month/day/hour/minute/second | NAV-TIMEUTC |
| Time-valid flags | nav.timeValidityFlags | bit 0 TOW, bit 1 week, bit 2 UTC |

Also decoded: nav.speed3D, nav.groundSpeed (m/s), nav.course (degrees),
nav.timeAccuracyNs and signed nav.nanosecondCorrection. The latter is added
to the calendar second; it may be negative. The test prints it separately
rather than incorrectly presenting it as a positive fractional second.
Vertical velocity positive upward can be derived as `-nav.downVelocity`.
Existing top-level speed/course/vtgValid remain VTG fields; use nav fields in
this profile. Existing utcTime remains GGA time-of-day, separate from TIMEUTC.

Validity matters: nav.velnedFresh/timeutcFresh each expire after 3 seconds
using DMA arrival time. TIMEUTC date/time additionally requires all three
validity bits and a sensible calendar. GNSS_GetData's return value still means
fresh GGA only; inspect NAV flags even if it returns false. VELNED has no own
fix-valid flag: nav.velocityValid requires a fresh VELNED and fresh valid GGA.
That is a usability heuristic, not proof of a same-epoch or valid 3D fix; GGA
cannot distinguish 2D from 3D fixes. For stricter flight use, a future NAV-SOL
or NAV-STATUS message would provide receiver fix flags. The test prints fresh
velocity without a fix as UNVALIDATED, not a valid motion measurement.
nav.navEpochMatched only compares the two fresh UBX iTOW values; it does not
assert GGA is from that same epoch. All are latest-message snapshots.

Using the supplied 79-byte GGA example, each cycle is 79 + 44 + 28 = 151
bytes (GGA length varies). At 9600 8N1 this is approximately 16% wire occupancy
at 1 Hz, 31% at 2 Hz and 79% at 5 Hz, before configuration traffic. Default 5 Hz at 38400 baud uses about 20% wire occupancy. No DMA/queue size or CubeMX change is needed; process
regularly and keep interrupts serviced as described below. This is a bandwidth
estimate, not a measured guarantee of lossless operation.

Protocol reference: [u-blox 6 Receiver Description, sections 35.13 and 35.15](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf).
Host tests: run `Tests/host/run_gnss_tests.ps1 -Nav` with the host GCC compiler.

## Satellites-in-view diagnostic firmware

Build/flash preset `GNSS_SatInView` (`Tests/GNSS_SatInView.c`). This is a
test-only driver variant compiled with `GNSS_ENABLE_GSV=1`, not a second
receiver competing for USART6. It reuses the existing DMA/queue and enables
GGA + VTG + GPGSV at 5 Hz, 38400 baud (initial connection at 9600). Normal builds still configure GGA/VTG only.
Connect both GNSS TX and RX so the receiver can acknowledge configuration.
The module must initially be at 9600 baud; configuration is volatile.

Console: USART3, 115200 8N1, newline/CRLF. Commands: `status`, `stream on`,
`stream off`, `configure` (retry), `help`. `config=2` means configuration done.
Watch `gnss_view_data` and `gnss_view_diagnostics` in the debugger.
The output shows GSV satellites in view versus GGA satellites used in the fix.
The count is available without a fix. It is the total in each GPGSV header,
not the sum of multipart reports, and does not mean all listed satellites have
a tracked signal. This test returns the count, not a per-satellite PRN/SNR list.
Data older than 3 seconds or invalidated by RX loss is shown as N/A.

No CubeMX or buffer-size change is required for this 5 Hz test. At 38400 8N1,
512 DMA bytes span about 133 ms, and HT/TC servicing is needed within about
67 ms with margin. The 4096-byte queue spans about 1.07 seconds at saturated
wire rate. Keep calling Process each loop; DMA does not need to hold a whole
multipart GSV report. Higher output rates may exceed UART bandwidth even when
RAM buffers are large enough; monitor drops/checksum/UART errors on hardware.

Host regression: `powershell -ExecutionPolicy Bypass -File
Tests/host/run_gnss_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe -Gsv`.

`USART6` is the peripheral name. In **Asynchronous** mode it is a normal UART:
no SCLK/CK pin is used. The driver uses USART6 on PC6/PC7.

| Breakout pin | STM32 connection |
| --- | --- |
| TX | PC7 / USART6_RX |
| RX | PC6 / USART6_TX |
| GND | Common ground |
| VCC | Supply appropriate for the actual breakout board |

Both TX and RX connections are needed for configuration; receiving NMEA alone
needs only module TX, MCU RX, power, and ground. No PPS or clock wire is required.

## Circular DMA decision

Circular RX DMA is appropriate for the continuously arriving byte stream. It
does not require the main loop to be free when a byte arrives, and it does not
need restarting at each sentence or buffer boundary. An interrupt-per-byte
receiver would also be feasible at these rates, but requires more interrupts.
The circular choice is useful here because sensor reads and SD writes can delay
the application loop.

The implementation is:

```
USART6 RX -> 512-byte circular DMA buffer
         -> HT / TC / IDLE callback copies new bytes and arrival ticks
         -> 4096-byte software queue
         -> GNSS_Process() in main loop: NMEA/UBX stream parsing
         -> GNSS_GetData(): latest values with freshness/validity flags
```

Copying in the callback keeps DMA reception independent of the main loop's
parsing schedule. The callback does not parse sentences, format text, wait for
data, or restart DMA. It samples the current DMA counter, so duplicate or
out-of-order HT/TC/IDLE notifications do not duplicate queued bytes. UART and DMA
interrupts have the same preemption priority so these callbacks cannot nest.
The queue uses about 20 KiB, including per-byte arrival ticks, plus the 512-byte
DMA buffer. Those ticks prevent old queued data becoming falsely fresh when
the main loop eventually parses it. Buffers remain in DMA-accessible SRAM,
not CCM RAM. STM32F405 has no data cache requiring buffer cache maintenance.

Two different servicing deadlines matter:

| Baud, 8N1 | Maximum wire throughput | Half of DMA buffer fills in | 4096 queued bytes at saturated wire rate |
| --- | --- | --- | --- |
| 9600 | 960 bytes/s | 267 ms | 4.27 s |
| 38400 | 3840 bytes/s | 66.7 ms | 1.07 s |
| 115200 | 11520 bytes/s | 22.2 ms | 0.356 s |

These are theoretical wire-time calculations, not measured board timings. The
actual GGA+VTG traffic at 1–5 Hz is usually much smaller than saturated UART
capacity. Size the queue against the **longest** main-loop delay, including SD
card stalls, and measure that delay on hardware. Call `GNSS_Process()` each
normal loop pass; it consumes at most 1024 bytes per call and never waits for
new bytes. If backlog persists, service it more frequently.

The DMA/USART interrupts must remain serviceable within a half-buffer interval,
with margin for interrupt latency and copying. A software queue does not help
if interrupts are disabled long enough for DMA to overwrite its own buffer.
The DMA counter/interrupt flags do not count arbitrarily many missed wraps;
that condition cannot be reconstructed reliably. Do not claim lossless capture
through long interrupt masking. The current SD driver requires interrupts on.

If the **software queue** fills, new bytes are dropped, `queueDrops` increases,
and the next process pass discards the ambiguous backlog/partial frame and
invalidates the solution. UART/DMA errors similarly invalidate data and appear
in diagnostics; call `GNSS_StartReception()` from main context to recover.
Long main-loop stalls can therefore lose data, but they do not become an
unreported valid fix through the normal queue-overflow path.

## Sentence boundaries and incomplete packets

DMA size and sentence length are unrelated. The parser retains its state across
process calls and DMA wraps. For example, one chunk can end with
`$GPGGA,123519,4807.` and the next can begin with `038,N,...*HH\r\n`.
No parsing result is published until the whole sentence arrives and its XOR
checksum is correct. A new `$` resynchronizes an incomplete NMEA line. Noise,
overlong lines, bad numeric fields, and bad checksums are rejected; a partial
frame with a receive gap over 1.5 seconds is discarded.

UBX binary replies share the UART with NMEA. The parser recognizes their sync,
length, and checksum, and does not interpret `$` or newlines inside an accepted
UBX frame as NMEA. It retains only small configuration reply payloads. Unsupported
NMEA sentences are ignored even before receiver filtering is configured.

GGA and VTG fields are published independently. VTG has **no UTC timestamp**;
these two sentence types cannot prove that every field is from exactly the same
navigation epoch. Separate receive timestamps and validity flags expose their
age. Do not present the combined struct as an atomic, time-synchronized fix.

## Data provided

| Field | Meaning |
| --- | --- |
| `latitude`, `longitude` | Signed decimal degrees; south/west are negative |
| `utcTime`, `utcMilliseconds` | GGA UTC time-of-day as HHMMSS plus milliseconds; **no calendar date** |
| `altitude` | GGA altitude above mean sea level, metres |
| `speed` | VTG speed over ground, metres/second |
| `course` | VTG course over ground, degrees clockwise from true north |
| `satellites` | GGA satellites used in the solution, not satellites in view |
| `fixQuality` | Raw GGA quality indicator |
| `ggaReceivedMs`, `vtgReceivedMs` | Host byte-capture times, wrapping with HAL tick |

`valid` means a well-formed GGA received less than three seconds ago while
reception is healthy. A valid **no-fix** message still returns true from
`GNSS_GetData()`. `fixValid` additionally requires quality 1 (GPS) or 2 (DGPS);
estimated/dead-reckoning quality 6 is not accepted as a satellite fix. Check
`altitudeValid`, `utcValid`, `vtgValid`, `speedValid`, and `courseValid` before using
those fields. Stale numeric values can remain in the snapshot, with flags false.
There is no requirement to invent a fix when satellites are visible but geometry
or reception is inadequate.

Course is the direction of motion, not roll/pitch/yaw attitude, and is not a
reliable body heading while stationary. GGA supplies **altitude**, not attitude.
GGA has no date; retain your RTC date for timestamped CSV or enable an additional
date-bearing sentence/protocol later. Exact PPS timing is not available through
the breakout's four exposed pins.

Your supplied Arduino example also reads GSV for satellites in view and probes
two software UART pins. This STM32 test uses the known USART6 connection and
shows GGA satellites used; GSV is disabled as requested. The attached C++ has
invalid literals/comparisons in `fieldInt`; it was treated as a behavior example,
not imported into this C/HAL project.

## Application API

```c
#include "drivers/GNSS.h"
extern UART_HandleTypeDef huart6;

/* Once after CubeMX UART and DMA initialization. Use the CURRENT module baud. */
bool gnss_ok = GNSS_Init(&huart6, 9600);

/* Each main-loop pass: */
GNSS_Process();

/* When the acquisition layer wants a snapshot: */
GNSSData gps;
if (GNSS_GetData(&gps)) {
    /* gps.satellites and gps.fixQuality are from a fresh GGA. */
    if (gps.fixValid) { /* use latitude/longitude */ }
    if (gps.speedValid) { /* use speed */ }
} else {
    /* No fresh GGA, RX error, or a detected queue discontinuity. */
}
```

All driver APIs except callback entry points are for main context. One driver
instance owns the selected UART. Start is automatic in `GNSS_Init()`; do not
restart reception every loop. `GNSS_IsDataReady()` means a fresh GGA exists, not
a consumable event. No dynamic allocation is used. `Init`, explicit stop/restart,
and the brief local baud reinitialization touch HAL hardware setup; routine
reception, parsing, and command/ACK waiting do not use blocking receive calls,
`HAL_Delay`, or polling for a future byte.

## Configurable baud, rate, and output sentences

```c
/* Start once, then continue calling GNSS_Process(). */
bool started = GNSS_Configure(500, 38400); /* 2 Hz, receiver+MCU target baud */

GNSSDiagnostics status;
GNSS_GetDiagnostics(&status);
/* status.configState: BUSY, DONE, NAK, TIMEOUT, IO_ERROR, BAUD_UNCONFIRMED */
```

`started=true` means the operation was accepted locally, not that the receiver
has acknowledged it. The driver serializes UBX commands over interrupt-driven
TX, with a persistent TX buffer and checksum-checked ACK/NAK replies. It enables
GGA/VTG once per navigation solution, disables the other standard NMEA message
IDs 0x00–0x0A, and disables NMEA TXT via CFG-INF. Other previously enabled binary
UBX/proprietary messages are not exhaustively reconfigured; they are ignored by
the navigation parser. Settings take effect progressively; failure does not
roll back commands already acknowledged.

| `periodMs` | Navigation rate | UBX-CFG-RATE settings |
| --- | --- | --- |
| 1000 | 1 Hz | measRate=1000, navRate=1, timeRef=UTC |
| 500 | 2 Hz | measRate=500, navRate=1, timeRef=UTC |
| 200 | 5 Hz | measRate=200, navRate=1, timeRef=UTC |

The supplied NEO-6 datasheet specifies up to 5 Hz for NEO-6M. This is a receiver
configuration over RX/TX, not a clock or GPIO setting. Rates depend on the actual
module/firmware; ACKs confirm command acceptance, while measured UTC increments
and byte/sentence counts confirm real output. Defaults are normally 9600 8N1,
1 Hz, and multiple NMEA sentences. Backup power may retain earlier settings.

Supported baud rates are 9600, 19200, 38400, 57600, and 115200. `GNSS_Init()`
sets **only the STM32** to the supplied current baud. `GNSS_Configure()` optionally
sends CFG-PRT at the old baud, waits for physical TX completion and a 100 ms
processing interval, switches the MCU, restarts RX, and polls CFG-PRT at the new
baud. It reports success only after matching port/baud/protocol readback. This
transition necessarily interrupts capture briefly. A module ACK may occur at
either baud, so an old-baud ACK alone is not treated as proof of success.

If port verification fails, `BAUD_UNCONFIRMED` means the receiver may be at
either baud. Inspect `status.baudRate` for the current MCU setting and recover by
initializing at the known/observed module baud. There is no automatic baud scan.
Poll `GNSS_Process()` frequently during configuration (preferably every 100 ms
or faster); each command has a 2.5-second response deadline. Configuration is a
startup/maintenance operation, not something to request for each logged sample.

9600 with GGA+VTG is adequate for typical 1–2 Hz output. For 5 Hz, 38400 gives
more transmission headroom while retaining a 66.7 ms half-buffer IRQ deadline.
Actual sentence lengths and other enabled messages determine utilization.
No CFG-CFG save is sent: these settings are volatile; they are not deliberately
written to module flash, EEPROM, or backup storage. Reapply after a cold start,
and account for retained settings when only the MCU resets.

## CubeMX settings already applied

`Documents.ioc` and the current generated C files now contain:

- USART6 Asynchronous, PC6 TX / PC7 RX, 9600 baud, 8N1, no flow control.
- USART6_RX DMA2 Stream1 / Channel5, peripheral-to-memory, **circular**,
  peripheral increment off, memory increment on, byte/byte alignment, high DMA
  priority, FIFO disabled.
- DMA2 clock enabled before UART initialization.
- DMA2_Stream1 and USART6 global interrupts enabled, preemption priority 1 and
  subpriority 0 for both. HT, TC, and IDLE events remain enabled by HAL.
- DMA link in the UART MSP function, plus DMA and UART IRQ handlers forwarding
  to HAL. No TX DMA is needed; configuration uses TX interrupts.

The changes can be reviewed/regenerated in CubeMX. Keep user files
`GNSS.c`/`GNSS_HAL.c` in CMake and preserve the asynchronous mode. Do not disable
half-transfer callbacks as is sometimes done in single-buffer receive examples.
`GNSS_HAL.c` contains the global HAL UART callback dispatcher; add future UART
clients there instead of defining duplicate callback functions elsewhere.

## Functional test

```powershell
cmake --preset TestGNSS
cmake --build --preset TestGNSS
```

Flash `build/TestGNSS/test_gnss.elf`. See [`Tests/test_gnss.c`](../Tests/test_gnss.c):
the test starts at 9600, waits for valid NMEA, then requests GGA+VTG at 5 Hz/38400.
Adjust its `GNSS_TEST_CURRENT_BAUD`, `GNSS_TEST_TARGET_BAUD`, and
`GNSS_TEST_PERIOD_MS` constants for another setup. It does not initialize servos
or switches through their drivers; generated main still initializes peripherals.

Inspect these debugger variables once per second:

- `gnss_test_state`: waiting for bytes, waiting for GGA, NO FIX, FIX, STALE, or RX error.
- `gnss_test_data`: satellites used, fix quality, UTC, position, altitude, speed, course, and flags.
- `gnss_test_diagnostics`: byte/sentence counts, rejected data, queue drops, UART errors, configuration result.
- `gnss_test_reports`: confirms the test application is running.

A well-formed no-fix message proves UART/parser operation, not successful
positioning. Test outdoors with the antenna exposed to the sky; cold acquisition
can take time. Disconnect module TX after a fix and check that the fix becomes
stale after three seconds. Test through expected SD stalls and inspect queue
drops. This separate target does not yet run combined SD logging.

The shared test console prints UTC, latitude/longitude, speed and course on
USART3 at 115200 8N1 by default. Invalid/stale fields print `N/A`. Connect PC10
to adapter RX, PC11 to adapter TX, and common ground (or use the Nucleo ST-LINK
virtual COM interface). Commands: `status`, `stream on`, `stream off`, `help`.
Console code and its USART3 IRQ are linked only into tests; see
[console usage and bounds](../Tests/README.md#shared-test-console).

Host regressions:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_gnss_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe
```

They exercise every sentence split, DMA wrap and callback ordering, deferred
parsing, units/checksums/no-fix/staleness, queue overflow, UART recovery, and UBX
configuration success/failure/baud verification. Hardware tests are still needed
for wiring, actual receiver replies, interrupt latency and SD coexistence.

References: supplied `GNSS.cpp` and NEO-6 datasheet;
[u-blox 6 protocol manual](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)
(GGA/VTG, CFG-MSG, CFG-INF, CFG-RATE, CFG-PRT), the local STM32F4 HAL UART driver,
and [ST RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-407-415-417-437-455-469-application-note-stmicroelectronics.pdf)
(DMA2 request mapping and USART asynchronous operation).
