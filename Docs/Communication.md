# Binary communication and telemetry

The schema follows `Budgets - Telemetry Packet Structure.pdf` and the user's clarified
wire format: scale the measurement to an integer, then transmit that integer's
**binary bytes**, not its decimal digits. The current schema has 44 fields. Sensor values come from `Cansat`
in native physical units.

## Wire layout

Every packet is **151 bytes: 107 data bytes + 43 comma bytes + one CR byte**.
It contains 44 fixed-width fields. Integer fields use **little-endian** byte order
(least significant byte first); signed integers use two's complement. Values
occupy exactly 1, 2, or 3 bytes as specified. The team ID occupies 27 ASCII bytes,
zero-padded if shorter. There is no extra packet NUL terminator, LF, CSV header,
text number formatting, or transmitted C-struct padding.

Example: acceleration stored as 72.25 m/s^2 is multiplied by 100 to give 7225.
The int16 field is `0x1C39`, transmitted as `39 1C` (hex). Negative -72.25 becomes
-7225, transmitted as `C7 E3`. The next separator is one byte `2C`.
A 24-bit value such as 123456 is sent as `40 E2 01`.

**The receiver must parse fixed widths/offsets, not split on commas or CR.**
Payload bytes can themselves equal comma (`0x2C`), CR (`0x0D`), or NUL (`0x00`).
Read/assemble all 151 bytes across UART receive chunks and validate separators at
the specified positions; only byte 150 is the packet-ending CR. Do not call
`strlen`, `strtok`, line readers, or `XBee_SendString` on this packet. The PDF defines
no checksum/escaping or unique sync marker; offset checks alone cannot guarantee
resynchronization or detect every corrupted packet after byte loss. Those would
require a separately agreed framing extension.

## Source units, scaling, widths, and offsets

The units are documented in `Core/Inc/apps/Cansat.h` and the included driver data
structures. Acquisition retains native units. The shared Telemetry serializer performs
scaling for communication and CSV logging, rounds to nearest integer (halfway away from zero), checks range, and
packs bytes. For example, the magnetometer stores microtesla: converting to gauss
and then to 0.0001-gauss counts is microtesla /100 *10000 = microtesla *100.

Field numbers below are one-based; byte offsets are zero-based. Each component
has its own following comma except the final field, which is followed by CR.
Estimated angular velocity is body X/Y/Z in degrees/s; it is separate from Euler
angle derivatives. Quaternion order is w,x,y,z, dimensionless. Communication does
not calculate estimates or normalize quaternions.

Timestamped BMP, BMI, AHT, magnetometer, quaternion, and angular-velocity fields
are emitted only while fresh. Their current maximum ages are 1000, 250, 3000,
250, and 250 ms respectively. A retained stale sample is encoded with the same
unavailable sentinel as an invalid sample; it is never presented as current data.

| Fields | PDF parameter | Cansat source | Scale / encoding | Wire range type | Byte offsets (zero-based) | Field widths (bytes) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | TEAM ID | `telemetry.teamId` | 27 ASCII bytes, zero-padded | char array | 0 | 27 |
| 2 | TIME STAMPING | `telemetry.uptimeMs` | Already milliseconds since boot | u24 | 28 | 3 |
| 3 | PACKET COUNT | `telemetry.packetCount` | Integer | uint16 | 32 | 2 |
| 4 | ALTITUDE | `bmp.relativeHeightM` | metres x10; requires fresh bmp.valid and heightValid | uint16 | 35 | 2 |
| 5 | PRESSURE | `bmp.pressure` | Pa x1 | u24 | 38 | 3 |
| 6 | TEMPERATURE | `bmp.temperature` | degrees C x100 | int16 | 42 | 2 |
| 7 | VOLTAGE | `battery.voltage` | volts x1000 | uint16 | 45 | 2 |
| 8,9,10 | GNSS TIME | `gnss.utcTime`, `gnss.utcMilliseconds` | hour, minute, milliseconds within minute | uint8,uint8,uint16 | 48,50,52 | 1,1,2 |
| 11 | GNSS LATITUDE | `gnss.latitude` | absolute decimal degrees x10000 | u24 | 55 | 3 |
| 12 | GNSS LONGITUDE | `gnss.longitude` | absolute decimal degrees x10000 | u24 | 59 | 3 |
| 13,14 | GNSS HEMISPHERE | `gnss.latitudeHemisphere`, `longitudeHemisphere` | N/S, E/W | char,char | 63,65 | 1,1 |
| 15 | GNSS ALTITUDE | `gnss.altitude` | metres above mean sea level x10 | uint16 | 67 | 2 |
| 16 | GNSS SATELLITES | `gnss.satellites` | Integer; satellites used in fix | uint8 | 70 | 1 |
| 17,18,19 | ACCELEROMETER DATA | `bmi.accelX/Y/Z` | m/s^2 x100 | int16 x3 | 72,75,78 | 2,2,2 |
| 20,21,22 | GYROSCOPE DATA | `bmi.gyroX/Y/Z` | degrees/second x10 | int16 x3 | 81,84,87 | 2,2,2 |
| 23 | FLIGHT SOFTWARE STATE | `missionState` | 0 launch pad, 1 ascent, 2 descent 1, 3 descent 2, 4 touchdown | uint8 | 90 | 1 |
| 24 | HUMIDITY | `env.humidity` | %RH x1000 | u24 | 92 | 3 |
| 25,26,27 | MAGNETIC FIELD | `mag.magX/Y/Z` | microtesla /100 x10000 = microtesla x100 | int16 x3 | 96,99,102 | 2,2,2 |
| 28 | GNSS SPEED | `gnss.speed` | metres/second x100 | uint16 | 105 | 2 |
| 29 | COURSE | `gnss.course` | degrees x100 | uint16 | 108 | 2 |
| 30,31,32,33 | QUATERNION | `attitude.quaternion[0..3]` | dimensionless x10000, w,x,y,z | int16 x4 | 111,114,117,120 | 2,2,2,2 |
| 34,35,36 | ANGULAR VELOCITY | `attitude.angularVelocity[0..2]` | degrees/second x10, body X,Y,Z | int16 x3 | 123,126,129 | 2,2,2 |
| 37 | PARAFOIL STATUS | `actuators.parafoilDeployed` | 0/1 | bool | 132 | 1 |
| 38 | TARGET DISTANCE | `navigation.targetDistance` | metres x10 | uint16 | 134 | 2 |
| 39 | HEADING ERROR | `navigation.headingError` | signed degrees x100 | int16 | 137 | 2 |
| 40,41 | LEFT/RIGHT SERVO | `actuators.servo.servo1Angle/servo2Angle` | whole degrees | uint8 x2 | 140,142 | 1,1 |
| 42 | LAST RECEIVED COMMAND | `telemetry.lastReceivedCommand` | Command ID | uint8 | 144 | 1 |
| 43 | COMMAND EXECUTION | `telemetry.commandExecuted` | 0/1 | bool | 146 | 1 |
| 44 | RECEIVED PACKET COUNT | `telemetry.receivedPacketCount` | Complete ground packets, modulo 65536 | uint16 | 148 | 2 |

GNSS time is three binary fields: hour, minute, and milliseconds within the minute.
12:34:59.875 encodes 12 (1 byte), 34 (1 byte), and 59875 (2 bytes), with separators.
The driver stores UTC as numeric HHMMSS plus milliseconds, not packed clock bytes.
GNSS coordinates remain signed decimal degrees in Cansat; the packet carries their
scaled magnitudes plus separate hemisphere bytes. A leap second 60.xxx is supported.
Servo1 is assumed left and servo2 right; these are command angles, not feedback.

## Unavailable values and reserved sentinels

As selected by the user, unavailable/nonfinite/out-of-range numeric measurements
keep their full field width and use **unsigned maximum / signed minimum**:

| Field encoding | Sentinel bytes (hex, little-endian) | Usable integer range |
| --- | --- | --- |
| unsigned 8-bit measurement | FF | 0..254 |
| unsigned 16-bit measurement | FF FF | 0..65534 |
| unsigned 24-bit measurement | FF FF FF | 0..16777214 |
| signed 16-bit measurement | 00 80 | -32767..32767 |

Invalid hemisphere is FF. No field is omitted or shortened. Semantic limits also
apply to coordinates, UTC, humidity, the existing mission-state enum, and 5..45
degree servo commands. Servo 1 is the left brake and servo 2 is the right brake.
The two counters, command ID, and boolean status fields
are known bookkeeping values and retain their full ranges without sentinels.

Reserving an endpoint means it cannot also represent a real measurement. For
example, -32768 is reserved in signed
16-bit measurement fields and 65535 in unsigned 16-bit measurement fields.

Validity comes from sensor flags, GNSS per-field flags, or estimate/servo validity
flags. Retained I2C samples use acquisition's last-good-sample contract. No additional
mission-dependent age threshold is applied. Valid finite negative barometric
heights are clamped to zero in ALTITUDE, identically for binary telemetry and CSV
logging; the stored sensor height is unchanged. Other out-of-range measurements
are not wrapped or saturated.

## Type and budget notes

- The corrected binary layout uses the revised **151-byte budget** and fits the
  restored **256-byte XBee TX buffer**. Each send passes an explicit length of 151;
  the UART buffer capacity does not determine RF packetization.
- The revised schema removes the one-byte reserved field and its delimiter from the original 153-byte layout. It contains 44 fields and 43 commas.
- C has no standard uint24_t. Sources remain float/double/uint32_t; packing writes
  exactly three low-order bytes after checking range. Never send sizeof(uint32_t)
  for these three-byte fields or transmit a native C struct directly.
- The 24-bit millisecond timestamp's last usable value is **4:39:37.214**, because
  0xFFFFFF is now reserved. At/above that limit it encodes the sentinel. Use a
  four-byte timestamp if longer operation is required (a protocol change).
- ALTITUDE clamps valid negative barometric heights to zero. GNSS_ALTITUDE still
  uses the sentinel for negative readings. Both unsigned fields have a last usable
  value of 6553.4 m with the sentinel reserved.
- uint16 packet counters wrap modulo 65536 (about 9.1 hours at 2 Hz). A wider
  counter would require updating the wire layout.
- Servo uint8 fields suit the current 5..45 degree commands. Signed deflections
  would need a signed convention/type.
- The native team ID array holds 27 characters plus NUL. The wire reserves exactly
  27 bytes. The default is the PDF example `2026-IN-SPACe-CAN-7USAT-016`;
  change `CANSAT_DEFAULT_TEAM_ID` if the flight team's actual ID differs.

## Integration and ground commands

`Communication_FormatTelemetry(cansat, uint8_t *buffer, capacity)` serializes the
current count and returns 151, or zero for invalid arguments/team ID/insufficient
capacity. It never changes Cansat. Use its returned length with `XBee_Send`.

`Communication_SendTelemetry()` packs a snapshot with packetCount+1, sends the
151-byte buffer, and commits the count only on XBEE_OK. First packet is 1;
rollover is 65535 -> 0. UART acceptance is not confirmed RF delivery; a transfer
that later aborts remains counted. XBee copies the whole packet before returning.

Initialization uses UART4 after CubeMX setup. The flight loop updates uptime in
Cansat and calls Communication_Update each pass. It services XBee, reads at most
256 incoming bytes. FlightApp separately attempts telemetry every 500 ms (2 Hz)
while telemetry.enabled is true, and logs CSV every 1000 ms (1 Hz). These are
nominal intervals: blocking sensor/SD operations can delay a loop pass. See
[CSV logging](Logging.md). The enable flag defaults false and is controlled by
TELEMETRY_ON/OFF. Busy/error sends do not build a backlog.
Startup snapshots servo commands. Future estimation/control/state-machine
operations must update their fields and validity flags in Cansat.

Commands are three **binary bytes**, not ASCII hex strings:

| ID | Command | Bytes | Execution |
| --- | --- | --- | --- |
| 1 | TELEMETRY_ON | A5 01 5A | Enables scheduled telemetry |
| 2 | TELEMETRY_OFF | A5 02 5A | Disables future scheduled telemetry |
| 3 | CALIBRATE_SENSORS | A5 03 5A | Temporarily disabled; received and reported but not executed |

`Communication_ProcessCommand` preserves partial framing across chunks and can
consume multiple packets in one chunk. A5 restarts framing; malformed frames and
unknown IDs are ignored. Any non-OK XBee read discards partial framing. Each
recognized packet updates `lastReceivedCommand` and increments `receivedPacketCount`
modulo 65536, including repeated commands. `commandExecuted` describes the latest
received command. `lastExecutedCommand` separately retains the last successful
command ID in RAM; it is not an additional telemetry/CSV field. Both IDs default
to zero (none). The existing 151-byte layout remains unchanged. Numeric command
IDs are distinct from the existing team ID string.

OFF does not abort a packet already queued/in progress, disable XBee reception,
or stop SD logging. ON resumes the flight loop's normal telemetry schedule. There
is no separate command acknowledgement packet. The OFF result can be observed in
SD logs/debugger; scheduled telemetry is then disabled. Explicit calls to
`Communication_SendTelemetry` still bypass the scheduled enable gate.

Command 3 is currently a no-op. The calibration-start call is commented out; it performs no sensor or flash operations. Reception updates command ID/count, but commandExecuted remains false and lastExecutedCommand is unchanged. Startup now performs ground-pressure and gyro calibration, then saves the pair before mission processing. Airborne boots restore the pair. See [calibration and flash recovery](Calibration.md).

UART4 RX uses circular DMA (`HAL_UARTEx_ReceiveToIdle_DMA`). UART IDLE and DMA
half/full-transfer interrupt callbacks copy new bytes into the driver's 2048-byte
software queue. `Communication_Update`, called every main-loop pass, services
the driver and drains up to 256 bytes through nonblocking `XBee_Read`. Parsing and
command execution occur in main-loop context, never in interrupt callbacks.
IDLE events and read chunks are not packet boundaries.

A non-DMA interrupt implementation could receive each byte with RXNE interrupts,
enqueue it, and use the same main-loop parser. Pure polling of the UART hardware
is different: bytes are not collected while the main loop is occupied by sensor
or SD operations. Polling an interrupt/DMA-filled software queue, as this code
does, retains asynchronous reception. The current DMA driver is unchanged.

## Verification

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_communication_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_acquisition_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_xbee_tests.ps1
cmake --build --preset Debug
cmake --build --preset Release
```

Host tests compare all 151 bytes against a golden packet, check fixed offsets,
16-bit signed and 24-bit unsigned encoding, native-unit scaling, sentinels/ranges,
short buffers/team padding, and payloads containing comma/CR/NUL. They also check
exact TX length, busy/error handling, count rollover, scheduling, command chunk
splits, back-to-back packets, malformed frames, resynchronization after RX errors,
and received/executed command bookkeeping. Tests also cover asynchronous calibration,
duplicate requests, initialization/motion/timeout failures, and reference persistence.
Hardware calibration quality and RF delivery remain board tests.
