# Sensor acquisition

`Core/Src/apps/SensorAcquisition.c` collects driver output into the shared
`Cansat` structure. Measurements retain driver units, axes, timestamps, and
validity flags. It does not calculate altitude from pressure, filter readings,
estimate attitude or velocity, evaluate mission gates, command actuators,
automatically calibrate sensors, transmit, or log. The explicit command-calibration
operation is also coordinated here; see [calibration and recovery](Calibration.md).

| Driver | Destination | Acquisition behavior |
| --- | --- | --- |
| BMP280 | `cansat.bmp` | Copy successful temperature/pressure reads |
| AHT20 | `cansat.env` | Start humidity conversion every two seconds; collect on later calls |
| BMI270 | `cansat.bmi` | Copy successful acceleration/angular-rate reads |
| IIS2MDC | `cansat.mag` | Copy successful magnetic-field reads, including overrun flag |
| GNSS | `cansat.gnss` | Process queued bytes, then copy the current snapshot |
| Battery | `cansat.battery` | Copy the ADC conversion result |

Switches and servo structures describe output states, not sensor measurements,
so acquisition does not write them. Attitude, navigation, and mission fields
also remain untouched.

## Startup and calling contract

After CubeMX has initialized peripherals, call `Cansat_Init(&cansat)` to clear
application data and set the default team ID, and `SensorAcquisition_Init()` to initialize the sensor drivers.
The latter attempts all six devices independently and returns true only if all
initialized. Successfully initialized devices continue operating after partial
startup failure. Acquisition does not attempt automatic reinitialization.
Inspect the individual I2C driver contexts and their `io.status` in the debugger
for failures; GNSS exposes its own diagnostics API.

The module owns one set of driver contexts and runs only in main-loop context.
Initialization is blocking, then reapplies the saved ground-pressure/gyro-bias pair
after driver defaults are loaded. It never measures a new ground zero at boot.
It uses I2C2, ADC1 through the battery driver, and USART6 at GNSS_EXPECTED_BAUD
(115200 baud). The BMP/BMI addresses default to `0x77`/`0x68`, matching the existing
sensor test. Adjust the constants at the top of `SensorAcquisition.c` to match
the board's address pins and receiver baud. GNSS receiver configuration is not
changed by acquisition.

`SensorAcquisition_Update(&cansat)` services all sensors independently.
Individual update functions are also available; they return true when a valid
sample or snapshot was copied. False can mean not ready, a failed read, an
uninitialized device, or NULL input. AHT starting a conversion alone returns
false. GNSS true means a fresh GGA snapshot exists; inspect `fixValid` separately.

The flight entry point now calls acquisition at a nominal 10 ms interval, with
no catch-up loop. This is a polling interval, not a guaranteed sample rate.
I2C and battery transactions are synchronous, bounded by their driver timeouts;
faults can delay subsequent operations. AHT conversion readiness is checked
without waiting, and its two-second trigger interval also limits retries after
a failed trigger. Tick comparisons handle the 32-bit HAL tick rollover.

`flight_cansat` and `flight_sensor_init_ok` are available in the debugger.
[Communication](Communication.md) now services XBee and sends telemetry when
enabled. [Logging](Logging.md) stores the same telemetry fields as CSV on the SD card.
State-machine processing follows acquisition and command service. Estimation/control
algorithms remain to be implemented. Without a valid saved calibration, relative
height remains invalid until explicitly calibrated on the launch pad.

## Stored measurement validity

BMP/AHT/BMI/magnetometer reads use a temporary sample. A failed or not-ready
read leaves the last successful structure unchanged, including its timestamp.
For these fields, `valid` means a successful sample has been stored; it does
**not** mean a new sample arrived this pass or that the sensor is still healthy.
Consumers must use the timestamp when deciding whether a measurement is recent
enough. Acquisition deliberately applies no mission-specific stale-data limits.
BMP normal-mode reads may return the same physical measurement with a later host
read timestamp, as documented by that driver.

GNSS snapshots are copied even when `GNSS_GetData()` returns false. This preserves
the driver's stale/no-fix and per-field validity changes, preventing an old fix
from remaining marked valid. Battery has no sample timestamp, so each conversion
copies the driver's result directly; a failure clears the data and its valid
flag. No battery-health decision is inferred from voltage.

## Verification

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_acquisition_tests.ps1
cmake --build --preset Debug
```

Host tests mock the driver boundary to verify copying, retained I2C measurements,
GNSS invalidation/no-fix snapshots, battery failure, independent startup failures,
AHT pending conversions/retry timing/tick rollover, NULL handling, preservation
of non-sensor fields, and flight-loop integration. Existing sensor and GNSS host
tests separately exercise the drivers. Hardware timing and measurements still
need verification on the assembled board.
