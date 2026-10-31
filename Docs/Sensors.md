# Polling I2C sensors

The four drivers use the existing `hi2c2`: PB10=SCL, PB11=SDA, 100 kHz.
Call the APIs from the main loop after `MX_I2C2_Init()`, with the HAL tick running.
Each device has its own context; share the same I2C handle, not a context.
Do not call these APIs concurrently or from interrupt handlers.

| Device / header | 7-bit address | Output units | Default operation |
| --- | --- | --- | --- |
| BMP280 / `drivers/BMP.h` | `0x76` or `0x77` | temperature °C, pressure Pa | normal mode, temperature x2 / pressure x16, filter x4, 62.5 ms standby (~9.5 Hz) |
| AHT20 / `drivers/AHT20.h` | `0x38` | humidity %RH only | start a conversion, check readiness, read once |
| BMI270 / `drivers/bmi270.h` | `0x68` or `0x69` | acceleration m/s², gyro °/s | 100 Hz, ±16 g, ±2000 °/s, normal bandwidth, performance modes |
| IIS2MDC / `drivers/IIS2MDC.h` | `0x1e` | magnetic field µT | continuous 50 Hz, temperature compensation, offset cancellation, block data update |

Pass addresses **unshifted**. The transport performs the STM32 HAL left shift.
Select BMP/BMI addresses to match SDO/SDO1 wiring. Their CS/CSB pins must select
I2C. All sensors need suitable power/decoupling, and SDA/SCL
pull-ups to the board's I/O supply. Do not assume every breakout's pull-ups can
be combined without checking their total resistance.

## Startup and timing

`Init()` is a **blocking startup operation**: resets, startup delays, factory
coefficient reads, and (BMI270) the configuration upload happen here. Do not
call it every main-loop pass. Reinitialization resets user calibration to its
defaults, so reapply your saved coefficients afterward.

`BMP280_Init` does not sample launch pressure. Call `BMP280_CalibrateGroundPressure`
explicitly on the ground (settle 1 s, average 16 samples, about 2.7 s), or restore
it through `BMP280_SetGroundPressure` without reading/calibrating. Relative height
remains invalid until one of these succeeds. Flight acquisition restores saved
ground pressure and gyro biases after initialization; see [Calibration.md](Calibration.md).

BMI270 uploads the full 8192-byte image in 32-byte chunks. At 100 kHz, the image
upload alone is approximately 0.92 seconds including address/register overhead;
initialization also includes hardware startup waits and HAL overhead. This is
a wire-time estimate, not a board benchmark. The image is `const` in flash.

Runtime reads do not wait for a future conversion. BMI270 requires both fresh
accel and gyro flags; IIS2MDC checks data-ready; AHT20 defers readiness polling
until at least 80 ms after triggering. BMP280 normal-mode reads return the last
completed sample after startup. Its registers have no per-sample data-ready
flag, so repeated normal-mode reads can return the same measurement.

**The small I2C transfers themselves are synchronous.** Typical ideal bus time
at 100 kHz is approximately 0.8 ms for a six-byte register read and 1.35 ms for a
12-byte register read, plus status queries and HAL overhead. Calls pass a 20 ms
transaction timeout to HAL; STM32F4 HAL also has its own 25 ms bus-busy wait.
There are no unbounded sensor-ready loops. A disconnected/stuck device can
therefore take longer than a healthy read. This implementation removes the
conversion waits from the runtime loop, not all CPU time spent on I2C.

## Results and errors

Boolean functions return true on success. On false, inspect `dev.io.status`:

- `SENSOR_NOT_READY`: conversion pending, no fresh sample, or a requested
  operation cannot run yet. Continue other work and try again later.
- `SENSOR_IO_ERROR` / `SENSOR_TIMEOUT`: a HAL transaction failed/timed out, or a
  device did not finish an expected startup/busy sequence.
- `SENSOR_WRONG_ID`: the addressed BMP/BMI/IIS does not match the expected chip.
- `SENSOR_BAD_DATA`: invalid trims, BMI configuration rejection, or AHT CRC error.
- `SENSOR_CALIBRATION_ERROR`: AHT factory calibration status is still invalid.
- `SENSOR_BAD_ARGUMENT` / `SENSOR_NOT_INITIALIZED`: invalid API use.

`Read()` clears its output first and only sets `valid=true` on success. Keep your
last good sample in a separate application variable if you want to retain it
through temporary errors. `timestamp_ms` is the MCU read-completion time,
wrapping like `HAL_GetTick()`; it is not UTC or the exact physical sampling time.
For timestamped CSV, associate an RTC timestamp in the application and keep
sample age/validity if reusing measurements between log rows.

Slow polling deliberately discards intermediate samples. IIS2MDC reports this
as `overrun=true` while still returning the latest valid measurement. These
drivers do not provide FIFO history, attitude estimation, or synchronized
sampling across devices.

## Application use

See the compiled, complete example in [`Tests/test_sensors.c`](../Tests/test_sensors.c).
The basic interfaces are:

```c
#include "drivers/BMP.h"
#include "drivers/AHT20.h"
#include "drivers/bmi270.h"
#include "drivers/IIS2MDC.h"

extern I2C_HandleTypeDef hi2c2;
static BMP280 bmp;
static AHT20 aht;
static BMI270 imu;
static IIS2MDC mag;

/* Once, after CubeMX peripheral initialization. Check each return separately. */
bool bmp_ok = BMP280_Init(&bmp, &hi2c2, BMP280_ADDRESS_LOW);
bool aht_ok = AHT20_Init(&aht, &hi2c2);
bool imu_ok = BMI270_Init(&imu, &hi2c2, BMI270_ADDRESS_LOW);
bool mag_ok = IIS2MDC_Init(&mag, &hi2c2);

/* During a scheduled main-loop pass: */
BMP280Data baro;
BMI270Data motion;
IIS2MDCData field;
if (bmp_ok && BMP280_Read(&bmp, &baro)) { /* use baro */ }
if (imu_ok && BMI270_Read(&imu, &motion)) { /* use motion */ }
if (mag_ok && IIS2MDC_Read(&mag, &field)) { /* use field */ }
```

For AHT20, issue a start **once**, then check on later loop passes:

```c
/* On the humidity sampling schedule, only when idle: */
if (aht_ok && aht.state == AHT20_IDLE) {
    if (!AHT20_StartMeasurement(&aht)) { /* inspect aht.io.status */ }
}

/* Later passes, with other application work in between: */
if (aht_ok && AHT20_IsReady(&aht)) {
    AHT20Data env;
    if (AHT20_Read(&aht, &env)) { /* use env.humidity */ }
} else if (aht_ok && aht.io.status != SENSOR_NOT_READY) {
    /* Handle the communication/calibration error. */
}
```

Do not call `StartMeasurement()` unconditionally on every loop pass, and do not
busy-wait on `IsReady()`. Successful reads consume the result. CRC/bus failures
return the context to idle so the next scheduled conversion can retry. AHT
calibration-status loss marks it uninitialized; handle recovery explicitly.
The 500 ms busy timeout applies only if the device is **still busy**, so a
completed conversion can be read by a slower loop after that interval.

Your AHT20 datasheet section 4.4 recommends a collection period **greater than
one second** to limit self-heating. The example uses two seconds. Logging at
1–2 Hz can reuse the latest humidity measurement while retaining its timestamp.
The AHT20 still converts temperature internally, but the driver exposes only
humidity as requested.

BMP mode changes use `BMP280_SetMode(&bmp, BMP280_SLEEP/BMP280_NORMAL/BMP280_FORCED)`.
When leaving normal mode, the function requests sleep first and may return
`SENSOR_NOT_READY` while the current conversion finishes; retry the requested
mode on a later pass. Forced mode triggers one conversion (~44 ms maximum for
these oversampling settings); `Read()` then returns the context to sleep.

## Calibration

Factory compensation is already applied: BMP trim coefficients, AHT factory
calibration/status checks, BMI gyro X/Z cross-sensitivity, and IIS hardware
temperature/offset compensation. Application calibration below is additional.
No calibration function automatically writes MCU flash or sensor NVM.

**BMP280 and AHT20 reference offsets:** use a stable, valid sample and a known
reference instrument/environment. These functions do not infer a true absolute
pressure or humidity from an unreferenced measurement.

```c
/* Recent samples taken with the current offsets; these numbers are examples. */
bool pressure_cal_ok = BMP280_Calibrate(&bmp, &baro, 25.0f, 101325.0f);
bool humidity_cal_ok = AHT20_Calibrate(&aht, &env, 50.0f);

/* Or load previously measured offsets stored as constants in your code: */
bool pressure_offsets_ok = BMP280_SetCalibration(&bmp, temp_offset_c, pressure_offset_pa);
bool humidity_offset_ok = AHT20_SetCalibration(&aht, humidity_offset_rh);
```

Offsets are available in `bmp.temperature_offset_c`, `bmp.pressure_offset_pa`,
and `aht.humidity_offset_rh` for exporting to your calibration storage. Do not
use a clamped 0/100 %RH sample for AHT reference calibration. Factory BMP
temperature remains the input to pressure compensation; a user temperature
offset is applied to the reported temperature only.

**Saved accelerometer and magnetometer parameters:**

```c
static const BMI270Calibration imu_cal = {
    .accel = SENSOR_CALIBRATION_3D_IDENTITY,
    .gyro_bias_dps = {0, 0, 0}
};
static const SensorCalibration3D mag_cal = IIS2MDC_DEFAULT_CALIBRATION;

bool accel_cal_ok = BMI270_SetCalibration(&imu, &imu_cal);
bool mag_cal_ok = IIS2MDC_SetCalibration(&mag, &mag_cal);
```

IIS2MDC_Init automatically loads IIS2MDC_DEFAULT_CALIBRATION from IIS2MDC.h:
hard-iron bias `{-5.84, 3.08, 11.92}` in microtesla, and soft-iron matrix
`{{1.001, 0.012, 0.009}, {0.012, 1.006, -0.001}, {0.009, -0.001, 0.994}}`,
with factor 1. These values are compiled into firmware and survive power cycles;
no application setter call is needed. IIS2MDC_SetCalibration can override them
in RAM, but the next driver initialization restores the compiled defaults.
The offsets are assumed to be in microtesla and in sensor X/Y/Z order, as used
by this driver. Replace the accelerometer identity with its measured coefficients.
The transformation is `output = factor * matrix * (input - bias)`. For accel,
`bias` is in **m/s²**; for mag, hard-iron `bias` is in **µT**. Both matrices and
the factor are dimensionless. The magnetometer matrix accepts all nine elements
for soft-iron correction. Coordinates are sensor X/Y/Z; fit/apply coefficients
in the same axes. Getters return the current coefficients. Invalid, nonfinite,
or singular matrix parameters are rejected. Accelerometer gravity is retained.

These APIs apply saved accelerometer/full magnetometer calibration; they do not
perform a six-position accel fit or a full magnetic ellipsoid fit onboard. Those
parameters must be obtained from your calibration measurements and stored in
code, as requested. IIS hardware offset cancellation does not replace board
hard-/soft-iron calibration.

**Onboard gyro zero-rate calibration:**

```c
/* Once, with the board stationary. At 2 Hz reads this takes about 32 seconds. */
bool started = BMI270_StartGyroCalibration(&imu, 64, 120000);

/* Each later pass: BMI270_Read() collects at most one fresh sample. */
BMI270Data motion;
bool fresh = BMI270_Read(&imu, &motion);
if (imu.calibration_state == BMI270_CAL_DONE) {
    BMI270Calibration measured;
    if (BMI270_GetCalibration(&imu, &measured)) {
        /* Export measured.gyro_bias_dps, or keep using the applied RAM values. */
    }
}
```

The board must remain stationary for the entire collection. The driver checks
gravity magnitude (0.85–1.15 g), gyro magnitude (≤5 °/s per axis), gyro sample
standard deviation (≤0.5 °/s), and accel sample standard deviation (≤0.2 m/s²
per axis). Motion, bus errors, and deadline expiry leave the old bias intact.
Inspect `calibration_state` for `MOTION`, `IO_ERROR`, or `TIMEOUT`; restart only
after addressing the cause. These checks cannot prove stillness or distinguish
very slow constant rotation from bias. Never start calibration during flight.
Sample count is 32–4096; use a timeout suited to your actual polling interval.
`BMI270_CancelGyroCalibration()` cancels collection without changing the bias.

## Build and board verification

```powershell
cmake --preset TestSensors
cmake --build --preset TestSensors
```

Flash `build/TestSensors/test_sensors.elf` using your existing debugger. The
example attempts initialization separately for each device, samples BMP/BMI/IIS
every 200 ms and humidity every two seconds, and never initializes actuators.
The generated main still configures the existing board peripherals.

Inspect `sensor_init_ok`, `sensor_status`, `sensor_*_data`, and
`sensor_sample_count`. With the board still, acceleration magnitude should be
near 9.81 m/s² and gyro near zero. Compare temperature/pressure/humidity to known
references; rotate the magnetometer to check axes and the saved calibration.
Set `sensor_calibrate_gyro=true` in the debugger while stationary, then inspect
`sensor_gyro_calibration_state` and `sensor_saved_imu` after completion. This test
does not declare hardware pass just because I2C ACKs.

The flight application now uses these APIs through
[SensorAcquisition](SensorAcquisition.md). `BMI.h` forwards to `bmi270.h` and keeps
the old data-type alias; the unimplemented old `BMI_*`/`BME_*`/mag singleton
function declarations were replaced by the new APIs. The Cansat data header now
uses actual BMP280/BMI270/AHT20/IIS2MDC types.

Host regression tests (no hardware required):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_sensor_tests.ps1
# If gcc is not on PATH, append: -Compiler C:/MinGW/bin/gcc.exe
```

These validate known BMP compensation values, deferred AHT timing/CRC/recovery,
the full BMI upload sequence, units/signs, gyro calibration and failures, and IIS
auto-increment/overrun/matrix correction. They do not validate electrical wiring,
actual sensor noise, clock stretching, or timing on your assembled board.

## Source provenance

The supplied Bosch `bmi270.c`, version 2.86.1 (2023-05-03), contains the standard
8192-byte image. Its bytes were extracted unchanged into `bmi270_config.c`, with
the complete Bosch BSD-3-Clause notice retained. Image SHA-256:
`2d75e68e343a13ff99be98261dfbd99d9e8c6f267da9e3c5aeb883276ad178db`.
Keep that license with redistributed source and reproduce it with binary releases.
Our `bmi270.c` is a focused HAL polling driver, not Bosch's complete feature/FIFO
API. Do not add the downloaded vendor driver alongside it; that vendor file
requires the rest of Bosch's SensorAPI.

Register definitions and calculations were checked against your four supplied
datasheets and the [Bosch SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI).
AHT initialization follows the supplied May 2021 section 7.4 (status mask 0x18,
restore registers 0x1b/0x1c/0x1e and verify), with the restore transaction checked
against [Zephyr's AHT20/DHT20 driver](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/aosong/dht20/dht20.c).
The [current Aosong example](https://www.aosong.com/uploadfiles/2025/02/20250211143151669.zip)
also checks CRC and calibration/busy status; it assumes valid factory startup
status and does not implement that restore procedure.
