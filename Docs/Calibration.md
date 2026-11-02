# Sensor calibration and reboot recovery

Calibration now runs automatically during application startup. Keep the CanSat
stationary at the launch site. After `SensorAcquisition_Init`, call
`SensorAcquisition_CalibrateAtStartup`: it checks the saved mission state, runs the
combined ground/gyro calibration on LAUNCH_PAD and waits for its flash commit.
`CALIBRATE_SENSORS` (`A5 03 5A`) is temporarily a no-op: the execution call is
commented out. Reception is still counted, `commandExecuted` stays false and the
last successful command is unchanged. It neither calibrates nor writes flash.
SensorAcquisition owns calibration persistence; the state machine owns mission records.

## Calibration

1. Reinitialize BMP/AHT/BMI/MAG, preserving existing software correction coefficients.
   GNSS and battery are not restarted. Old sensor snapshots are invalidated.
2. Call `BMP280_CalibrateGroundPressure`. It settles for 1 second, then averages
   16 readings 110 ms apart. This is launch-height zero, not thermometer/barometer
   accuracy calibration. `BMP280_Init` now only initializes hardware/factory trims.
3. Call `BMI270_StartGyroCalibration` with `CALIBRATION_GYRO_SAMPLES` (128) and
   `CALIBRATION_GYRO_TIMEOUT_MS` (10000). Subsequent BMI reads collect stationary
   samples; the existing driver detects motion, timeout and I/O failures.
4. Once gyro calibration completes, acquire all three bias values with
   `BMI270_GetCalibration` and commit them with the ground reference as one record.
   `SensorAcquisition_UpdateCalibration` services completion after normal reads;
   it can also be called directly in main-loop context. The status accessor is
   read-only. There are no writes in interrupts.
5. Report success only after the complete flash append verifies. A failure restores
   the previous persisted gyro/ground pair in any initialized drivers; without an
   earlier valid pair, height stays invalid and gyro software bias defaults to zero.
   Failed hardware initialization is not magically repaired by this rollback.

The startup wrapper blocks for ground averaging and then services gyro sampling
until success/failure (128 samples, 10-second gyro timeout). The BMP sample's heightValid is suppressed while calibration is
running so a candidate reference cannot drive transitions or appear as valid
telemetry altitude before commit. Flight height gates remain inhibited if startup
calibration/recovery fails. Main-loop telemetry/logging start after this operation.

## Reboot

SensorAcquisition scans calibration flash, initializes hardware, then applies the
saved pressure with `BMP280_SetGroundPressure` and the three biases through
`BMI270_GetCalibration`/`BMI270_SetCalibration`. Driver initialization must come
before applying the values because it clears the device context.

Driver initialization restores any saved pair first. The startup wrapper then
recalibrates and appends a replacement pair on LAUNCH_PAD (including launch-pad
reboots). On ASCENT, DESCENT_1, DESCENT_2 or TOUCHDOWN it retains the restored pair,
without sampling a new zero or writing calibration. It reads mission flash before
deciding, without invoking state-entry actuators. An empty journal is seeded with
LAUNCH_PAD before calibration. Invalid mission storage blocks startup calibration.
`SensorAcquisition_HasCalibration` reports saved-pair availability, not sensor health.

Without a complete valid record, pressure readings remain available but relative
height is invalid on airborne recovery; perform a ground startup calibration.
An invalid/torn record with no earlier valid pair also makes sensor initialization
report failure. A valid earlier pair is used if a later append is interrupted.
Legacy mission records containing only rounded pressure are not a complete pair
and are not used as calibration. Perform one calibration on the ground after
upgrading from the old firmware.

`BMI270_Read` subtracts saved gyro biases from every measurement. BMP height,
telemetry ALTITUDE, state-transition checks and auxiliary navigation altitude all
use the same restored pressure reference. There is no separate navigation zero.

## Flash layout and failure handling

The existing reserved sector 11 (0x080E0000..0x080FFFFF, 128 KiB) contains both
16-byte mission records and 32-byte calibration records. Both use one append
allocator in MissionStore, so they cannot overwrite one another. Firmware remains
limited to 896 KiB by the linker. No new sector is used and no automatic erase runs.

Calibration record words, in order:

| Word | Content |
| --- | --- |
| 0 | CAL1 format/version identifier |
| 1 | Ground pressure, IEEE-754 binary32 Pa |
| 2..4 | X/Y/Z gyro biases, binary32 degrees/s |
| 5 | CRC-32 of words 0..4, least-significant byte first |
| 6 | Bitwise complement of CRC |
| 7 | Commit marker, written last |

Readback checks every programmed word. Loading checks format, commit, CRC and value
ranges (pressure 30000..110000 Pa; finite gyro biases within +/-5 degrees/s, matching
the stationary collector). Physical append order determines the newest valid
record. A partially programmed record cannot replace the previous complete tuple.
Calibration writes consume two 16-byte slots, mission writes one. Full flash or
programming failure returns failure without erasing or destroying the old record.
Reboot does not itself rewrite a recovered calibration; only explicit successful
calibration appends it. Raw flash has finite endurance/capacity; do not repeatedly
send completed calibration commands as a heartbeat.

Preserve sector 11 during flashing. A programmer mass erase removes both mission
and calibration history. Starting a new mission does not delete calibration;
explicitly recalibrate at the new launch location. No erase command is added here.

Only the ground pressure and gyro bias tuple is newly persisted. Accelerometer,
humidity and BMP pressure/temperature software offsets are not in this record;
their startup values remain driver defaults. Changing these offsets separately
requires keeping their reference/persistence policy consistent. Magnetometer
defaults are compiled in IIS2MDC.h and existing runtime corrections are preserved
across command reinitialization. Maximum ASCENT height remains RAM-only.

## Verification

`Tests/host/run_calibration_tests.ps1` covers mixed mission/calibration records,
every interrupted-write boundary, corrupt records, range checks, and full storage.
Acquisition tests check flash restore without resampling, bias application calls,
successful commit, failed-write rollback and invalid storage. Sensor tests exercise
actual BMP reference restore/averaging and BMI bias subtraction. State/communication
tests cover reference consistency, mission recovery and asynchronous command status.
Physical power-cut retention and calibrated sensor accuracy still require board tests.
