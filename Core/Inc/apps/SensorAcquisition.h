#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include <stdbool.h>
#define ACQUISITION_PERIOD_MS 10U
#define CALIBRATION_GYRO_SAMPLES 128U
#define CALIBRATION_GYRO_TIMEOUT_MS 10000U

typedef enum {
    SENSOR_CAL_IDLE, SENSOR_CAL_RUNNING, SENSOR_CAL_SUCCESS, SENSOR_CAL_FAILED
} SensorCalibrationStatus;

typedef struct Cansat Cansat;

/* Single instance, main-loop context only. Call once after CubeMX peripheral
 * initialization. Attempts every sensor independently; false means at least
 * one failed startup, but successfully initialized devices remain usable.
 * Uses I2C2, USART6 (115200 baud), and the battery driver's ADC1.
 * Startup configures sensors and restores a valid saved gyro/ground-reference
 * pair after driver initialization. Never calculates a new ground zero on boot.
 * Without a saved record, pressure is usable but relative height is invalid. */
bool SensorAcquisition_Init(void);
/* Call after Init, before starting mission processing. On a launch-pad boot,
 * blocks for combined calibration and atomic flash commit. Airborne/touchdown
 * boots retain the calibration restored by Init and never establish a new zero. */
bool SensorAcquisition_CalibrateAtStartup(Cansat *cansat);

/* Explicit ground-only calibration operation, used by the startup wrapper.
 * Reinitializes BMP/AHT/BMI/MAG (blocking startup delays), preserves existing
 * correction coefficients, then starts asynchronous stationary gyro sampling.
 * Launch-pad only. UpdateCalibration services completion and atomically
 * persists both gyro biases and ground pressure. Failure restores the prior pair.
 * Repeated requests while RUNNING do not restart the operation. */
bool SensorAcquisition_StartCalibration(Cansat *cansat);
/* Called by the acquisition task; may append flash once on completion. */
SensorCalibrationStatus SensorAcquisition_UpdateCalibration(void);
SensorCalibrationStatus SensorAcquisition_GetCalibrationStatus(void);
/* True when a validated persisted pair is available (not a sensor-health flag). */
bool SensorAcquisition_HasCalibration(void);
float SensorAcquisition_GetGroundPressure(void);
/* Explicit ground-only bench zero after Init: blocking BMP reference sampling.
 * RAM-only, no gyro calibration or flash write. Failure invalidates BMP height.
 * Flight/recovery must continue using the combined persisted calibration. */
bool SensorAcquisition_CalibrateGroundPressure(void);

/* Call regularly on a zero-initialized Cansat (or after Cansat_Init).
 * Only sensor fields are written. No estimation, mission decisions, actuator
 * commands, automatic calibration, or sensor recovery is performed. BMI reads
 * advance gyro calibration only when explicitly started by the command operation.
 * I2C samples retain their last successful value/valid flag/timestamp when a
 * read fails or is not ready: valid means a sample exists, not that it is fresh.
 * GNSS always copies the driver's snapshot, including stale/no-fix flags.
 * Battery copies each conversion result, including invalid output on failure.
 * Transfers are synchronous and bounded by driver timeouts; no conversion waits
 * or busy loops are added here. NULL is ignored. */
void SensorAcquisition_Update(Cansat *cansat);

/* Individual updates return true when a valid sample/snapshot was copied.
 * False includes not-ready and failure; NULL returns false without I/O. */

bool SensorAcquisition_UpdateBMP(Cansat *cansat);

/* Services a pending conversion, or starts one when due (every 2000 ms).
 * Starting a conversion alone returns false; later calls collect the result. */
bool SensorAcquisition_UpdateAHT(Cansat *cansat);

bool SensorAcquisition_UpdateBMI(Cansat *cansat);

bool SensorAcquisition_UpdateMag(Cansat *cansat);

bool SensorAcquisition_UpdateGNSS(Cansat *cansat);

bool SensorAcquisition_UpdateBattery(Cansat *cansat);

#endif
