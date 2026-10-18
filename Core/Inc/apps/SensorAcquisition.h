#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include <stdbool.h>

typedef struct Cansat Cansat;

/* Single instance, main-loop context only. Call once after CubeMX peripheral
 * initialization. Attempts every sensor independently; false means at least
 * one failed startup, but successfully initialized devices remain usable.
 * Uses I2C2, USART6 (9600 baud), and the battery driver's ADC1.
 * Startup is blocking and resets driver calibration to its defaults. */
bool SensorAcquisition_Init(void);

/* Call regularly on a zero-initialized Cansat (or after Cansat_Init).
 * Only sensor fields are written. No estimation, mission decisions, actuator
 * commands, automatic calibration, or sensor recovery is performed.
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
