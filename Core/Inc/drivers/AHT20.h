#ifndef AHT20_H
#define AHT20_H
#include "drivers/SensorI2C.h"

#define AHT20_ADDRESS 0x38U
#define AHT20_MEASUREMENT_MS 80U
#define AHT20_BUSY_TIMEOUT_MS 500U
typedef struct {
    float humidity;                 /* %RH, corrected and clamped to 0..100 */
    uint32_t timestamp_ms;
    bool valid;
} AHT20Data;
typedef enum { AHT20_IDLE = 0, AHT20_MEASURING, AHT20_READY } AHT20State;
typedef struct {
    SensorI2C io;
    bool initialized;
    AHT20State state;
    uint32_t started_ms;
    float humidity_offset_rh;
} AHT20;

/* Startup only; waits for power-on/calibration. Measurements never HAL_Delay. */
bool AHT20_Init(AHT20 *dev, I2C_HandleTypeDef *bus);
bool AHT20_StartMeasurement(AHT20 *dev);
bool AHT20_IsReady(AHT20 *dev);
bool AHT20_Read(AHT20 *dev, AHT20Data *data);
bool AHT20_SetCalibration(AHT20 *dev, float humidity_offset_rh);
/* A one-point offset against a known reference, using a recent sample taken
 * with the current offset. Do not calibrate from a clamped 0/100 %RH sample. */
bool AHT20_Calibrate(AHT20 *dev, const AHT20Data *sample, float reference_humidity_rh);
#endif
