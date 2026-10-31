#ifndef BMP280_H
#define BMP280_H
#include "drivers/SensorI2C.h"

#define BMP280_ADDRESS_LOW  0x76U
#define BMP280_ADDRESS_HIGH 0x77U
typedef enum { BMP280_SLEEP = 0, BMP280_FORCED = 1, BMP280_NORMAL = 3 } BMP280Mode;
typedef struct {
    float temperature;             /* degrees C */
    float pressure;                /* Pa, not hPa */
    uint32_t timestamp_ms;          /* host read time */
    bool valid;
    float groundPressurePa;        /* Explicitly calibrated/restored launch reference. */
    float relativeHeightM;         /* Relative to the saved launch reference. */
    bool heightValid;
} BMP280Data;
typedef struct {
    SensorI2C io;
    bool initialized;
    BMP280Mode mode;
    uint32_t started_ms;
    bool pending;
    uint16_t t1, p1;
    int16_t t2, t3, p[8];           /* p2 .. p9 */
    float temperature_offset_c, pressure_offset_pa;
    float groundPressurePa;
    bool groundPressureValid;
} BMP280;

/* Startup only: reset + factory coefficients; x2 temperature, x16 pressure,
 * filter x4, 62.5 ms standby, normal mode (~9.5 Hz).
 * Does not sample ground pressure. Height stays invalid until explicitly
 * calibrated or restored using SetGroundPressure. Reads do not wait. */
bool BMP280_Init(BMP280 *dev, I2C_HandleTypeDef *bus, uint8_t address);
/* Explicit ground-only operation in NORMAL mode: settles 1 s, then averages
 * 16 samples 110 ms apart (~2.7 s). Failure retains the previous reference. */
bool BMP280_CalibrateGroundPressure(BMP280 *dev);
/* Restore a validated reference without reading the sensor or recalibrating. */
bool BMP280_SetGroundPressure(BMP280 *dev, float pressurePa);
/* May return SENSOR_NOT_READY while a measurement is in progress; retry later.
 * Setting FORCED triggers one measurement. Read consumes it, then mode=SLEEP. */
bool BMP280_SetMode(BMP280 *dev, BMP280Mode mode);
bool BMP280_IsReady(BMP280 *dev);
bool BMP280_Read(BMP280 *dev, BMP280Data *data);
bool BMP280_SetCalibration(BMP280 *dev, float temperature_offset_c, float pressure_offset_pa);
/* Uses a valid previously returned sample and known reference values.
 * Pass a recent sample obtained with the CURRENT offsets. */
bool BMP280_Calibrate(BMP280 *dev, const BMP280Data *sample,
                     float reference_temperature_c, float reference_pressure_pa);
#endif
