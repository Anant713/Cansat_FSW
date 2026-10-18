#ifndef IIS2MDC_H
#define IIS2MDC_H
#include "drivers/SensorI2C.h"
#define IIS2MDC_ADDRESS 0x1eU

typedef struct {
    float magX, magY, magZ;         /* microtesla, sensor axes */
    uint32_t timestamp_ms;
    bool overrun;                  /* Older samples were replaced; latest is valid. */
    bool valid;
} IIS2MDCData;
typedef struct {
    SensorI2C io;
    bool initialized;
    SensorCalibration3D calibration; /* hard iron bias in uT; soft iron matrix */
} IIS2MDC;

/* Continuous 50 Hz, temperature compensation, offset cancellation, BDU.
 * No interrupt pins used. */
bool IIS2MDC_Init(IIS2MDC *dev, I2C_HandleTypeDef *bus);
bool IIS2MDC_IsReady(IIS2MDC *dev);
bool IIS2MDC_Read(IIS2MDC *dev, IIS2MDCData *data);
bool IIS2MDC_SetCalibration(IIS2MDC *dev, const SensorCalibration3D *calibration);
bool IIS2MDC_GetCalibration(const IIS2MDC *dev, SensorCalibration3D *calibration);
#endif
