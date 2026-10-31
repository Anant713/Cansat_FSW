#ifndef IIS2MDC_H
#define IIS2MDC_H
#include "drivers/SensorI2C.h"
#define IIS2MDC_ADDRESS 0x1eU

/* Board calibration: bias in microtesla, sensor X/Y/Z axes.
 * corrected = matrix * (measured_uT - bias), with unit scale factor.
 * Compiled into firmware and restored by every IIS2MDC_Init(). */
#define IIS2MDC_DEFAULT_CALIBRATION \
    { {-5.84f, 3.08f, 11.92f}, \
      {{1.001f, 0.012f, 0.009f}, \
       {0.012f, 1.006f, -0.001f}, \
       {0.009f, -0.001f, 0.994f}}, 1.0f }

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
