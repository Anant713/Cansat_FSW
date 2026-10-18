#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Short synchronous bus transfers; conversion waits are handled by each driver. */
#define SENSOR_I2C_TIMEOUT_MS 20U
typedef enum {
    SENSOR_OK = 0, SENSOR_NOT_READY, SENSOR_BAD_ARGUMENT, SENSOR_NOT_INITIALIZED,
    SENSOR_IO_ERROR, SENSOR_TIMEOUT, SENSOR_WRONG_ID, SENSOR_BAD_DATA,
    SENSOR_CALIBRATION_ERROR
} SensorStatus;

typedef struct {
    I2C_HandleTypeDef *handle;
    uint8_t address;                 /* Unshifted 7-bit address. */
    SensorStatus status;            /* Result of the most recent operation. */
} SensorI2C;

/* output = factor * matrix * (input - bias), in the same physical units. */
typedef struct {
    float bias[3];
    float matrix[3][3];
    float factor;
} SensorCalibration3D;
#define SENSOR_CALIBRATION_3D_IDENTITY \
    { {0.0f, 0.0f, 0.0f}, {{1.0f, 0.0f, 0.0f}, \
      {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, 1.0f }

bool SensorI2C_Read(SensorI2C *bus, uint8_t reg, uint8_t *data, uint16_t size);
bool SensorI2C_Write(SensorI2C *bus, uint8_t reg, const uint8_t *data, uint16_t size);
bool SensorI2C_WriteByte(SensorI2C *bus, uint8_t reg, uint8_t value);
bool SensorI2C_Send(SensorI2C *bus, const uint8_t *data, uint16_t size);
bool SensorI2C_Receive(SensorI2C *bus, uint8_t *data, uint16_t size);
bool Sensor_Fail(SensorI2C *bus, SensorStatus status);
bool SensorCalibration3D_IsValid(const SensorCalibration3D *cal);
void SensorCalibration3D_Apply(const SensorCalibration3D *cal,
                               const float input[3], float output[3]);
int16_t Sensor_SignedLE(const uint8_t *data);

#endif
