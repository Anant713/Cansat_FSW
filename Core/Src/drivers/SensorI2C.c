#include "drivers/SensorI2C.h"
#include <math.h>

bool Sensor_Fail(SensorI2C *bus, SensorStatus status)
{
    if (bus != NULL) bus->status = status;
    return false;
}

static bool valid(SensorI2C *bus, const void *data, uint16_t size)
{
    if (bus == NULL || bus->handle == NULL || bus->address > 0x7fU ||
        data == NULL || size == 0U) return Sensor_Fail(bus, SENSOR_BAD_ARGUMENT);
    return true;
}

static bool result(SensorI2C *bus, HAL_StatusTypeDef status)
{
    bus->status = status == HAL_OK ? SENSOR_OK :
                  status == HAL_TIMEOUT ? SENSOR_TIMEOUT : SENSOR_IO_ERROR;
    return status == HAL_OK;
}

bool SensorI2C_Read(SensorI2C *bus, uint8_t reg, uint8_t *data, uint16_t size)
{
    if (!valid(bus, data, size)) return false;
    return result(bus, HAL_I2C_Mem_Read(bus->handle, (uint16_t)(bus->address << 1),
        reg, I2C_MEMADD_SIZE_8BIT, data, size, SENSOR_I2C_TIMEOUT_MS));
}

bool SensorI2C_Write(SensorI2C *bus, uint8_t reg, const uint8_t *data, uint16_t size)
{
    if (!valid(bus, data, size)) return false;
    /* HAL does not modify transmit buffers. */
    return result(bus, HAL_I2C_Mem_Write(bus->handle, (uint16_t)(bus->address << 1),
        reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, size, SENSOR_I2C_TIMEOUT_MS));
}

bool SensorI2C_WriteByte(SensorI2C *bus, uint8_t reg, uint8_t value)
{
    return SensorI2C_Write(bus, reg, &value, 1);
}

bool SensorI2C_Send(SensorI2C *bus, const uint8_t *data, uint16_t size)
{
    if (!valid(bus, data, size)) return false;
    return result(bus, HAL_I2C_Master_Transmit(bus->handle,
        (uint16_t)(bus->address << 1), (uint8_t *)data, size, SENSOR_I2C_TIMEOUT_MS));
}

bool SensorI2C_Receive(SensorI2C *bus, uint8_t *data, uint16_t size)
{
    if (!valid(bus, data, size)) return false;
    return result(bus, HAL_I2C_Master_Receive(bus->handle,
        (uint16_t)(bus->address << 1), data, size, SENSOR_I2C_TIMEOUT_MS));
}

int16_t Sensor_SignedLE(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)(value < 0x8000U ? (int32_t)value : (int32_t)value - 65536);
}

bool SensorCalibration3D_IsValid(const SensorCalibration3D *cal)
{
    if (cal == NULL || !isfinite(cal->factor) || cal->factor <= 0.0f) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(cal->bias[i])) return false;
        for (unsigned j = 0; j < 3; ++j)
            if (!isfinite(cal->matrix[i][j])) return false;
    }
    const float (*m)[3] = cal->matrix;
    float det = m[0][0] * (m[1][1]*m[2][2] - m[1][2]*m[2][1])
              - m[0][1] * (m[1][0]*m[2][2] - m[1][2]*m[2][0])
              + m[0][2] * (m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    return isfinite(det) && fabsf(det) > 1.0e-6f;
}

void SensorCalibration3D_Apply(const SensorCalibration3D *cal,
                               const float input[3], float output[3])
{
    float v[3];
    for (unsigned i = 0; i < 3; ++i) v[i] = input[i] - cal->bias[i];
    for (unsigned i = 0; i < 3; ++i)
        output[i] = cal->factor * (cal->matrix[i][0]*v[0] +
            cal->matrix[i][1]*v[1] + cal->matrix[i][2]*v[2]);
}
