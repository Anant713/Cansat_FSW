#include "drivers/IIS2MDC.h"
#include <math.h>
#include <string.h>

static bool usable(IIS2MDC *dev)
{
    return dev != NULL && (dev->initialized ||
        Sensor_Fail(&dev->io, SENSOR_NOT_INITIALIZED));
}

bool IIS2MDC_Init(IIS2MDC *dev, I2C_HandleTypeDef *bus)
{
    if (dev == NULL) return false;
    memset(dev, 0, sizeof(*dev));
    dev->io = (SensorI2C){bus, IIS2MDC_ADDRESS, SENSOR_OK};
    dev->calibration = (SensorCalibration3D)IIS2MDC_DEFAULT_CALIBRATION;
    if (bus == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    HAL_Delay(20);
    uint8_t id, config;
    if (!SensorI2C_Read(&dev->io, 0x4f, &id, 1)) return false;
    if (id != 0x40U) return Sensor_Fail(&dev->io, SENSOR_WRONG_ID);
    if (!SensorI2C_WriteByte(&dev->io, 0x60, 0x20)) return false;
    uint32_t start = HAL_GetTick();
    do {
        HAL_Delay(1);
        if (!SensorI2C_Read(&dev->io, 0x60, &config, 1)) return false;
        if (!(config & 0x20U)) break;
        if ((uint32_t)(HAL_GetTick()-start) >= 100U)
            return Sensor_Fail(&dev->io, SENSOR_TIMEOUT);
    } while (true);
    if (!SensorI2C_WriteByte(&dev->io, 0x61, 0x02) ||
        !SensorI2C_WriteByte(&dev->io, 0x62, 0x10) ||
        !SensorI2C_WriteByte(&dev->io, 0x60, 0x88)) return false;
    dev->initialized = true;
    return true;
}

bool IIS2MDC_IsReady(IIS2MDC *dev)
{
    if (!usable(dev)) return false;
    uint8_t status;
    if (!SensorI2C_Read(&dev->io, 0x67, &status, 1)) return false;
    return (status & 8U) != 0U || Sensor_Fail(&dev->io, SENSOR_NOT_READY);
}

bool IIS2MDC_Read(IIS2MDC *dev, IIS2MDCData *data)
{
    if (data != NULL) memset(data, 0, sizeof(*data));
    if (dev == NULL) return false;
    if (data == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    if (!usable(dev)) return false;
    uint8_t status, bytes[6];
    if (!SensorI2C_Read(&dev->io, 0x67, &status, 1)) return false;
    if (!(status & 8U)) return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    /* I2C SUB bit7 enables incrementing across all six output bytes. */
    if (!SensorI2C_Read(&dev->io, 0x68 | 0x80, bytes, sizeof(bytes))) return false;
    float raw[3], corrected[3];
    for (unsigned i = 0; i < 3; ++i) raw[i] = Sensor_SignedLE(bytes+2*i)*0.15f;
    SensorCalibration3D_Apply(&dev->calibration, raw, corrected);
    for (unsigned i = 0; i < 3; ++i)
        if (!isfinite(corrected[i])) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    data->magX = corrected[0]; data->magY = corrected[1]; data->magZ = corrected[2];
    data->overrun = (status & 0x80U) != 0U;
    data->timestamp_ms = HAL_GetTick();
    data->valid = true;
    return true;
}

bool IIS2MDC_SetCalibration(IIS2MDC *dev, const SensorCalibration3D *calibration)
{
    if (!usable(dev)) return false;
    if (!SensorCalibration3D_IsValid(calibration)) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    dev->calibration = *calibration;
    dev->io.status = SENSOR_OK;
    return true;
}

bool IIS2MDC_GetCalibration(const IIS2MDC *dev, SensorCalibration3D *calibration)
{
    if (dev == NULL || !dev->initialized || calibration == NULL) return false;
    *calibration = dev->calibration;
    return true;
}
