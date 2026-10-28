#include "drivers/BMP.h"
#include <math.h>
#include <string.h>

/* x2 temperature + x16 pressure: maximum conversion time rounds up to 44 ms. */
#define BMP280_CONVERSION_MS 44U
#define BMP280_CTRL 0x54U

static bool usable(BMP280 *dev)
{
    return dev != NULL && (dev->initialized ||
        Sensor_Fail(&dev->io, SENSOR_NOT_INITIALIZED));
}

bool BMP280_Init(BMP280 *dev, I2C_HandleTypeDef *bus, uint8_t address)
{
    if (dev == NULL) return false;
    memset(dev, 0, sizeof(*dev));
    dev->io = (SensorI2C){bus, address, SENSOR_OK};
    if (bus == NULL || (address != 0x76U && address != 0x77U))
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    HAL_Delay(3);
    uint8_t id, status, coeff[24];
    if (!SensorI2C_Read(&dev->io, 0xd0, &id, 1)) return false;
    if (id != 0x58U) return Sensor_Fail(&dev->io, SENSOR_WRONG_ID);
    if (!SensorI2C_WriteByte(&dev->io, 0xe0, 0xb6)) return false;
    HAL_Delay(3);
    uint32_t start = HAL_GetTick();
    do {
        if (!SensorI2C_Read(&dev->io, 0xf3, &status, 1)) return false;
        if (!(status & 1U)) break;
        if ((uint32_t)(HAL_GetTick()-start) >= 100U)
            return Sensor_Fail(&dev->io, SENSOR_TIMEOUT);
        HAL_Delay(1);
    } while (true);
    if (!SensorI2C_Read(&dev->io, 0x88, coeff, sizeof(coeff))) return false;
    dev->t1 = (uint16_t)coeff[0] | ((uint16_t)coeff[1] << 8);
    dev->t2 = Sensor_SignedLE(coeff+2);
    dev->t3 = Sensor_SignedLE(coeff+4);
    dev->p1 = (uint16_t)coeff[6] | ((uint16_t)coeff[7] << 8);
    for (unsigned i = 0; i < 8; ++i) dev->p[i] = Sensor_SignedLE(coeff+8+2*i);
    if (dev->p1 == 0U || dev->p1 == 0xffffU || dev->t1 == 0U || dev->t1 == 0xffffU)
        return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    /* Config written while sleeping after reset: t_sb=62.5 ms, filter=4. */
    if (!SensorI2C_WriteByte(&dev->io, 0xf5, 0x28) ||
        !SensorI2C_WriteByte(&dev->io, 0xf4, BMP280_CTRL | BMP280_NORMAL)) return false;
    dev->initialized = true;
    dev->mode = BMP280_NORMAL;
    dev->pending = true;
    dev->started_ms = HAL_GetTick();
    /* Startup-only settling and averaging. 110 ms exceeds the maximum
     * conversion + 62.5 ms standby, avoiding repeated reads of one sample. */
    HAL_Delay(1000);
    float sum = 0.0f;
    for (unsigned i = 0; i < 16U; ++i) {
        BMP280Data sample;
        uint32_t wait_start = HAL_GetTick();
        while (!BMP280_Read(dev, &sample)) {
            if (dev->io.status != SENSOR_NOT_READY) goto ground_failed;
            if ((uint32_t)(HAL_GetTick() - wait_start) >= 200U) {
                Sensor_Fail(&dev->io, SENSOR_TIMEOUT);
                goto ground_failed;
            }
            HAL_Delay(1);
        }
        if (sample.pressure < 30000.0f || sample.pressure > 110000.0f) {
            Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
            goto ground_failed;
        }
        sum += sample.pressure;
        if (i < 15U) HAL_Delay(110);
    }
    dev->groundPressurePa = sum / 16.0f;
    dev->groundPressureValid = true;
    return true;
ground_failed:
    dev->initialized = false;
    return false;
}

bool BMP280_SetMode(BMP280 *dev, BMP280Mode mode)
{
    if (!usable(dev)) return false;
    if (mode != BMP280_SLEEP && mode != BMP280_NORMAL && mode != BMP280_FORCED)
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    uint8_t status;
    /* Request sleep first when leaving normal mode. The current conversion
     * finishes in hardware; callers retry this function without spinning. */
    if (dev->mode == BMP280_NORMAL && mode != BMP280_NORMAL) {
        if (!SensorI2C_WriteByte(&dev->io, 0xf4, BMP280_CTRL)) return false;
        dev->mode = BMP280_SLEEP;
        dev->pending = false;
    }
    if (!SensorI2C_Read(&dev->io, 0xf3, &status, 1)) return false;
    if (status & 9U) return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    if (!SensorI2C_WriteByte(&dev->io, 0xf4, (uint8_t)(BMP280_CTRL | mode))) return false;
    dev->mode = mode;
    dev->pending = mode != BMP280_SLEEP;
    dev->started_ms = HAL_GetTick();
    return true;
}

bool BMP280_IsReady(BMP280 *dev)
{
    if (!usable(dev)) return false;
    if (dev->mode == BMP280_SLEEP ||
        (dev->pending && (uint32_t)(HAL_GetTick()-dev->started_ms) < BMP280_CONVERSION_MS))
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    uint8_t status;
    if (!SensorI2C_Read(&dev->io, 0xf3, &status, 1)) return false;
    /* In normal mode output registers hold the last completed sample even
     * during the next conversion. Wait only for startup/forced conversions. */
    if ((status & 1U) || (dev->pending && (status & 8U)))
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    return true;
}

bool BMP280_Read(BMP280 *dev, BMP280Data *data)
{
    if (data != NULL) memset(data, 0, sizeof(*data));
    if (dev == NULL) return false;
    if (data == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    if (!BMP280_IsReady(dev)) return false;
    uint8_t raw[6];
    if (!SensorI2C_Read(&dev->io, 0xf7, raw, sizeof(raw))) return false;
    int32_t rp = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t rt = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    if (rp == 0x80000 || rt == 0x80000) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    /* Bosch datasheet floating-point compensation, using factory trims. */
    double v1 = (rt/16384.0 - dev->t1/1024.0)*dev->t2;
    double v2 = rt/131072.0 - dev->t1/8192.0;
    v2 = v2*v2*dev->t3;
    int32_t fine = (int32_t)(v1+v2);
    float temperature = (float)((v1+v2)/5120.0);
    v1 = fine/2.0 - 64000.0;
    v2 = v1*v1*dev->p[4]/32768.0 + v1*dev->p[3]*2.0;
    v2 = v2/4.0 + dev->p[2]*65536.0;
    v1 = (dev->p[1]*v1*v1/524288.0 + dev->p[0]*v1)/524288.0;
    v1 = (1.0 + v1/32768.0)*dev->p1;
    if (v1 == 0.0) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    double pressure = (1048576.0-rp-v2/4096.0)*6250.0/v1;
    v1 = dev->p[7]*pressure*pressure/2147483648.0;
    v2 = pressure*dev->p[6]/32768.0;
    pressure += (v1+v2+dev->p[5])/16.0;
    data->temperature = temperature + dev->temperature_offset_c;
    data->pressure = (float)pressure + dev->pressure_offset_pa;
    if (!isfinite(data->temperature) || !isfinite(data->pressure) || data->pressure <= 0.0f)
        return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    data->timestamp_ms = HAL_GetTick();
    data->groundPressurePa = dev->groundPressurePa;
    if (dev->groundPressureValid && dev->groundPressurePa > 0.0f) {
        data->relativeHeightM = 44330.0f *
            (1.0f - powf(data->pressure / dev->groundPressurePa, 0.1903f));
        data->heightValid = isfinite(data->relativeHeightM);
    }
    data->valid = true;
    dev->pending = false;
    if (dev->mode == BMP280_FORCED) dev->mode = BMP280_SLEEP;
    return true;
}

bool BMP280_SetCalibration(BMP280 *dev, float temperature_offset_c, float pressure_offset_pa)
{
    if (!usable(dev)) return false;
    if (!isfinite(temperature_offset_c) || !isfinite(pressure_offset_pa))
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    float ground = dev->groundPressurePa + pressure_offset_pa - dev->pressure_offset_pa;
    if (dev->groundPressureValid && (!isfinite(ground) || ground <= 0.0f))
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    if (dev->groundPressureValid) dev->groundPressurePa = ground;
    dev->temperature_offset_c = temperature_offset_c;
    dev->pressure_offset_pa = pressure_offset_pa;
    dev->io.status = SENSOR_OK;
    return true;
}

bool BMP280_Calibrate(BMP280 *dev, const BMP280Data *sample,
                     float reference_temperature_c, float reference_pressure_pa)
{
    if (!usable(dev)) return false;
    if (sample == NULL || !sample->valid || !isfinite(sample->temperature) ||
        !isfinite(sample->pressure) || !isfinite(reference_pressure_pa) || reference_pressure_pa <= 0.0f)
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    return BMP280_SetCalibration(dev,
        dev->temperature_offset_c + reference_temperature_c - sample->temperature,
        dev->pressure_offset_pa + reference_pressure_pa - sample->pressure);
}
