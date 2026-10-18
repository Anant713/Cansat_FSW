#include "drivers/bmi270.h"
#include "drivers/bmi270_config.h"
#include <math.h>
#include <string.h>

static bool usable(BMI270 *dev)
{
    return dev != NULL && (dev->initialized ||
        Sensor_Fail(&dev->io, SENSOR_NOT_INITIALIZED));
}

bool BMI270_Init(BMI270 *dev, I2C_HandleTypeDef *bus, uint8_t address)
{
    if (dev == NULL) return false;
    memset(dev, 0, sizeof(*dev));
    dev->io = (SensorI2C){bus, address, SENSOR_OK};
    dev->calibration.accel = (SensorCalibration3D)SENSOR_CALIBRATION_3D_IDENTITY;
    if (bus == NULL || (address != 0x68U && address != 0x69U))
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    HAL_Delay(2);
    uint8_t id, status;
    if (!SensorI2C_Read(&dev->io, 0x00, &id, 1)) return false;
    if (id != 0x24U) return Sensor_Fail(&dev->io, SENSOR_WRONG_ID);
    if (!SensorI2C_WriteByte(&dev->io, 0x7e, 0xb6)) return false;
    HAL_Delay(2);
    /* Advanced power saving must stay disabled for config/data accesses. */
    if (!SensorI2C_WriteByte(&dev->io, 0x7c, 0x00)) return false;
    HAL_Delay(1); /* >=450 us */
    if (!SensorI2C_WriteByte(&dev->io, 0x59, 0x00)) return false;
    for (uint16_t offset = 0; offset < BMI270_CONFIG_SIZE; offset += 32U) {
        uint16_t word = offset/2U;
        uint8_t addr[] = {(uint8_t)(word & 0x0fU), (uint8_t)(word >> 4)};
        if (!SensorI2C_Write(&dev->io, 0x5b, addr, sizeof(addr)) ||
            !SensorI2C_Write(&dev->io, 0x5e, bmi270_config_file+offset, 32)) return false;
    }
    if (!SensorI2C_WriteByte(&dev->io, 0x59, 0x01)) return false;
    HAL_Delay(20);
    uint32_t start = HAL_GetTick();
    do {
        if (!SensorI2C_Read(&dev->io, 0x21, &status, 1)) return false;
        uint8_t message = status & 0x0fU;
        if (message == 1U) break;
        if (message != 0U) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
        if ((uint32_t)(HAL_GetTick()-start) >= 100U)
            return Sensor_Fail(&dev->io, SENSOR_TIMEOUT);
        HAL_Delay(1);
    } while (true);
    /* Standard Bosch image: feature output page 0, byte 0x0c contains
     * the signed 7-bit factory gyro X/Z cross-sensitivity coefficient. */
    uint8_t cross;
    if (!SensorI2C_WriteByte(&dev->io, 0x2f, 0x00) ||
        !SensorI2C_Read(&dev->io, 0x3c, &cross, 1)) return false;
    cross &= 0x7fU;
    dev->gyro_cross_zx = (int8_t)((cross & 0x40U) ? (int)cross-128 : cross);
    if (!SensorI2C_WriteByte(&dev->io, 0x40, 0xa8) ||
        !SensorI2C_WriteByte(&dev->io, 0x41, 0x03) ||
        !SensorI2C_WriteByte(&dev->io, 0x42, 0xe8) ||
        !SensorI2C_WriteByte(&dev->io, 0x43, 0x00) ||
        !SensorI2C_WriteByte(&dev->io, 0x7d, 0x0e)) return false;
    HAL_Delay(50);
    /* ERR_REG reports fatal/internal errors; EVENT reports invalid sensor
     * configuration. Check both before exposing the device as initialized. */
    if (!SensorI2C_Read(&dev->io, 0x02, &status, 1)) return false;
    if (status & 0x1dU) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    if (!SensorI2C_Read(&dev->io, 0x1b, &status, 1)) return false;
    if (status & 0x1cU) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    dev->initialized = true;
    return true;
}

static void calibration_timeout(BMI270 *dev)
{
    if (dev->calibration_state == BMI270_CAL_COLLECTING &&
        (uint32_t)(HAL_GetTick()-dev->calibration_started_ms) >= dev->calibration_timeout_ms)
        dev->calibration_state = BMI270_CAL_TIMEOUT;
}

bool BMI270_IsReady(BMI270 *dev)
{
    if (!usable(dev)) return false;
    calibration_timeout(dev);
    uint8_t status;
    if (!SensorI2C_Read(&dev->io, 0x03, &status, 1)) {
        if (dev->calibration_state == BMI270_CAL_COLLECTING)
            dev->calibration_state = BMI270_CAL_IO_ERROR;
        return false;
    }
    return (status & 0xc0U) == 0xc0U || Sensor_Fail(&dev->io, SENSOR_NOT_READY);
}

static void collect_bias(BMI270 *dev, const float accel[3], const float gyro[3])
{
    if (dev->calibration_state != BMI270_CAL_COLLECTING) return;
    float norm2 = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
        norm2 += accel[i]*accel[i];
        if (fabsf(gyro[i]) > 5.0f) { dev->calibration_state = BMI270_CAL_MOTION; return; }
    }
    /* Within 0.85..1.15 g, with low variance across the complete interval. */
    if (norm2 < 0.85f*0.85f*BMI270_GRAVITY_MPS2*BMI270_GRAVITY_MPS2 ||
        norm2 > 1.15f*1.15f*BMI270_GRAVITY_MPS2*BMI270_GRAVITY_MPS2) {
        dev->calibration_state = BMI270_CAL_MOTION;
        return;
    }
    ++dev->calibration_count;
    for (unsigned i = 0; i < 3; ++i) {
        float delta = gyro[i] - dev->gyro_mean[i];
        dev->gyro_mean[i] += delta/dev->calibration_count;
        dev->gyro_m2[i] += delta*(gyro[i]-dev->gyro_mean[i]);
        delta = accel[i] - dev->accel_mean[i];
        dev->accel_mean[i] += delta/dev->calibration_count;
        dev->accel_m2[i] += delta*(accel[i]-dev->accel_mean[i]);
    }
    if (dev->calibration_count == dev->calibration_target) {
        for (unsigned i = 0; i < 3; ++i) {
            if (dev->gyro_m2[i]/(dev->calibration_count-1U) > 0.25f ||
                dev->accel_m2[i]/(dev->calibration_count-1U) > 0.04f) {
                dev->calibration_state = BMI270_CAL_MOTION;
                return;
            }
        }
        memcpy(dev->calibration.gyro_bias_dps, dev->gyro_mean, sizeof(dev->gyro_mean));
        dev->calibration_state = BMI270_CAL_DONE;
    }
}

bool BMI270_Read(BMI270 *dev, BMI270Data *data)
{
    if (data != NULL) memset(data, 0, sizeof(*data));
    if (dev == NULL) return false;
    if (data == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    if (!BMI270_IsReady(dev)) return false;
    uint8_t raw[12];
    if (!SensorI2C_Read(&dev->io, 0x0c, raw, sizeof(raw))) {
        if (dev->calibration_state == BMI270_CAL_COLLECTING)
            dev->calibration_state = BMI270_CAL_IO_ERROR;
        return false;
    }
    float accel[3], corrected[3], gyro[3];
    int32_t g[3];
    for (unsigned i = 0; i < 3; ++i) {
        accel[i] = Sensor_SignedLE(raw+2*i)*(16.0f*BMI270_GRAVITY_MPS2/32768.0f);
        g[i] = Sensor_SignedLE(raw+6+2*i);
    }
    g[0] -= (dev->gyro_cross_zx*g[2])/512;
    if (g[0] > 32767) g[0] = 32767;
    if (g[0] < -32768) g[0] = -32768;
    for (unsigned i = 0; i < 3; ++i) gyro[i] = g[i]*(2000.0f/32768.0f);
    SensorCalibration3D_Apply(&dev->calibration.accel, accel, corrected);
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(corrected[i])) return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    }
    collect_bias(dev, corrected, gyro); /* Uses unbiased gyro, never double-subtracts. */
    data->accelX = corrected[0]; data->accelY = corrected[1]; data->accelZ = corrected[2];
    data->gyroX = gyro[0] - dev->calibration.gyro_bias_dps[0];
    data->gyroY = gyro[1] - dev->calibration.gyro_bias_dps[1];
    data->gyroZ = gyro[2] - dev->calibration.gyro_bias_dps[2];
    data->timestamp_ms = HAL_GetTick();
    data->valid = true;
    return true;
}

bool BMI270_SetCalibration(BMI270 *dev, const BMI270Calibration *calibration)
{
    if (!usable(dev)) return false;
    if (dev->calibration_state == BMI270_CAL_COLLECTING)
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    if (calibration == NULL || !SensorCalibration3D_IsValid(&calibration->accel))
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    for (unsigned i = 0; i < 3; ++i)
        if (!isfinite(calibration->gyro_bias_dps[i])) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    dev->calibration = *calibration;
    dev->io.status = SENSOR_OK;
    return true;
}

bool BMI270_GetCalibration(const BMI270 *dev, BMI270Calibration *calibration)
{
    if (dev == NULL || !dev->initialized || calibration == NULL) return false;
    *calibration = dev->calibration;
    return true;
}

bool BMI270_StartGyroCalibration(BMI270 *dev, uint16_t samples, uint32_t timeout_ms)
{
    if (!usable(dev)) return false;
    if (dev->calibration_state == BMI270_CAL_COLLECTING)
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    if (samples < 32U || samples > 4096U || timeout_ms < (uint32_t)samples*10U || timeout_ms > 3600000U)
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    memset(dev->gyro_mean, 0, sizeof(dev->gyro_mean));
    memset(dev->gyro_m2, 0, sizeof(dev->gyro_m2));
    memset(dev->accel_mean, 0, sizeof(dev->accel_mean));
    memset(dev->accel_m2, 0, sizeof(dev->accel_m2));
    dev->calibration_target = samples;
    dev->calibration_count = 0;
    dev->calibration_timeout_ms = timeout_ms;
    dev->calibration_started_ms = HAL_GetTick();
    dev->calibration_state = BMI270_CAL_COLLECTING;
    dev->io.status = SENSOR_OK;
    return true;
}

void BMI270_CancelGyroCalibration(BMI270 *dev)
{
    if (dev != NULL) dev->calibration_state = BMI270_CAL_IDLE;
}
