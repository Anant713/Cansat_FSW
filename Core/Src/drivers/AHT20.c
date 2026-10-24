#include "drivers/AHT20.h"
#include <math.h>
#include <string.h>

static bool usable(AHT20 *dev)
{
    return dev != NULL && (dev->initialized ||
        Sensor_Fail(&dev->io, SENSOR_NOT_INITIALIZED));
}

static bool read_status(AHT20 *dev, uint8_t *status)
{
    const uint8_t command = 0x71;
    return SensorI2C_Send(&dev->io, &command, 1) &&
           SensorI2C_Receive(&dev->io, status, 1);
}

static uint8_t crc8(const uint8_t *data)
{
    uint8_t crc = 0xff;
    for (unsigned i = 0; i < 6; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x31U : 0U));
    }
    return crc;
}

bool AHT20_Init(AHT20 *dev, I2C_HandleTypeDef *bus)
{
    if (dev == NULL) return false;
    memset(dev, 0, sizeof(*dev));
    dev->io = (SensorI2C){bus, AHT20_ADDRESS, SENSOR_OK};
    if (bus == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    HAL_Delay(100);
    uint8_t status;
    if (!read_status(dev, &status)) return false;
    if ((status & 0x18U) != 0x18U) {
        /* May 2021 datasheet section 7.4: restore 1b, 1c, 1e using their
         * own calibration bytes. Protocol also documented by Zephyr's
         * aosong/dht20 driver (supports AHT20). Startup-only delays. */
        const uint8_t registers[] = {0x1b, 0x1c, 0x1e};
        for (unsigned i = 0; i < sizeof(registers); ++i) {
            uint8_t command[] = {registers[i], 0, 0};
            uint8_t saved[3];
            if (!SensorI2C_Send(&dev->io, command, sizeof(command))) return false;
            HAL_Delay(5);
            if (!SensorI2C_Receive(&dev->io, saved, sizeof(saved))) return false;
            HAL_Delay(10);
            command[0] |= 0xb0U;
            command[1] = saved[1]; command[2] = saved[2];
            if (!SensorI2C_Send(&dev->io, command, sizeof(command))) return false;
        }
        HAL_Delay(10);
        if (!read_status(dev, &status)) return false;
        if ((status & 0x18U) != 0x18U)
            return Sensor_Fail(&dev->io, SENSOR_CALIBRATION_ERROR);
    }
    if (status & 0x80U) return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    HAL_Delay(10);
    dev->initialized = true;
    return true;
}

bool AHT20_StartMeasurement(AHT20 *dev)
{
    if (!usable(dev)) return false;
    if (dev->state != AHT20_IDLE) return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    const uint8_t trigger[] = {0xac, 0x33, 0x00};
    if (!SensorI2C_Send(&dev->io, trigger, sizeof(trigger))) return false;
    dev->started_ms = HAL_GetTick();
    dev->state = AHT20_MEASURING;
    return true;
}

bool AHT20_IsReady(AHT20 *dev)
{
    if (!usable(dev)) return false;
    if (dev->state == AHT20_READY) { dev->io.status = SENSOR_OK; return true; }
    if (dev->state != AHT20_MEASURING ||
        (uint32_t)(HAL_GetTick()-dev->started_ms) < AHT20_MEASUREMENT_MS)
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    /* Read status + payload + CRC together, without a separate 0x71 command
     * or a one-byte transaction before the measurement frame. */
    uint8_t frame[7];
    if (!SensorI2C_Receive(&dev->io, frame, sizeof(frame))) {
        dev->state = AHT20_IDLE;
        return false;
    }
    memcpy(dev->frame, frame, sizeof(frame));
    ++dev->frame_count;
    dev->calculated_crc = crc8(frame);
    uint8_t status = frame[0];
    if (status & 0x80U) {
        if ((uint32_t)(HAL_GetTick()-dev->started_ms) >= AHT20_BUSY_TIMEOUT_MS) {
            dev->state = AHT20_IDLE;
            return Sensor_Fail(&dev->io, SENSOR_TIMEOUT);
        }
        return Sensor_Fail(&dev->io, SENSOR_NOT_READY);
    }
    if (dev->calculated_crc != frame[6]) {
        ++dev->crc_errors;
        dev->state = AHT20_IDLE;
        return Sensor_Fail(&dev->io, SENSOR_BAD_DATA);
    }
    if ((status & 0x18U) != 0x18U) {
        dev->state = AHT20_IDLE;
        dev->initialized = false;
        return Sensor_Fail(&dev->io, SENSOR_CALIBRATION_ERROR);
    }
    dev->state = AHT20_READY;
    return true;
}

bool AHT20_Read(AHT20 *dev, AHT20Data *data)
{
    if (data != NULL) memset(data, 0, sizeof(*data));
    if (dev == NULL) return false;
    if (data == NULL) return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    if (!AHT20_IsReady(dev)) return false;
    const uint8_t *frame = dev->frame;
    dev->state = AHT20_IDLE;
    if ((frame[0] & 0x18U) != 0x18U) {
        dev->initialized = false;
        return Sensor_Fail(&dev->io, SENSOR_CALIBRATION_ERROR);
    }
    uint32_t raw = ((uint32_t)frame[1] << 12) | ((uint32_t)frame[2] << 4) | (frame[3] >> 4);
    float rh = raw*(100.0f/1048576.0f) + dev->humidity_offset_rh;
    data->humidity = fminf(100.0f, fmaxf(0.0f, rh));
    data->timestamp_ms = HAL_GetTick();
    data->valid = true;
    return true;
}

bool AHT20_SetCalibration(AHT20 *dev, float humidity_offset_rh)
{
    if (!usable(dev)) return false;
    if (!isfinite(humidity_offset_rh) || fabsf(humidity_offset_rh) > 100.0f)
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    dev->humidity_offset_rh = humidity_offset_rh;
    dev->io.status = SENSOR_OK;
    return true;
}

bool AHT20_Calibrate(AHT20 *dev, const AHT20Data *sample, float reference_humidity_rh)
{
    if (!usable(dev)) return false;
    if (sample == NULL || !sample->valid || !isfinite(sample->humidity) ||
        sample->humidity <= 0.0f || sample->humidity >= 100.0f ||
        !isfinite(reference_humidity_rh) || reference_humidity_rh < 0.0f || reference_humidity_rh > 100.0f)
        return Sensor_Fail(&dev->io, SENSOR_BAD_ARGUMENT);
    return AHT20_SetCalibration(dev, dev->humidity_offset_rh + reference_humidity_rh - sample->humidity);
}
