/* Linked ONLY by TestHILS using --wrap; production drivers are unchanged. */
#include "HilsInput.h"
#include "drivers/BMP.h"
#include "main.h"
#include <math.h>
#include <string.h>

bool __wrap_BMP280_Init(BMP280 *dev, I2C_HandleTypeDef *bus, uint8_t address)
{
    if (!dev || !bus) return false;
    memset(dev, 0, sizeof(*dev));
    dev->io = (SensorI2C){bus, address, SENSOR_OK};
    dev->initialized = true; dev->mode = BMP280_NORMAL;
    return true;
}
bool __wrap_BMP280_Read(BMP280 *dev, BMP280Data *data)
{
    HilsSample sample;
    if (!dev || !dev->initialized || !data || !HilsInput_Take(HAL_GetTick(), &sample)) return false;
    *data = (BMP280Data){0};
    data->pressure = sample.pressurePa + dev->pressure_offset_pa;
    data->temperature = sample.temperatureC + dev->temperature_offset_c;
    data->timestamp_ms = sample.receivedMs; /* Never refresh an old PC sample. */
    data->groundPressurePa = dev->groundPressurePa;
    data->valid = true;
    if (dev->groundPressureValid && dev->groundPressurePa > 0) {
        data->relativeHeightM = 44330.0f * (1.0f - powf(data->pressure / dev->groundPressurePa, 0.1903f));
        data->heightValid = isfinite(data->relativeHeightM);
    }
    return true;
}
bool __wrap_BMP280_CalibrateGroundPressure(BMP280 *dev)
{
    float pressure;
    if (!dev || !dev->initialized || !HilsInput_GroundPressure(HAL_GetTick(), &pressure)) return false;
    return BMP280_SetGroundPressure(dev, pressure + dev->pressure_offset_pa);
}
