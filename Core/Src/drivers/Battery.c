#include "drivers/Battery.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

extern ADC_HandleTypeDef hadc1;
static bool initialized;

bool Battery_Init(void)
{
    initialized = false;
    if (hadc1.Instance != ADC1 ||
        hadc1.Init.Resolution != ADC_RESOLUTION_12B ||
        hadc1.Init.DataAlign != ADC_DATAALIGN_RIGHT ||
        hadc1.Init.ScanConvMode != DISABLE ||
        hadc1.Init.ContinuousConvMode != DISABLE ||
        hadc1.Init.NbrOfConversion != 1U ||
        hadc1.Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE ||
        hadc1.Init.ExternalTrigConv != ADC_SOFTWARE_START ||
        hadc1.Init.DMAContinuousRequests != DISABLE ||
        !isfinite(BATTERY_ADC_REFERENCE_V) || BATTERY_ADC_REFERENCE_V <= 0.0f ||
        !isfinite(BATTERY_DIVIDER_RATIO) || BATTERY_DIVIDER_RATIO <= 0.0f)
        return false;

    /* Reassert the dedicated channel and its settling time. This also keeps
     * the driver correct if CubeMX later regenerates its default 3 cycles. */
    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = ADC_CHANNEL_12;
    channel.Rank = 1;
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    initialized = HAL_ADC_ConfigChannel(&hadc1, &channel) == HAL_OK;
    return initialized;
}

bool Battery_Read(BatteryData *data)
{
    if (data == NULL) return false;
    *data = (BatteryData){0};
    if (!initialized) return false;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return false;
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_Stop(&hadc1) != HAL_OK || raw > 4095U) return false;
    data->adcRaw = (uint16_t)raw;
    data->voltage = ((float)raw / 4095.0f) * BATTERY_ADC_REFERENCE_V * BATTERY_DIVIDER_RATIO;
    data->valid = true;
    return true;
}
