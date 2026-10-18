#include "main.h"
#include "drivers/Battery.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1 = {ADC1, {ADC_RESOLUTION_12B, ADC_DATAALIGN_RIGHT,
    DISABLE, DISABLE, 1, ADC_EXTERNALTRIGCONVEDGE_NONE, ADC_SOFTWARE_START, DISABLE}};
static int config_result, start_result, poll_result, stop_result;
static unsigned stops, starts;
static uint32_t raw;
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc, ADC_ChannelConfTypeDef *channel)
{
    assert(adc == &hadc1 && channel->Channel == ADC_CHANNEL_12 && channel->Rank == 1);
    assert(channel->SamplingTime == ADC_SAMPLETIME_480CYCLES);
    return config_result;
}
int HAL_ADC_Start(ADC_HandleTypeDef *adc) { assert(adc == &hadc1); ++starts; return start_result; }
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout)
{
    assert(adc == &hadc1 && timeout == 2); return poll_result;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc) { assert(adc == &hadc1); return raw; }
int HAL_ADC_Stop(ADC_HandleTypeDef *adc) { assert(adc == &hadc1); ++stops; return stop_result; }
static void expect_failure(void)
{
    BatteryData output = {1234, 12.0f, true};
    assert(!Battery_Read(&output));
    assert(!output.valid && output.adcRaw == 0 && output.voltage == 0);
}
int main(void)
{
    expect_failure(); assert(starts == 0);
    assert(!Battery_Read(NULL));
    assert(Battery_Init());
    const uint16_t samples[] = {0, 2048, 3723, 3909, 4095};
    const float volts[] = {0, 6.601612f, 12.00088f, 12.60044f, 13.2f};
    for (unsigned i = 0; i < 5; ++i) {
        BatteryData output;
        raw = samples[i];
        assert(Battery_Read(&output) && output.valid && output.adcRaw == raw);
        assert(fabsf(output.voltage - volts[i]) < 0.002f);
    }
    assert(stops == starts);
    poll_result = 1;
    unsigned before = stops;
    expect_failure(); assert(stops == before + 1);
    poll_result = 0; start_result = 1;
    before = stops; expect_failure(); assert(stops == before);
    start_result = 0; stop_result = 1; expect_failure();
    stop_result = 0; raw = 4096; expect_failure();
    config_result = 1; assert(!Battery_Init()); expect_failure();
    config_result = 0; hadc1.Init.Resolution = 10; assert(!Battery_Init());
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    assert(Battery_Init()); raw = 3909;
    BatteryData output; assert(Battery_Read(&output));
    puts("Battery tests passed: scaling, endpoints, timeout cleanup, HAL failures, stale-data clearing, configuration and retry");
    return 0;
}
