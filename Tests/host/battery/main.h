#ifndef BATTERY_MOCK_MAIN_H
#define BATTERY_MOCK_MAIN_H
#include <stdint.h>
#define ADC1 ((void *)1)
#define ADC_RESOLUTION_12B 12U
#define ADC_DATAALIGN_RIGHT 0U
#define DISABLE 0U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0U
#define ADC_SOFTWARE_START 0U
#define ADC_CHANNEL_12 12U
#define ADC_SAMPLETIME_480CYCLES 480U
#define HAL_OK 0
typedef struct {
    void *Instance;
    struct {
        uint32_t Resolution, DataAlign, ScanConvMode, ContinuousConvMode;
        uint32_t NbrOfConversion, ExternalTrigConvEdge, ExternalTrigConv, DMAContinuousRequests;
    } Init;
} ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
int HAL_ADC_Start(ADC_HandleTypeDef *);
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
int HAL_ADC_Stop(ADC_HandleTypeDef *);
#endif
