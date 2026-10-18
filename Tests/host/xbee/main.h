#ifndef MOCK_XBEE_MAIN_H
#define MOCK_XBEE_MAIN_H
#include "stm32f4xx_hal.h"
extern GPIO_TypeDef mock_gpioa;
#define Xbee_Reset_GPIO_Port (&mock_gpioa)
#define Xbee_Reset_Pin 4U
#define Wake_Sleep_XBee_GPIO_Port (&mock_gpioa)
#define Wake_Sleep_XBee_Pin 8U
#endif
