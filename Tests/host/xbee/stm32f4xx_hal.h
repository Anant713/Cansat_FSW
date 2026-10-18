#ifndef MOCK_XBEE_HAL_H
#define MOCK_XBEE_HAL_H
#include "../gnss/stm32f4xx_hal.h"
#define DMA_NORMAL 0U
#define DMA_MEMORY_TO_PERIPH 4U
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *);
#endif
