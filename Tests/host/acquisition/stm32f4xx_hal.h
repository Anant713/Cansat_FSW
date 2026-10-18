#ifndef MOCK_ACQUISITION_HAL_H
#define MOCK_ACQUISITION_HAL_H
#include <stdint.h>
typedef struct { unsigned instance; } I2C_HandleTypeDef;
typedef struct { unsigned instance; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
#endif
