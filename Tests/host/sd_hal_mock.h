#ifndef SD_HAL_MOCK_H
#define SD_HAL_MOCK_H
/* Preinclude this header; skip the real main.h for host builds. */
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>
/* Match STM32 widths without importing Windows' BYTE/byte/TCHAR macros. */
#define _FF_INTEGER
typedef int INT;
typedef unsigned int UINT;
typedef uint8_t BYTE;
typedef int16_t SHORT;
typedef uint16_t WORD, WCHAR;
typedef int32_t LONG;
typedef uint32_t DWORD;
typedef uint64_t QWORD;
typedef struct { uint32_t CR1; } SPI_TypeDef;
typedef struct {
    SPI_TypeDef *Instance;
    struct { uint32_t BaudRatePrescaler; } Init;
    void *hdmarx, *hdmatx;
} SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
extern SPI_TypeDef mock_spi;
#define SPI3 (&mock_spi)
#define SPI_CR1_BR_0 8U
#define SPI_CR1_BR 56U
#define CS_GPIO_Port ((void *)1)
#define CS_Pin 4U
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define HAL_SPI_STATE_READY 1
#define __HAL_SPI_ENABLE(h) ((void)(h))
#define __HAL_SPI_DISABLE(h) ((void)(h))
#define MODIFY_REG(reg, clear, set) ((reg) = ((reg) & ~(clear)) | (set))
uint32_t __get_IPSR(void);
uint32_t __get_PRIMASK(void);
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(void *port, uint16_t pin, int state);
int HAL_SPI_GetState(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi);
#endif
