#ifndef MOCK_GNSS_HAL_H
#define MOCK_GNSS_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {
    struct { unsigned Mode,Direction,MemInc,PeriphInc,MemDataAlignment,PeriphDataAlignment; } Init;
    uint32_t counter;
} DMA_HandleTypeDef;
typedef struct {
    DMA_HandleTypeDef *hdmarx;
    DMA_HandleTypeDef *hdmatx;
    struct { uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl; } Init;
    uint32_t ErrorCode;
} UART_HandleTypeDef;
#define DMA_CIRCULAR 1U
#define DMA_PERIPH_TO_MEMORY 2U
#define DMA_MINC_ENABLE 3U
#define DMA_PINC_DISABLE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_PDATAALIGN_BYTE 0U
#define UART_WORDLENGTH_8B 8U
#define UART_STOPBITS_1 1U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 3U
#define UART_HWCONTROL_NONE 0U
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->counter)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t p) { (void)p; }
static inline void __disable_irq(void) {}
static inline void __DMB(void) {}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
#endif
