#ifndef MOCK_CONSOLE_HAL_H
#define MOCK_CONSOLE_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; uint32_t RxState; } UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
#define USART3_IRQn 39
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define __HAL_UART_CLEAR_OREFLAG(uart) ((uart)->Instance->SR = 0U)
extern uint32_t mock_primask;
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1; }
static inline void __set_PRIMASK(uint32_t mask) { mock_primask = mask; }
static inline void __DMB(void) { }
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
void HAL_UART_IRQHandler(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
#endif
