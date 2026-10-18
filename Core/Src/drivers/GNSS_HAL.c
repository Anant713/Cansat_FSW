#include "drivers/GNSS.h"
#include "drivers/XBee.h"

/* Shared HAL callback dispatch. Add future UART clients here instead of
 * defining a second copy of these global callbacks. Each driver filters its handle. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    GNSS_RxEventCallback(uart, size);
    XBee_RxEventCallback(uart, size);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    GNSS_ErrorCallback(uart);
    XBee_ErrorCallback(uart);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    GNSS_TxCompleteCallback(uart);
    XBee_TxCompleteCallback(uart);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *uart)
{
    XBee_AbortCompleteCallback(uart);
}
