/* Keep GNSS's unit test isolated while exercising the real shared dispatcher. */
#include "drivers/XBee.h"
void XBee_RxEventCallback(UART_HandleTypeDef *p, uint16_t n) { (void)p; (void)n; }
void XBee_ErrorCallback(UART_HandleTypeDef *p) { (void)p; }
void XBee_TxCompleteCallback(UART_HandleTypeDef *p) { (void)p; }
void XBee_AbortCompleteCallback(UART_HandleTypeDef *p) { (void)p; }
