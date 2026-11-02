#include "drivers/XBee.h"
#include "drivers/GNSS.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
GPIO_TypeDef mock_gpioa;
static UART_HandleTypeDef uart, other;
static DMA_HandleTypeDef rx_dma, tx_dma;
static uint8_t *rx, *tx;
static uint16_t rx_position, tx_length;
static uint32_t tick, starts, aborts;
static bool abort_immediate;
static GPIO_PinState awake = GPIO_PIN_SET, reset_pin;
static HAL_StatusTypeDef rx_result = HAL_OK, tx_result = HAL_OK, abort_result = HAL_OK;
/* GNSS is isolated in its own suite; real dispatcher still handles all events. */
void GNSS_RxEventCallback(UART_HandleTypeDef *p, uint16_t n) { (void)p; (void)n; }
void GNSS_ErrorCallback(UART_HandleTypeDef *p) { (void)p; }
void GNSS_TxCompleteCallback(UART_HandleTypeDef *p) { (void)p; }
uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin)
{ CHECK(p == &mock_gpioa && pin == Wake_Sleep_XBee_Pin); return awake; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState state)
{ CHECK(p == &mock_gpioa && pin == Xbee_Reset_Pin); reset_pin = state; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *p, uint8_t *data, uint16_t n)
{
    CHECK(p == &uart && n == XBEE_RX_DMA_SIZE);
    if (rx_result != HAL_OK) return rx_result;
    rx = data; rx_position = 0; rx_dma.counter = n; ++starts;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *p, uint8_t *data, uint16_t n)
{
    CHECK(p == &uart && n <= XBEE_TX_CAPACITY);
    if (tx_result != HAL_OK) return tx_result;
    tx = data; tx_length = n;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *p)
{
    CHECK(p == &uart); ++aborts;
    if (abort_result != HAL_OK) return abort_result;
    rx = NULL;
    if (abort_immediate) HAL_UART_AbortCpltCallback(p);
    return HAL_OK;
}
static XBeeDiagnostics stats(void) { XBeeDiagnostics d; XBee_GetDiagnostics(&d); return d; }
static void input(const uint8_t *bytes, uint16_t length, bool idle)
{
    CHECK(rx != NULL);
    for (uint16_t i = 0; i < length; ++i) {
        rx[rx_position++] = bytes[i];
        if (rx_position == XBEE_RX_DMA_SIZE) rx_position = 0;
        rx_dma.counter = XBEE_RX_DMA_SIZE - rx_position;
        if (rx_position == 0 || rx_position == XBEE_RX_DMA_SIZE / 2)
            HAL_UARTEx_RxEventCallback(&uart, rx_position ? rx_position : XBEE_RX_DMA_SIZE);
    }
    if (idle) HAL_UARTEx_RxEventCallback(&uart, rx_position);
}
static void finish_boot(void) { tick += XBEE_BOOT_GUARD_MS; XBee_Process(); }
static void consume_loss(void)
{
    uint8_t b; uint16_t n = 123;
    CHECK(XBee_Read(&b, 1, &n) == XBEE_RX_LOST && n == 0);
    CHECK(XBee_Read(&b, 1, &n) == XBEE_OK && n == 0);
}
int main(void)
{
    uint8_t output[800], bytes[800]; uint16_t n;
    for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;
    CHECK(XBee_GetStatus() == XBEE_UNINITIALIZED);
    XBee_Process();
    CHECK(XBee_Send(bytes, 153) == XBEE_NOT_READY);
    CHECK(XBee_Init(NULL) == XBEE_INVALID_ARGUMENT);
    rx_dma.Init.Mode = DMA_CIRCULAR; rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    tx_dma = rx_dma; tx_dma.Init.Mode = DMA_NORMAL; tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    uart.hdmarx = &rx_dma; uart.hdmatx = &tx_dma;
    uart.Init.BaudRate = 115200; uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1; uart.Init.Mode = UART_MODE_TX_RX;
    tx_dma.Init.Mode = DMA_CIRCULAR;
    CHECK(XBee_Init(&uart) == XBEE_INVALID_ARGUMENT);
    tx_dma.Init.Mode = DMA_NORMAL;
    rx_result = HAL_ERROR;
    CHECK(XBee_Init(&uart) == XBEE_IO_ERROR && XBee_GetStatus() == XBEE_FAULT);
    CHECK(XBee_Init(&uart) == XBEE_BUSY);
    rx_result = HAL_OK;
    CHECK(XBee_Recover() == XBEE_OK && XBee_GetStatus() == XBEE_RECOVERING);
    XBee_Process(); CHECK(starts == 0); /* Must wait for asynchronous abort completion. */
    HAL_UART_AbortCpltCallback(&other); XBee_Process(); CHECK(starts == 0);
    HAL_UART_AbortCpltCallback(&uart); XBee_Process(); CHECK(starts == 1);
    consume_loss();
    CHECK(XBee_Send(bytes, 153) == XBEE_NOT_READY);
    finish_boot(); CHECK(XBee_GetStatus() == XBEE_READY);
    CHECK(reset_pin == GPIO_PIN_SET);
    awake = GPIO_PIN_RESET;
    CHECK(!XBee_IsAwake() && XBee_GetStatus() == XBEE_READY);
    /* Leave ON/SLEEP low through the following successful transmission. */
    CHECK(XBee_Send(NULL, 1) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_Send(bytes, 0) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_Send(bytes, XBEE_TX_CAPACITY + 1) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_SendString(NULL) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_SendString("") == XBEE_INVALID_ARGUMENT);
    memset(output, 'a', XBEE_TX_CAPACITY + 1); output[XBEE_TX_CAPACITY + 1] = 0;
    CHECK(XBee_SendString((char *)output) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_Send(bytes, 153) == XBEE_OK && tx_length == 153 && tx != bytes);
    CHECK(memcmp(tx, bytes, 153) == 0);
    bytes[0] = 99; CHECK(tx[0] == 0); bytes[0] = 0;
    CHECK(XBee_SendString("must not overwrite") == XBEE_BUSY && tx[0] == 0);
    CHECK(XBee_Reset() == XBEE_BUSY);
    HAL_UART_TxCpltCallback(&other); CHECK(XBee_IsTxBusy());
    HAL_UART_TxCpltCallback(&uart); CHECK(!XBee_IsTxBusy() && stats().txCompleted == 1);
    awake = GPIO_PIN_SET;
    HAL_UART_TxCpltCallback(&uart); CHECK(stats().txCompleted == 1);
    tx_result = HAL_BUSY;
    CHECK(XBee_Send(bytes, 153) == XBEE_BUSY && !XBee_IsTxBusy());
    tx_result = HAL_OK;
    CHECK(XBee_SendString("hello") == XBEE_OK && tx_length == 5);
    HAL_UART_TxCpltCallback(&uart);

    /* Partial input, multiple DMA wraps, binary data and small application reads. */
    input(bytes, 7, false);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_OK && n == 0);
    HAL_UARTEx_RxEventCallback(&uart, 7);
    HAL_UARTEx_RxEventCallback(&uart, 128); /* Stale HT Size must not duplicate bytes. */
    CHECK(XBee_Read(output, 3, &n) == XBEE_OK && n == 3 && memcmp(output, bytes, 3) == 0);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_OK && n == 4 && memcmp(output, bytes + 3, 4) == 0);
    input(bytes, sizeof(bytes), true);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_OK && n == sizeof(bytes));
    CHECK(memcmp(output, bytes, sizeof(bytes)) == 0);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_OK && n == 0);
    CHECK(XBee_Read(NULL, 1, &n) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_Read(output, 0, &n) == XBEE_INVALID_ARGUMENT);
    CHECK(XBee_Read(output, 1, NULL) == XBEE_INVALID_ARGUMENT);
    uint32_t received = stats().receivedBytes;
    HAL_UARTEx_RxEventCallback(&other, 5);
    HAL_UART_ErrorCallback(&other);
    CHECK(stats().receivedBytes == received && stats().uartErrors == 0);
    input(bytes, 800, true); input(bytes, 800, true); input(bytes, 800, true);
    CHECK(stats().droppedBytes > 0);
    consume_loss(); /* Must not join bytes either side of a lost command segment. */
    CHECK(stats().droppedBytes == 2400);
    input(bytes, 11, true);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_OK && n == 11 && memcmp(output, bytes, 11) == 0);

    CHECK(XBee_Send(bytes, 153) == XBEE_OK);
    uart.ErrorCode = 8;
    HAL_UART_ErrorCallback(&uart);
    CHECK(XBee_GetStatus() == XBEE_FAULT && !stats().receiving);
    CHECK(XBee_Read(output, sizeof(output), &n) == XBEE_IO_ERROR);
    CHECK(XBee_Send(bytes, 153) == XBEE_IO_ERROR);
    HAL_UART_TxCpltCallback(&uart); CHECK(stats().txCompleted == 2);
    abort_immediate = true;
    CHECK(XBee_Recover() == XBEE_OK);
    XBee_Process(); consume_loss(); finish_boot();
    CHECK(stats().txAborted == 1 && stats().lastUartError == 8);
    CHECK(XBee_GetStatus() == XBEE_READY);

    /* Nonblocking reset pulse, including wraparound of the millisecond clock. */
    tick = UINT32_MAX - 1U;
    CHECK(XBee_Reset() == XBEE_OK); XBee_Process();
    CHECK(reset_pin == GPIO_PIN_RESET && XBee_GetStatus() == XBEE_BOOTING);
    CHECK(XBee_Recover() == XBEE_BUSY);
    ++tick; XBee_Process(); CHECK(reset_pin == GPIO_PIN_RESET);
    ++tick; XBee_Process(); CHECK(reset_pin == GPIO_PIN_SET);
    CHECK(XBee_Send(bytes, 153) == XBEE_NOT_READY);
    consume_loss(); finish_boot();
    CHECK(XBee_GetStatus() == XBEE_READY);
    CHECK(XBee_Send(bytes, XBEE_TX_CAPACITY) == XBEE_OK && tx_length == XBEE_TX_CAPACITY);
    HAL_UART_TxCpltCallback(&uart);
    tx_result = HAL_ERROR;
    CHECK(XBee_Send(bytes, 153) == XBEE_IO_ERROR && XBee_GetStatus() == XBEE_FAULT);
    abort_result = HAL_ERROR;
    CHECK(XBee_Recover() == XBEE_IO_ERROR);
    CHECK(XBee_GetStatus() == XBEE_FAULT);
    abort_result = HAL_OK; tx_result = HAL_OK;
    CHECK(XBee_Recover() == XBEE_OK); XBee_Process(); consume_loss(); finish_boot();
    XBee_GetDiagnostics(NULL);
    CHECK(aborts >= 5 && XBee_GetStatus() == XBEE_READY);
    puts("XBee tests passed: TX ownership/busy/length, RX wrap/chunks/loss, GPIO status, async reset/recovery, callback routing.");
    return 0;
}
