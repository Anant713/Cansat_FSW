#include "Console.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static USART_TypeDef peripheral;
UART_HandleTypeDef huart3 = { &peripheral, HAL_UART_STATE_READY };
uint32_t mock_primask;
static uint8_t *rx_target;
static HAL_StatusTypeDef rx_result = HAL_ERROR, tx_result = HAL_OK;
static char transmitted[CONSOLE_TX_SIZE + 1U];
static unsigned tx_calls;
static uint16_t tx_length;
static bool inject_during_tx;
void USART3_IRQHandler(void);

void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{ assert(irq == USART3_IRQn && preempt == 2 && sub == 0); }
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == USART3_IRQn); }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart3 && size == 1);
    if (rx_result != HAL_OK) return rx_result;
    assert(uart->RxState == HAL_UART_STATE_READY);
    rx_target = data;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
void HAL_UART_IRQHandler(UART_HandleTypeDef *uart)
{
    uart->RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(uart);
    uart->Instance->SR = 0;
}
static void feed_byte(uint8_t byte)
{
    assert(!mock_primask && huart3.RxState == HAL_UART_STATE_BUSY_RX);
    *rx_target = byte;
    USART3_IRQHandler();
}
static void feed(const char *text) { while (*text) feed_byte((uint8_t)*text++); }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                     uint16_t size, uint32_t timeout)
{
    assert(uart == &huart3 && size <= CONSOLE_TX_SIZE && timeout == CONSOLE_TX_TIMEOUT_MS);
    assert(!mock_primask); /* Blocking TX must not mask RX interrupts. */
    ++tx_calls;
    tx_length = size;
    memcpy(transmitted, data, size);
    transmitted[size] = 0;
    if (inject_during_tx) { inject_during_tx = false; feed("during TX\n"); }
    return tx_result;
}
static ConsoleLineResult read_line(char *line, size_t capacity)
{
    ConsoleLineResult r = CONSOLE_NO_LINE;
    for (unsigned i = 0; i < 10 && r == CONSOLE_NO_LINE; ++i) r = Console_ReadLine(line, capacity);
    return r;
}
int main(void)
{
    char line[CONSOLE_LINE_MAX + 1U];
    assert(!Console_Write("before init"));
    assert(!Console_Init());
    assert(console_diagnostics.rx_restart_failures == 1);
    rx_result = HAL_OK;
    Console_Process();
    assert(huart3.RxState == HAL_UART_STATE_BUSY_RX);
    assert(Console_Printf("T=%.2f count=%lu", 25.5, 123UL));
    assert(strcmp(transmitted, "T=25.50 count=123") == 0);
    assert(Console_WriteBytes("A\0B", 3) && tx_length == 3 && transmitted[2] == 'B');
    char large[CONSOLE_TX_SIZE + 2U];
    memset(large, 'x', sizeof(large) - 1U); large[sizeof(large) - 1U] = 0;
    unsigned previous = tx_calls;
    assert(!Console_Printf("%s", large) && tx_calls == previous);
    assert(!Console_Write(large) && tx_calls == previous);
    assert(console_diagnostics.format_errors == 1);
    tx_result = HAL_TIMEOUT;
    assert(!Console_Write("timeout") && console_diagnostics.tx_errors == 1);
    tx_result = HAL_OK;
    inject_during_tx = true;
    assert(Console_Write("output"));
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "during TX"));
    feed("one\r\ntwo\nthree\r");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "one"));
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "two"));
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "three"));
    assert(read_line(line, sizeof(line)) == CONSOLE_NO_LINE);
    feed("abc\b\177Z\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "aZ"));
    feed("partial");
    assert(read_line(line, sizeof(line)) == CONSOLE_NO_LINE);
    feed(" command\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "partial command"));
    for (unsigned i = 0; i < 300; ++i) {
        feed("wrap\n");
        assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "wrap"));
    }
    for (unsigned i = 0; i < CONSOLE_LINE_MAX; ++i) feed_byte('a');
    feed("\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && strlen(line) == CONSOLE_LINE_MAX);
    for (unsigned i = 0; i < CONSOLE_LINE_MAX + 1; ++i) feed_byte('b');
    feed("suffix\nvalid\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_REJECTED);
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "valid"));
    feed("bad"); feed_byte(0); feed("tail\nnext\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_REJECTED);
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "next"));
    feed("too big\n");
    assert(read_line(line, 2) == CONSOLE_LINE_REJECTED);
    assert(Console_ReadLine(NULL, 0) == CONSOLE_BAD_ARGUMENT);
    feed("dangerous prefix");
    assert(read_line(line, sizeof(line)) == CONSOLE_NO_LINE);
    for (unsigned i = 0; i < CONSOLE_RX_SIZE + 10; ++i) feed_byte('c');
    assert(read_line(line, sizeof(line)) == CONSOLE_INPUT_LOST);
    assert(console_diagnostics.rx_dropped_bytes == 10);
    feed("suffix\nrecovered\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "recovered"));
    feed("prefix");
    assert(read_line(line, sizeof(line)) == CONSOLE_NO_LINE);
    peripheral.SR = USART_SR_ORE;
    rx_result = HAL_ERROR;
    feed_byte('x');
    assert(console_diagnostics.rx_errors == 1 && huart3.RxState == HAL_UART_STATE_READY);
    rx_result = HAL_OK;
    Console_Process();
    assert(read_line(line, sizeof(line)) == CONSOLE_INPUT_LOST);
    feed("\nstatus\n");
    assert(read_line(line, sizeof(line)) == CONSOLE_LINE_READY && !strcmp(line, "status"));
    UART_HandleTypeDef other = { &peripheral, HAL_UART_STATE_READY };
    HAL_UART_RxCpltCallback(&other);
    assert(read_line(line, sizeof(line)) == CONSOLE_NO_LINE);
    puts("Console tests passed: TX bounds/errors, RX during TX, lines, wrap, overflow and UART recovery.");
    return 0;
}
