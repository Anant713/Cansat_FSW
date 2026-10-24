#include "Console.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
volatile ConsoleDiagnostics console_diagnostics;
static uint8_t rx_byte, rx_buffer[CONSOLE_RX_SIZE];
static volatile uint32_t rx_head, rx_tail;
static volatile bool rx_lost;
static char tx_buffer[CONSOLE_TX_SIZE], line_buffer[CONSOLE_LINE_MAX + 1U];
static size_t line_length;
static bool discard_line, initialized;

static bool start_rx(void)
{
    if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1) == HAL_OK) return true;
    ++console_diagnostics.rx_restart_failures;
    return false;
}

bool Console_Init(void)
{
    if (initialized) return huart3.RxState != HAL_UART_STATE_READY;
    initialized = true;
    __HAL_UART_CLEAR_OREFLAG(&huart3); /* Discard pre-init bytes/errors. */
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    bool ok = start_rx();
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return ok;
}

void Console_Process(void)
{
    if (initialized && huart3.RxState == HAL_UART_STATE_READY) (void)start_rx();
}

/* This handler is linked ONLY in test firmware. Do not also generate it in
 * CubeMX. Observe errors here to avoid test dependencies in the shared driver
 * HAL error dispatcher: HAL can clear ErrorCode before returning. */
void USART3_IRQHandler(void)
{
    uint32_t errors = huart3.Instance->SR & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE);
    if (errors) {
        console_diagnostics.last_rx_error_flags = errors;
        ++console_diagnostics.rx_errors;
        rx_lost = true;
    }
    HAL_UART_IRQHandler(&huart3);
    /* Rearm after HAL finishes error handling, not inside RxCpltCallback. */
    Console_Process();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart3) return;
    ++console_diagnostics.rx_received_bytes;
    if (rx_lost || (uint32_t)(rx_head - rx_tail) >= CONSOLE_RX_SIZE) {
        ++console_diagnostics.rx_dropped_bytes;
        rx_lost = true;
        return;
    }
    rx_buffer[rx_head % CONSOLE_RX_SIZE] = rx_byte;
    __DMB();
    ++rx_head;
}

bool Console_WriteBytes(const void *bytes, size_t length)
{
    if (!initialized || (!bytes && length) || length > CONSOLE_TX_SIZE) return false;
    if (!length) return true;
    if (HAL_UART_Transmit(&huart3, (const uint8_t *)bytes, (uint16_t)length,
                          CONSOLE_TX_TIMEOUT_MS) == HAL_OK) return true;
    ++console_diagnostics.tx_errors;
    return false;
}

bool Console_Write(const char *text)
{
    if (!text) return false;
    size_t length = 0;
    while (length <= CONSOLE_TX_SIZE && text[length]) ++length;
    return Console_WriteBytes(text, length);
}

bool Console_Printf(const char *format, ...)
{
    if (!format) return false;
    va_list args;
    va_start(args, format);
    int length = vsnprintf(tx_buffer, sizeof(tx_buffer), format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof(tx_buffer)) {
        ++console_diagnostics.format_errors;
        return false; /* Never silently transmit a truncated report. */
    }
    return Console_WriteBytes(tx_buffer, (size_t)length);
}

ConsoleLineResult Console_ReadLine(char *line, size_t capacity)
{
    if (!line || !capacity) return CONSOLE_BAD_ARGUMENT;
    Console_Process();
    for (unsigned i = 0; i < 64U; ++i) {
        /* Mask interrupts only for byte consumption/loss handling, never TX. */
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (rx_lost) {
            rx_tail = rx_head;
            rx_lost = false;
            __set_PRIMASK(mask);
            line_length = 0;
            discard_line = true;
            return CONSOLE_INPUT_LOST;
        }
        if (rx_head == rx_tail) { __set_PRIMASK(mask); break; }
        uint8_t byte = rx_buffer[rx_tail % CONSOLE_RX_SIZE];
        ++rx_tail;
        __set_PRIMASK(mask);
        if (byte == '\r' || byte == '\n') {
            if (discard_line) { discard_line = false; line_length = 0; continue; }
            if (!line_length) continue;
            if (line_length >= capacity) {
                line_length = 0;
                ++console_diagnostics.rejected_lines;
                return CONSOLE_LINE_REJECTED;
            }
            memcpy(line, line_buffer, line_length);
            line[line_length] = '\0';
            line_length = 0;
            ++console_diagnostics.completed_lines;
            return CONSOLE_LINE_READY;
        }
        if (discard_line) continue;
        if (byte == '\b' || byte == 127U) {
            if (line_length) --line_length;
        } else if (!byte || line_length >= CONSOLE_LINE_MAX) {
            discard_line = true;
            line_length = 0;
            ++console_diagnostics.rejected_lines;
            return CONSOLE_LINE_REJECTED;
        } else line_buffer[line_length++] = (char)byte;
    }
    return CONSOLE_NO_LINE;
}
