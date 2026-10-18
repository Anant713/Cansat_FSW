#ifndef TEST_CONSOLE_H
#define TEST_CONSOLE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Test-only USART3 console. Call from the main loop, never from interrupts.
 * Fixed buffers, blocking TX with timeout, interrupt-driven RX. */
#define CONSOLE_TX_SIZE 512U
#define CONSOLE_RX_SIZE 512U
#define CONSOLE_LINE_MAX 254U
#define CONSOLE_TX_TIMEOUT_MS 100U
typedef enum {
    CONSOLE_INPUT_LOST = -3, CONSOLE_LINE_REJECTED = -2,
    CONSOLE_BAD_ARGUMENT = -1, CONSOLE_NO_LINE = 0, CONSOLE_LINE_READY = 1
} ConsoleLineResult;
typedef struct {
    uint32_t rx_dropped_bytes, rx_errors, rx_restart_failures;
    uint32_t rejected_lines, tx_errors, format_errors;
} ConsoleDiagnostics;
extern volatile ConsoleDiagnostics console_diagnostics;

/* Call once after CubeMX USART3 init; Process retries failed RX starts. */
bool Console_Init(void);
void Console_Process(void);
/* No TX queue: data may be reused when the call returns. False indicates an
 * invalid argument, oversize report or UART failure (possibly partial TX).
 * WriteBytes preserves binary data, maximum CONSOLE_TX_SIZE bytes per call. */
bool Console_WriteBytes(const void *bytes, size_t length);
bool Console_Write(const char *text);
bool Console_Printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
/* CR/LF/CRLF; empty lines ignored; backspace/delete supported; no local echo.
 * At most one line per call, with bounded processing. Supply LINE_MAX+1 bytes.
 * Execute commands only on LINE_READY. On loss/invalid/oversized input the
 * affected command is discarded through a newline; after INPUT_LOST send Enter
 * before resending. The caller's output buffer is only changed on LINE_READY. */
ConsoleLineResult Console_ReadLine(char *line, size_t capacity);
#endif
