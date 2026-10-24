#include "App.h"
#include "Console.h"
#include "main.h"
#include "drivers/XBee.h"
#include <string.h>

/* USART3 is the test console; XBee remains on UART4 (PA0/PA1), using DMA. */
extern UART_HandleTypeDef huart4;
volatile XBeeDiagnostics xbee_test_diagnostics;
volatile XBeeStatus xbee_test_status;
static uint32_t report_tick;
static uint8_t pending[XBEE_TX_CAPACITY];
static uint16_t pending_length;
static char line[CONSOLE_LINE_MAX + 1U];
static bool fixed_sent, initialized;
_Static_assert(CONSOLE_LINE_MAX + 2U <= XBEE_TX_CAPACITY, "Console line plus CRLF must fit XBee TX");

static void fixed_message(void)
{
    if (pending_length) { Console_Write("Local TX queue busy; retry.\r\n"); return; }
    static const char prefix[] = "XBEE_TEST_153_BYTES,";
    memcpy(pending, prefix, sizeof(prefix) - 1U);
    for (unsigned i = sizeof(prefix) - 1U; i < 151U; ++i) pending[i] = (uint8_t)('0' + i % 10U);
    pending[151] = '\r'; pending[152] = '\n';
    pending_length = 153;
}

static void report(void)
{
    XBeeDiagnostics d;
    XBee_GetDiagnostics(&d);
    xbee_test_diagnostics = d;
    xbee_test_status = XBee_GetStatus();
    static const char *const states[] = {
        "UNINITIALIZED", "BOOTING", "ASLEEP", "READY", "TRANSMITTING", "RECOVERING", "FAULT"
    };
    Console_Printf("Status=%s ON/SLEEP=%u TX started/done/aborted=%lu/%lu/%lu RX=%lu dropped=%lu UART errors=%lu console RX drops/errors=%lu/%lu TX errors=%lu\r\n",
        states[xbee_test_status], XBee_IsAwake() ? 1U : 0U,
        (unsigned long)d.txStarted, (unsigned long)d.txCompleted, (unsigned long)d.txAborted,
        (unsigned long)d.receivedBytes, (unsigned long)d.droppedBytes, (unsigned long)d.uartErrors,
        (unsigned long)console_diagnostics.rx_dropped_bytes,
        (unsigned long)console_diagnostics.rx_errors, (unsigned long)console_diagnostics.tx_errors);
}

static void handle_line(void)
{
    if (strcmp(line, "/test") == 0) fixed_message();
    else if (strcmp(line, "/status") == 0) report();
    else if (strcmp(line, "/reset") == 0 || strcmp(line, "/recover") == 0) {
        bool reset = strcmp(line, "/reset") == 0;
        XBeeResult result = reset ? XBee_Reset() : XBee_Recover();
        if (result == XBEE_OK) {
            pending_length = 0;
            Console_Write("Reset/recovery started; pending local message discarded.\r\n");
        } else Console_Write("Reset/recovery refused (busy or not initialized).\r\n");
    } else if (strcmp(line, "/help") == 0) {
        Console_Write("/test: send 153 bytes; /status; /reset; /recover; /help. Other lines sent with CRLF.\r\n");
    } else {
        if (pending_length) Console_Write("Local TX queue busy; line rejected. Retry later.\r\n");
        else {
            size_t length = strlen(line);
            memcpy(pending, line, length);
            pending[length] = '\r'; pending[length + 1U] = '\n';
            pending_length = (uint16_t)(length + 2U);
        }
    }
}

void App_Init(void)
{
    (void)Console_Init();
    initialized = XBee_Init(&huart4) == XBEE_OK;
    Console_Write("XBee DMA test, UART4 115200 8N1. Console on USART3. Type /help + Enter.\r\n");
    Console_Write(initialized ? "One fixed 153-byte message will send when awake.\r\n" :
        "XBee init failed. Inspect DMA configuration; use /recover for an RX start error.\r\n");
}

void App_RunOnce(void)
{
    XBee_Process();
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(line, sizeof(line));
    if (input == CONSOLE_LINE_READY) handle_line();
    else if (input == CONSOLE_INPUT_LOST)
        Console_Write("Console input lost. Press Enter, then resend the line.\r\n");
    else if (input == CONSOLE_LINE_REJECTED)
        Console_Write("Line rejected: maximum 254 text bytes, no embedded NUL.\r\n");
    if (initialized && !fixed_sent && XBee_GetStatus() == XBEE_READY && !pending_length) {
        fixed_message(); fixed_sent = true;
    }
    if (pending_length && XBee_GetStatus() == XBEE_READY) {
        XBeeResult r = XBee_Send(pending, pending_length);
        if (r == XBEE_OK) {
            pending_length = 0;
            Console_Write("TX accepted by UART DMA (not an RF acknowledgement).\r\n");
        } else if (r == XBEE_IO_ERROR || r == XBEE_INVALID_ARGUMENT) {
            pending_length = 0;
            Console_Write("TX failed; message discarded, use /status and /recover.\r\n");
        }
    }
    uint8_t bytes[128]; uint16_t length;
    XBeeResult r = XBee_Read(bytes, sizeof(bytes), &length);
    if (r == XBEE_RX_LOST) Console_Write("RX stream interrupted: discard partial ground command.\r\n");
    else if (r == XBEE_OK && length) {
        Console_Write("\r\nRX chunk: ");
        Console_WriteBytes(bytes, length); /* Explicit length preserves embedded NUL. */
        Console_Write("\r\n");
    }
    if ((uint32_t)(HAL_GetTick() - report_tick) >= 10000U) {
        report_tick = HAL_GetTick(); report();
    }
}
