#include "App.h"
#include "Console.h"
#include "drivers/SDCard.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>

/* Debugger-visible: 0=not started, 1=running, 2=passed, -1=failed.
 * Creates a NEW SDTnnnn.BIN, never overwrites existing files. */
volatile int sd_test_state;
volatile FRESULT sd_test_result;
volatile uint32_t sd_test_stage;
char sd_test_filename[24];
static uint8_t expected[1574], received[1574];
static char command[CONSOLE_LINE_MAX + 1U];

static void report(void)
{
    Console_Printf("SD %s state=%d stage=%lu FRESULT=%u file=%s\r\n",
        sd_test_state == 2 ? "PASS" : sd_test_state == -1 ? "FAIL" : "RUNNING",
        sd_test_state, (unsigned long)sd_test_stage, (unsigned)sd_test_result, sd_test_filename);
    Console_Printf("SPI init_stage=%u CMD=%u sent=%u R1=0x%02X token=0x%02X HAL=%u R7=0x%08lX OCR=0x%08lX\r\n",
        (unsigned)sd_spi_diagnostics.init_stage, (unsigned)sd_spi_diagnostics.command,
        (unsigned)sd_spi_diagnostics.command_sent, (unsigned)sd_spi_diagnostics.response,
        (unsigned)sd_spi_diagnostics.token, (unsigned)sd_spi_diagnostics.hal_status,
        (unsigned long)sd_spi_diagnostics.r7, (unsigned long)sd_spi_diagnostics.ocr);
}

void App_Init(void)
{
    (void)Console_Init();
    Console_Write("SD write/read verification starting. Type status + Enter to repeat the result.\r\n");
    sd_test_state = 1;
    sd_test_stage = 1;
    if (!SDCard_Init()) goto failed;
    for (unsigned i = 0; i < sizeof(expected); ++i)
        expected[i] = (uint8_t)(i * 37U + i / 251U);
    sd_test_stage = 2;
    bool created = false;
    for (unsigned i = 0; i < 10000; ++i) {
        snprintf(sd_test_filename, sizeof(sd_test_filename), "0:/SDT%04u.BIN", i);
        if (SDCard_CreateFile(sd_test_filename)) { created = true; break; }
        if (SDCard_GetLastResult() != FR_EXIST) goto failed;
    }
    if (!created) goto failed;
    sd_test_stage = 3;
    if (!SDCard_Write(expected, 1537) || !SDCard_Sync() || !SDCard_CloseFile()) goto failed;
    sd_test_stage = 4;
    if (!SDCard_OpenAppend(sd_test_filename) || !SDCard_Write(expected + 1537, 37) ||
        !SDCard_CloseFile() || !SDCard_DeInit() || !SDCard_Init()) goto failed;
    sd_test_stage = 5;
    uint32_t length = 0;
    if (!SDCard_ReadFile(sd_test_filename, received, sizeof(received), &length)) goto failed;
    if (length != sizeof(expected) || memcmp(expected, received, sizeof(expected))) {
        sd_test_result = FR_DISK_ERR;
        sd_test_state = -1;
        (void)SDCard_DeInit();
        report();
        return;
    }
    sd_test_stage = 6;
    if (!SDCard_DeInit()) goto failed;
    sd_test_result = FR_OK;
    sd_test_state = 2;
    report();
    return;
failed:
    sd_test_result = SDCard_GetLastResult();
    sd_test_state = -1;
    (void)SDCard_DeInit();
    report();
}

void App_RunOnce(void)
{
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status") || !strcmp(command, "diag")) report();
        else Console_Write("Commands: status, diag, help. Reset to rerun the SD test.\r\n");
    } else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
}
