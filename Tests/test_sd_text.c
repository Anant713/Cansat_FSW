#include "App.h"
#include "Console.h"
#include "drivers/SDCard.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>

/* Deterministic sample data, not live timestamps. Each run creates a new CSV. */
#define INITIAL_TEXT "time_ms,event\r\n0,opened\r\n"
#define APPENDED_TEXT "1000,appended\r\n"
static const char expected[] = INITIAL_TEXT APPENDED_TEXT;
volatile int sd_text_state; /* 0=not started, 1=running, 2=PASS, -1=FAIL */
volatile uint32_t sd_text_stage;
volatile FRESULT sd_text_result;
char sd_text_filename[24];
char sd_text_readback[128];
static char command[CONSOLE_LINE_MAX + 1U];

static void diagnostics(void)
{
    Console_Printf("SPI init_stage=%u CMD=%u sent=%u R1=0x%02X token=0x%02X HAL=%u R7=0x%08lX OCR=0x%08lX\r\n",
        (unsigned)sd_spi_diagnostics.init_stage, (unsigned)sd_spi_diagnostics.command,
        (unsigned)sd_spi_diagnostics.command_sent, (unsigned)sd_spi_diagnostics.response,
        (unsigned)sd_spi_diagnostics.token, (unsigned)sd_spi_diagnostics.hal_status,
        (unsigned long)sd_spi_diagnostics.r7, (unsigned long)sd_spi_diagnostics.ocr);
}
static void report(void)
{
    Console_Printf("SD TEXT %s state=%d stage=%lu FRESULT=%u file=%s\r\n",
        sd_text_state == 2 ? "PASS" : sd_text_state == -1 ? "FAIL" : "RUNNING",
        sd_text_state, (unsigned long)sd_text_stage, (unsigned)sd_text_result, sd_text_filename);
    diagnostics();
}
static bool verify(const char *text)
{
    uint32_t length = 0;
    memset(sd_text_readback, 0, sizeof(sd_text_readback));
    /* ReadFile opens in read mode and closes internally. Spare byte for NUL. */
    if (!SDCard_ReadFile(sd_text_filename, (uint8_t *)sd_text_readback,
                         sizeof(sd_text_readback) - 1U, &length)) {
        sd_text_result = SDCard_GetLastResult();
        return false;
    }
    if (length != strlen(text) || memcmp(sd_text_readback, text, length)) {
        sd_text_result = FR_DISK_ERR;
        return false;
    }
    return true;
}
static void run_test(void)
{
    sd_text_state = 1;
    sd_text_result = FR_OK;
    sd_text_filename[0] = '\0';
    sd_text_readback[0] = '\0';
    sd_text_stage = 1;
    if (!SDCard_Init()) goto io_failed;

    sd_text_stage = 2;
    bool created = false;
    for (unsigned i = 0; i < 10000U; ++i) {
        snprintf(sd_text_filename, sizeof(sd_text_filename), "0:/SDC%04u.CSV", i);
        if (SDCard_CreateFile(sd_text_filename)) { created = true; break; }
        if (SDCard_GetLastResult() != FR_EXIST) goto io_failed;
    }
    if (!created) { sd_text_result = FR_EXIST; goto failed; }
    Console_Printf("Opened new file: %s\r\n", sd_text_filename);

    sd_text_stage = 3;
    if (!SDCard_WriteString(INITIAL_TEXT)) goto io_failed;
    sd_text_stage = 4;
    if (!SDCard_CloseFile()) goto io_failed; /* Close flushes the file. */
    Console_Write("Wrote header + first row; closed file.\r\n");

    sd_text_stage = 5;
    if (!verify(INITIAL_TEXT)) goto failed;
    Console_Write("Reopened for reading; initial contents verified; closed.\r\n");

    sd_text_stage = 6;
    if (!SDCard_OpenAppend(sd_text_filename)) goto io_failed;
    sd_text_stage = 7;
    if (!SDCard_WriteString(APPENDED_TEXT)) goto io_failed;
    sd_text_stage = 8;
    if (!SDCard_CloseFile()) goto io_failed;
    Console_Write("Reopened for append; added second row; closed file.\r\n");

    sd_text_stage = 9;
    if (!SDCard_DeInit() || !SDCard_Init()) goto io_failed;
    sd_text_stage = 10;
    if (!verify(expected)) goto failed;
    Console_Write("Remounted; verified full file:\r\n");
    Console_Write(sd_text_readback);

    sd_text_stage = 11;
    if (!SDCard_DeInit()) goto io_failed;
    sd_text_stage = 12;
    sd_text_state = 2;
    sd_text_result = FR_OK;
    Console_Write("File closed and volume unmounted.\r\n");
    report();
    return;
io_failed:
    sd_text_result = SDCard_GetLastResult();
failed:
    sd_text_state = -1;
    (void)SDCard_DeInit(); /* Preserve the original test error above. */
    report();
}
void App_Init(void)
{
    (void)Console_Init();
    Console_Write("SD CSV open/write/close/append test. Commands: status, diag, run, help.\r\n");
    run_test();
}
void App_RunOnce(void)
{
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) report();
        else if (!strcmp(command, "diag")) diagnostics();
        else if (!strcmp(command, "run")) run_test();
        else Console_Write("Commands: status, diag, run (creates another CSV), help.\r\n");
    } else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
}
