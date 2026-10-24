#include "App.h"
#include "Console.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Standalone SPI3 master test, no SD driver/FatFs calls.
 * Deliberately polling, one byte per millisecond: isolates physical SPI
 * before testing DMA. Partner: Tests/arduino/uno_spi_slave/uno_spi_slave.ino.
 * Disconnect the SD module before attaching the Uno. See Docs/SPI_Test.md. */
extern SPI_HandleTypeDef hspi3;
volatile int spi_test_state; /* 0=waiting, 1=running, 2=echo PASS, -1=FAIL */
volatile uint32_t spi_test_packets, spi_test_passes, spi_test_failures;
volatile uint32_t spi_test_hal_error, spi_test_clock_hz;
volatile uint16_t spi_test_mismatches;
volatile HAL_StatusTypeDef spi_test_hal_status;
uint8_t spi_test_rx[128]; /* Raw response: byte 0 ignored, then delayed echo. */
static char sentence[127], command[CONSOLE_LINE_MAX + 1U];
static uint32_t last_send;
static bool initialized;

static void send_sentence(void)
{
    int length = snprintf(sentence, sizeof(sentence),
        "Hello Arduino UNO! STM32 SPI3 test packet %lu.\n",
        (unsigned long)++spi_test_packets);
    if (length <= 0 || (size_t)length >= sizeof(sentence)) return;
    spi_test_state = 1;
    spi_test_mismatches = 0;
    memset(spi_test_rx, 0, sizeof(spi_test_rx));
    spi_test_hal_status = HAL_OK;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    /* SPI slave echoes the previous byte. An extra NUL clocks out the final
     * newline. First received byte is unspecified after slave startup/reset. */
    for (int i = 0; i <= length; ++i) {
        uint8_t tx = i < length ? (uint8_t)sentence[i] : 0;
        spi_test_hal_status = HAL_SPI_TransmitReceive(&hspi3, &tx, &spi_test_rx[i], 1, 100);
        if (spi_test_hal_status != HAL_OK) break;
        if (i > 0 && spi_test_rx[i] != (uint8_t)sentence[i - 1]) ++spi_test_mismatches;
        HAL_Delay(1); /* Allow Uno SPI ISR to preload its next response. */
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    spi_test_hal_error = HAL_SPI_GetError(&hspi3);
    bool passed = spi_test_hal_status == HAL_OK && spi_test_mismatches == 0;
    spi_test_state = passed ? 2 : -1;
    if (passed) ++spi_test_passes; else ++spi_test_failures;
    Console_Printf("SPI %s packet=%lu HAL=%u error=0x%08lX echo mismatches=%u SCK=%lu Hz\r\n",
        passed ? "PASS" : "FAIL", (unsigned long)spi_test_packets,
        (unsigned)spi_test_hal_status, (unsigned long)spi_test_hal_error,
        (unsigned)spi_test_mismatches, (unsigned long)spi_test_clock_hz);
    Console_Printf("TX: %s", sentence);
    if (!passed) {
        Console_Write("RX first bytes: ");
        for (unsigned i = 0; i < 12U; ++i) Console_Printf("%02X ", spi_test_rx[i]);
        Console_Write("\r\n");
    }
}
void App_Init(void)
{
    (void)Console_Init();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    /* Override only test speed. Leave CubeMX mode/pin configuration in place
     * so an incorrect setting is reported rather than silently repaired. */
    bool config_ok = hspi3.Instance == SPI3 && hspi3.Init.Mode == SPI_MODE_MASTER &&
        hspi3.Init.Direction == SPI_DIRECTION_2LINES && hspi3.Init.DataSize == SPI_DATASIZE_8BIT &&
        hspi3.Init.CLKPolarity == SPI_POLARITY_LOW && hspi3.Init.CLKPhase == SPI_PHASE_1EDGE &&
        hspi3.Init.NSS == SPI_NSS_SOFT && hspi3.Init.FirstBit == SPI_FIRSTBIT_MSB &&
        hspi3.Init.TIMode == SPI_TIMODE_DISABLE && hspi3.Init.CRCCalculation == SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    initialized = config_ok && HAL_SPI_Init(&hspi3) == HAL_OK;
    spi_test_clock_hz = HAL_RCC_GetPCLK1Freq() / 256U;
    if (!initialized) {
        spi_test_state = -1;
        Console_Write("SPI configuration/init FAIL: requires SPI3 master, mode 0, 8-bit, MSB first, software NSS.\r\n");
    } else Console_Write("SPI test: automatic sentence + delayed echo every 2 s. Type send to repeat.\r\n");
    last_send = HAL_GetTick();
}
void App_RunOnce(void)
{
    Console_Process();
    bool send = false;
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "send")) send = true;
        else Console_Printf("Commands: send, status. state=%d pass=%lu fail=%lu\r\n",
            spi_test_state, (unsigned long)spi_test_passes, (unsigned long)spi_test_failures);
    } else if (input < 0) Console_Write("Console input lost; press Enter and retry.\r\n");
    if (initialized && (send || (uint32_t)(HAL_GetTick() - last_send) >= 2000U)) {
        send_sentence();
        last_send = HAL_GetTick();
    }
}
