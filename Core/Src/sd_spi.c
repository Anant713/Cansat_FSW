/* Adapted from ControllersTech sd_spi.c, June 26, 2025.
 * https://controllerstech.com -- original permitted learning/development use.
 * SPI3 integration: bounded waits, SRAM DMA staging, SDSC/SDHC and CSD. */
#include "sd_spi.h"
#include <stdbool.h>
#include <string.h>

extern SPI_HandleTypeDef hspi3;
#define IO_TIMEOUT_MS 100U
#define READY_TIMEOUT_MS 1000U
#define INIT_TIMEOUT_MS 2000U

uint8_t card_initialized;
static uint8_t sdhc;
static uint32_t sector_count;
static bool io_failed;
static volatile bool dma_done, dma_error;
/* .bss is DMA-accessible SRAM1/2. Do not place these arrays in CCM.
 * Staging permits caller buffers in CCM or flash and unaligned buffers. */
static uint8_t dma_tx[SD_BLOCK_SIZE] __attribute__((aligned(4)));
static uint8_t dma_rx[SD_BLOCK_SIZE] __attribute__((aligned(4)));

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) dma_done = true;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) dma_error = true;
}
static bool thread_context(void)
{
    return __get_IPSR() == 0U && __get_PRIMASK() == 0U;
}
static uint8_t exchange(uint8_t value)
{
    uint8_t received = 0xFF;
    if (!io_failed && HAL_SPI_TransmitReceive(&hspi3, &value, &received,
                                             1, IO_TIMEOUT_MS) != HAL_OK)
        io_failed = true;
    return received;
}
static bool transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    if (io_failed || length == 0 || length > SD_BLOCK_SIZE) return false;
    if (tx) memcpy(dma_tx, tx, length);
    else memset(dma_tx, 0xFF, length);
#if SD_SPI_USE_DMA
    dma_done = false;
    dma_error = false;
    uint32_t started = HAL_GetTick();
    if (HAL_SPI_TransmitReceive_DMA(&hspi3, dma_tx, dma_rx, length) != HAL_OK)
        io_failed = true;
    while (!io_failed && !dma_done && !dma_error) {
        if ((uint32_t)(HAL_GetTick() - started) >= IO_TIMEOUT_MS) io_failed = true;
    }
    if (dma_error) io_failed = true;
    if (io_failed) {
        HAL_SPI_Abort(&hspi3); /* Stop both streams before reusing buffers. */
        return false;
    }
#else
    if (HAL_SPI_TransmitReceive(&hspi3, dma_tx, dma_rx, length, IO_TIMEOUT_MS) != HAL_OK) {
        io_failed = true;
        return false;
    }
#endif
    if (rx) memcpy(rx, dma_rx, length);
    return true;
}
static bool wait_ready(void)
{
    uint32_t started = HAL_GetTick();
    do {
        if (exchange(0xFF) == 0xFF && !io_failed) return true;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < READY_TIMEOUT_MS);
    return false;
}
static void deselect(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    (void)exchange(0xFF); /* Release MISO with eight clocks while CS is high. */
}
static bool select_card(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    (void)exchange(0xFF);
    return wait_ready();
}
/* CMD12 must interrupt an active read without waiting ready or toggling CS.
 * It has a stuff byte before R1 and a busy phase after R1. */
static uint8_t command(uint8_t cmd, uint32_t argument)
{
    if (cmd != 12U) {
        deselect();
        if (!select_card()) return 0xFF;
    }
    (void)exchange(0x40U | cmd);
    (void)exchange((uint8_t)(argument >> 24));
    (void)exchange((uint8_t)(argument >> 16));
    (void)exchange((uint8_t)(argument >> 8));
    (void)exchange((uint8_t)argument);
    (void)exchange(cmd == CMD0 ? 0x95 : cmd == CMD8 ? 0x87 : 0x01);
    if (cmd == 12U) (void)exchange(0xFF);
    for (unsigned retry = 0; retry < 10; ++retry) {
        uint8_t response = exchange(0xFF);
        if (!(response & 0x80U) && !io_failed) return response;
    }
    return 0xFF;
}
static bool receive_data(uint8_t *buffer, uint16_t length)
{
    uint32_t started = HAL_GetTick();
    uint8_t token;
    do {
        token = exchange(0xFF);
        if (token != 0xFF) break;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < READY_TIMEOUT_MS);
    if (io_failed || token != 0xFE || !transfer(NULL, buffer, length)) return false;
    (void)exchange(0xFF); /* CRC disabled; consume both CRC bytes. */
    (void)exchange(0xFF);
    return !io_failed;
}
static bool transmit_data(const uint8_t *buffer, uint8_t token)
{
    if (!wait_ready()) return false;
    (void)exchange(token);
    if (!transfer(buffer, NULL, SD_BLOCK_SIZE)) return false;
    (void)exchange(0xFF);
    (void)exchange(0xFF);
    bool accepted = (exchange(0xFF) & 0x1FU) == 0x05U;
    (void)exchange(0xFF);
    return wait_ready() && accepted && !io_failed;
}
static bool set_speed(uint32_t maximum_hz)
{
    uint32_t source = HAL_RCC_GetPCLK1Freq(), divisor = 2U, bits = 0;
    while (source / divisor > maximum_hz && divisor < 256U) {
        divisor <<= 1;
        bits += SPI_CR1_BR_0;
    }
    if (source / divisor > maximum_hz) return false;
    __HAL_SPI_DISABLE(&hspi3);
    MODIFY_REG(hspi3.Instance->CR1, SPI_CR1_BR, bits);
    hspi3.Init.BaudRatePrescaler = bits;
    __HAL_SPI_ENABLE(&hspi3);
    return true;
}
static bool read_capacity(void)
{
    uint8_t csd[16];
    if (command(9, 0) != 0 || !receive_data(csd, sizeof(csd))) return false;
    uint64_t sectors;
    if ((csd[0] >> 6) == 1U) {
        uint32_t size = ((uint32_t)(csd[7] & 0x3FU) << 16) |
                        ((uint32_t)csd[8] << 8) | csd[9];
        sectors = ((uint64_t)size + 1U) * 1024U;
    } else if ((csd[0] >> 6) == 0U) {
        uint32_t size = ((uint32_t)(csd[6] & 3U) << 10) |
                        ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        unsigned shift = ((csd[9] & 3U) << 1) | (csd[10] >> 7);
        sectors = (((uint64_t)size + 1U) << (shift + 2U + (csd[5] & 15U))) / 512U;
    } else return false;
    if (sectors == 0 || sectors > UINT32_MAX) return false;
    sector_count = (uint32_t)sectors;
    return true;
}
SD_Status SD_SPI_Init(void)
{
    if (!thread_context()) return SD_ERROR;
    card_initialized = 0;
    sector_count = 0;
    sdhc = 0;
    io_failed = false;
    if (hspi3.Instance != SPI3 || HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY ||
        hspi3.hdmarx == NULL || hspi3.hdmatx == NULL) return SD_ERROR;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    if (!set_speed(400000U)) return SD_ERROR;
    HAL_Delay(2);
    for (unsigned i = 0; i < 10; ++i) (void)exchange(0xFF);
    uint32_t started = HAL_GetTick();
    uint8_t response;
    do {
        response = command(CMD0, 0);
        if (response == 1) break;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < INIT_TIMEOUT_MS);
    if (response != 1 || io_failed) goto fail;
    response = command(CMD8, 0x1AA);
    bool version2 = response == 1;
    if (version2) {
        uint8_t r7[4];
        for (unsigned i = 0; i < 4; ++i) r7[i] = exchange(0xFF);
        if (r7[2] != 1 || r7[3] != 0xAA) goto fail;
    } else if (response != 5) goto fail;
    started = HAL_GetTick();
    do {
        if (command(CMD55, 0) > 1) goto fail;
        response = command(ACMD41, version2 ? 0x40000000U : 0);
        if (response == 0) break;
        if (response != 1 || io_failed) goto fail;
    } while ((uint32_t)(HAL_GetTick() - started) < INIT_TIMEOUT_MS);
    if (response != 0 || command(CMD58, 0) != 0) goto fail;
    uint32_t ocr = 0;
    for (unsigned i = 0; i < 4; ++i) ocr = (ocr << 8) | exchange(0xFF);
    if (!(ocr & 0x80000000U) || !(ocr & 0x00300000U)) goto fail;
    sdhc = version2 && (ocr & 0x40000000U);
    if (!sdhc && command(16, SD_BLOCK_SIZE) != 0) goto fail;
    if (!read_capacity()) goto fail;
    deselect();
    if (io_failed || !set_speed(8000000U)) goto fail;
    card_initialized = 1;
    return SD_OK;
fail:
    deselect();
    sector_count = 0;
    sdhc = 0;
    return SD_ERROR;
}
static bool valid_request(const void *buffer, uint32_t sector, uint32_t count)
{
    return thread_context() && card_initialized && buffer && count &&
           count <= UINT32_MAX / SD_BLOCK_SIZE && sector < sector_count &&
           count <= sector_count - sector &&
           (sdhc || sector + count - 1U <= UINT32_MAX / SD_BLOCK_SIZE);
}
static SD_Status finish(bool success)
{
    deselect();
    if (!success || io_failed) {
        card_initialized = 0;
        return SD_ERROR;
    }
    return SD_OK;
}
SD_Status SD_ReadBlocks(uint8_t *buffer, uint32_t sector, uint32_t count)
{
    if (!valid_request(buffer, sector, count)) return SD_ERROR;
    io_failed = false;
    bool multiple = count > 1;
    if (command(multiple ? 18 : CMD17, sdhc ? sector : sector * SD_BLOCK_SIZE) != 0)
        return finish(false);
    bool ok = true;
    while (count--) {
        if (!receive_data(buffer, SD_BLOCK_SIZE)) { ok = false; break; }
        buffer += SD_BLOCK_SIZE;
    }
    if (multiple) {
        uint8_t response = command(12, 0);
        bool ready = wait_ready();
        ok = ok && response == 0 && ready;
    }
    return finish(ok);
}
SD_Status SD_WriteBlocks(const uint8_t *buffer, uint32_t sector, uint32_t count)
{
    if (!valid_request(buffer, sector, count)) return SD_ERROR;
    io_failed = false;
    bool multiple = count > 1;
    if (command(multiple ? 25 : CMD24, sdhc ? sector : sector * SD_BLOCK_SIZE) != 0)
        return finish(false);
    bool ok = true;
    while (count--) {
        if (!transmit_data(buffer, multiple ? 0xFC : 0xFE)) { ok = false; break; }
        buffer += SD_BLOCK_SIZE;
    }
    if (multiple) {
        bool ready = wait_ready();
        if (ready) {
            (void)exchange(0xFD);
            (void)exchange(0xFF);
            ready = wait_ready();
        }
        ok = ok && ready;
    }
    if (ok) {
        /* CMD13 R2 detects programming errors after data acceptance. */
        uint8_t r1 = command(13, 0);
        uint8_t r2 = exchange(0xFF);
        ok = r1 == 0 && r2 == 0;
    }
    return finish(ok);
}
SD_Status SD_ReadMultiBlocks(uint8_t *buffer, uint32_t sector, uint32_t count)
{
    return SD_ReadBlocks(buffer, sector, count);
}
SD_Status SD_WriteMultiBlocks(const uint8_t *buffer, uint32_t sector, uint32_t count)
{
    return SD_WriteBlocks(buffer, sector, count);
}
SD_Status SD_Sync(void)
{
    if (!thread_context() || !card_initialized) return SD_ERROR;
    io_failed = false;
    bool ready = select_card();
    return finish(ready);
}
uint8_t sd_is_sdhc(void) { return sdhc; }
uint32_t SD_GetSectorCount(void) { return card_initialized ? sector_count : 0; }
