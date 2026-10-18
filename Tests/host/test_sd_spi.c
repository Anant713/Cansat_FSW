#include "sd_spi.h"
#include "sd_diskio_spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

SPI_TypeDef mock_spi;
SPI_HandleTypeDef hspi3 = {&mock_spi, {0}, (void *)1, (void *)2};
static struct { uint8_t tx, rx; } wire[10000];
static unsigned head, tail, aborts, dma_mode, dma_calls;
static uint32_t ticks, ipsr;
static int cs = 1;
static int slow_expected;
static void byte(uint8_t tx, uint8_t rx) { assert(tail < 10000); wire[tail].tx = tx; wire[tail++].rx = rx; }
static void idle(void) { byte(255, 255); }
static void cmd(uint8_t command, uint32_t arg, uint8_t response)
{
    if (command != 12) { idle(); idle(); idle(); }
    byte(0x40 | command, 255);
    byte(arg >> 24, 255); byte(arg >> 16, 255); byte(arg >> 8, 255); byte(arg, 255);
    byte(command == 0 ? 0x95 : command == 8 ? 0x87 : 1, 255);
    if (command == 12) byte(255, 0x7F); /* Stuff byte must be ignored. */
    byte(255, response);
}
static void payload(const uint8_t *data, unsigned len)
{
    byte(255, 0xFE);
    for (unsigned i = 0; i < len; ++i) byte(255, data[i]);
    idle(); idle();
}
static void consumed(void) { assert(head == tail); assert(cs == 1); head = tail = 0; }
uint32_t __get_IPSR(void) { return ipsr; }
uint32_t __get_PRIMASK(void) { return 0; }
uint32_t HAL_GetTick(void) { return ticks++; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 16000000; }
void HAL_Delay(uint32_t ms) { ticks += ms; }
void HAL_GPIO_WritePin(void *port, uint16_t pin, int value)
{
    assert(port == CS_GPIO_Port && pin == CS_Pin); cs = value;
}
int HAL_SPI_GetState(SPI_HandleTypeDef *spi) { assert(spi == &hspi3); return HAL_SPI_STATE_READY; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi3 && timeout != UINT32_MAX);
    if (slow_expected)
        assert(16000000U / (2U << (hspi3.Init.BaudRatePrescaler / SPI_CR1_BR_0)) <= 400000U);
    for (unsigned i = 0; i < size; ++i) {
        assert(head < tail);
        if (tx[i] != wire[head].tx) {
            fprintf(stderr, "wire mismatch at %u: got %02x expected %02x\n", head, tx[i], wire[head].tx);
            assert(0);
        }
        rx[i] = wire[head++].rx;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *tx,
                                             uint8_t *rx, uint16_t size)
{
    assert(cs == 0 && size <= 512);
    ++dma_calls;
    if (dma_mode == 1) return HAL_ERROR;
    if (dma_mode == 2) return HAL_OK; /* No interrupt: must time out. */
    if (dma_mode == 3) { HAL_SPI_ErrorCallback(spi); return HAL_OK; }
    assert(HAL_SPI_TransmitReceive(spi, tx, rx, size, 100) == HAL_OK);
    HAL_SPI_TxRxCpltCallback(spi);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi) { assert(spi == &hspi3); ++aborts; return HAL_OK; }

static void initialize(int hc)
{
    dma_mode = 0;
    for (unsigned i = 0; i < 10; ++i) idle();
    cmd(0, 0, 1);
    cmd(8, 0x1AA, hc ? 1 : 5);
    if (hc) { byte(255, 0); byte(255, 0); byte(255, 1); byte(255, 0xAA); }
    cmd(55, 0, 1); cmd(41, hc ? 0x40000000U : 0, 0);
    cmd(58, 0, 0);
    byte(255, hc ? 0xC0 : 0x80); byte(255, 0x30); byte(255, 0); byte(255, 0);
    if (!hc) cmd(16, 512, 0);
    uint8_t csd[16] = {0};
    if (hc) { csd[0] = 0x40; csd[8] = 0x1F; csd[9] = 0xFF; }
    else { csd[5] = 9; csd[6] = 3; csd[7] = 255; csd[8] = 0xC0; }
    cmd(9, 0, 0); payload(csd, 16); idle();
    slow_expected = 1;
    assert(SD_SPI_Init() == SD_OK);
    slow_expected = 0;
    assert(sd_is_sdhc() == hc);
    assert(SD_GetSectorCount() == (hc ? 8388608U : 16384U));
    assert(hspi3.Init.BaudRatePrescaler == 0); /* 16MHz / 2 */
    consumed();
}
int main(void)
{
    uint8_t data[1024], received[1024];
    for (unsigned i = 0; i < sizeof(data); ++i) data[i] = i * 37U;
    assert(SD_disk_status(0) == STA_NOINIT);
    assert(SD_disk_read(0, received, 0, 1) == RES_NOTRDY);
    initialize(1);
    DWORD capacity = 0;
    assert(SD_disk_ioctl(0, GET_SECTOR_COUNT, &capacity) == RES_OK && capacity == 8388608U);
    assert(SD_disk_ioctl(0, GET_SECTOR_SIZE, NULL) == RES_PARERR);
    cmd(18, 7, 0); payload(data, 512); payload(data + 512, 512);
    cmd(12, 0, 0); byte(255, 0); idle(); idle();
    assert(SD_ReadBlocks(received, 7, 2) == SD_OK);
    assert(!memcmp(data, received, sizeof(data))); consumed();
    cmd(25, 9, 0);
    for (unsigned block = 0; block < 2; ++block) {
        idle(); byte(0xFC, 255);
        for (unsigned i = 0; i < 512; ++i) byte(data[block * 512 + i], 255);
        idle(); idle(); byte(255, 5); byte(255, 0); idle();
    }
    idle(); byte(0xFD, 255); byte(255, 0); idle();
    cmd(13, 0, 0); byte(255, 0); idle();
    assert(SD_WriteBlocks(data, 9, 2) == SD_OK); consumed();
    /* Stop an active multi-read even after a card error token. */
    cmd(18, 1, 0); byte(255, 8); cmd(12, 0, 0); idle(); idle();
    assert(SD_ReadBlocks(received, 1, 2) == SD_ERROR); consumed();
    initialize(1);
    /* Rejected multi-write must still issue the stop token and report failure. */
    cmd(25, 1, 0); idle(); byte(0xFC, 255);
    for (unsigned i = 0; i < 512; ++i) byte(data[i], 255);
    idle(); idle(); byte(255, 0x0B); byte(255, 0); idle();
    idle(); byte(0xFD, 255); byte(255, 0); idle(); idle();
    assert(SD_WriteBlocks(data, 1, 2) == SD_ERROR); consumed();
    initialize(1);
    assert(SD_ReadBlocks(NULL, 0, 1) == SD_ERROR);
    assert(SD_disk_read(0, data, capacity - 1, 2) == RES_PARERR);
    ipsr = 1; assert(SD_ReadBlocks(data, 0, 1) == SD_ERROR); ipsr = 0;
    for (unsigned fault = 1; fault <= 3; ++fault) {
        if (fault != 1) initialize(1);
        cmd(17, 0, 0); byte(255, 0xFE);
        dma_mode = fault;
        ticks = UINT32_MAX - 20; /* Timeout must survive tick rollover. */
        unsigned before = aborts;
        assert(SD_ReadBlocks(data, 0, 1) == SD_ERROR);
        assert(aborts == before + 1 && !card_initialized); consumed();
    }
    initialize(0);
    cmd(17, 7 * 512, 0); payload(data, 512); idle();
    assert(SD_ReadBlocks(received, 7, 1) == SD_OK); consumed();
    /* No token must time out, deassert CS, and invalidate the card. */
    cmd(17, 0, 0);
    for (unsigned i = 0; i < 1000; ++i) idle();
    idle();
    assert(SD_ReadBlocks(data, 0, 1) == SD_ERROR); consumed();
    assert(dma_calls >= 10);
    puts("SPI tests passed: SDSC/SDHC, addressing, capacity, multi-block stop, DMA faults, rollover, token timeout");
    return 0;
}
