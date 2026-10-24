/* Scripted card responses test the polling transport without physical media. */
#include "sd_spi.h"
#include "sd_diskio_spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
SPI_TypeDef mock_spi;
SPI_HandleTypeDef hspi3 = {&mock_spi, {0}, NULL, NULL}; /* No DMA handles required. */
static struct { uint8_t tx, rx; } wire[10000];
static unsigned head, tail, calls, fail_at;
static uint32_t ticks, ipsr;
static int cs = 1, slow_expected, busy;
static void byte(uint8_t tx, uint8_t rx) { CHECK(tail < 10000); wire[tail].tx = tx; wire[tail++].rx = rx; }
static void idle(void) { byte(255, 255); }
static void cmd(uint8_t c, uint32_t arg, uint8_t response) {
    /* Before CMD0, DO is not assumed to provide SPI-ready. Regression:
     * return low on the selected dummy byte; CMD0 must still be sent. */
    if (c == 0) byte(255, 0);
    else idle();
    byte(0x40 | c, 255);
    byte(arg >> 24, 255); byte(arg >> 16, 255); byte(arg >> 8, 255); byte(arg, 255);
    byte(c == 0 ? 0x95 : c == 8 ? 0x87 : 255, 255);
    byte(255, response);
}
static void payload(const uint8_t *data, unsigned n) {
    byte(255, 0xFE);
    while (n--) byte(255, *data++);
    idle(); idle();
}
static void consumed(void) { CHECK(head == tail && cs == 1); head = tail = 0; }
uint32_t __get_IPSR(void) { return ipsr; }
uint32_t __get_PRIMASK(void) { return 0; }
uint32_t HAL_GetTick(void) { return ticks++; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 16000000; }
void HAL_Delay(uint32_t ms) { ticks += ms; }
void HAL_GPIO_WritePin(void *p, uint16_t pin, int state) { CHECK(p == CS_GPIO_Port && pin == CS_Pin); cs = state; }
int HAL_SPI_GetState(SPI_HandleTypeDef *spi) { CHECK(spi == &hspi3); return HAL_SPI_STATE_READY; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout) {
    CHECK(spi == &hspi3 && timeout == 100 && n == 1);
    if (slow_expected) CHECK(16000000U / (2U << (hspi3.Init.BaudRatePrescaler / SPI_CR1_BR_0)) <= 400000U);
    if (++calls == fail_at) return HAL_TIMEOUT;
    if (busy && head == tail) { *rx = 0; return HAL_OK; }
    CHECK(head < tail);
    CHECK(*tx == wire[head].tx);
    *rx = wire[head++].rx;
    return HAL_OK;
}
/* Any accidental return to DMA causes an explicit test failure. */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *p, uint8_t *a, uint8_t *b, uint16_t n) {
    (void)p; (void)a; (void)b; (void)n; CHECK(0); return HAL_ERROR;
}
static void initialize(int hc) {
    for (unsigned i = 0; i < 10; ++i) byte(255, 0);
    cmd(0, 0, 1); idle();
    cmd(8, 0x1AA, hc ? 1 : 5);
    if (hc) { byte(255,0); byte(255,0); byte(255,1); byte(255,0xAA); }
    idle();
    cmd(55, 0, 1); cmd(41, hc ? 0x40000000U : 0, 0); idle();
    cmd(58, 0, 0);
    byte(255,hc ? 0xC0 : 0x80); byte(255,0x30); byte(255,0); byte(255,0); idle();
    if (!hc) { cmd(16, 512, 0); idle(); }
    uint8_t csd[16] = {0};
    if (hc) { csd[0] = 0x40; csd[8] = 0x1F; csd[9] = 0xFF; }
    else { csd[5] = 9; csd[6] = 3; csd[7] = 255; csd[8] = 0xC0; }
    cmd(9, 0, 0); payload(csd, 16); idle();
    slow_expected = 1;
    CHECK(SD_SPI_Init() == SD_OK);
    CHECK(sd_spi_diagnostics.init_stage == SD_INIT_READY);
    slow_expected = 0;
    CHECK(sd_is_sdhc() == hc);
    CHECK(SD_GetSectorCount() == (hc ? 8388608U : 16384U));
    CHECK(hspi3.Init.BaudRatePrescaler == 0);
    consumed();
}
static void read_block(const uint8_t *data, uint32_t address) {
    cmd(17, address, 0); payload(data, 512); idle();
}
static void write_block(const uint8_t *data, uint32_t address) {
    cmd(24, address, 0); byte(0xFE,255);
    for (unsigned i = 0; i < 512; ++i) byte(data[i],255);
    idle(); idle(); byte(255,5);
    idle(); byte(255,0); idle(); /* gap, busy, ready */
    idle();
}
int main(void) {
    uint8_t sent[1024], got[1024];
    for (unsigned i = 0; i < sizeof(sent); ++i) sent[i] = (uint8_t)(i * 37);
    CHECK(SD_disk_status(0) == STA_NOINIT);
    CHECK(SD_disk_ioctl(0, CTRL_SYNC, NULL) == RES_NOTRDY);
    initialize(1);
    read_block(sent, 10); read_block(sent + 512, 11);
    CHECK(SD_ReadBlocks(got, 10, 2) == SD_OK && !memcmp(sent,got,sizeof(sent))); consumed();
    write_block(sent, 12); write_block(sent + 512, 13);
    CHECK(SD_WriteBlocks(sent, 12, 2) == SD_OK); consumed();
    idle(); idle(); CHECK(SD_disk_ioctl(0,CTRL_SYNC,NULL) == RES_OK); consumed();
    DWORD count; CHECK(SD_disk_ioctl(0,GET_SECTOR_COUNT,&count) == RES_OK && count == 8388608U);
    CHECK(SD_disk_ioctl(0,GET_SECTOR_COUNT,NULL) == RES_PARERR);
    CHECK(SD_ReadBlocks(got, count - 1, 2) == SD_ERROR);
    CHECK(SD_ReadBlocks(NULL, 0, 1) == SD_ERROR);
    initialize(0);
    read_block(sent, 3 * 512); CHECK(SD_ReadBlocks(got, 3, 1) == SD_OK); consumed();
    write_block(sent, 4 * 512); CHECK(SD_WriteBlocks(sent, 4, 1) == SD_OK); consumed();
    /* HAL timeout during payload must not produce a successful block read. */
    cmd(17, 0, 0); byte(255,0xFE);
    fail_at = calls + 10; /* ready + six command bytes + R1 + token, then payload */
    CHECK(SD_ReadBlocks(got,0,1) == SD_ERROR && !card_initialized); consumed();
    fail_at = 0; initialize(1);
    /* Card stuck busy: finite wait across tick rollover, CS released. */
    busy = 1; ticks = UINT32_MAX - 100U;
    CHECK(SD_Sync() == SD_ERROR && !card_initialized && cs == 1);
    busy = 0; consumed();
    ipsr = 1; CHECK(SD_SPI_Init() == SD_ERROR); ipsr = 0;
    puts("Polling SPI tests passed: SDHC/SDSC, real capacity, repeated sectors, read/write, HAL errors, busy timeout, rollover, no DMA.");
    return 0;
}
