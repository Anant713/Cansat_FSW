/* Adapted from ControllersTech sd_benchmark.c, June 26, 2025.
 * https://controllerstech.com -- original permitted learning/development use. */
#include "sd_benchmark.h"
#include "sd_functions.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>

#define TEST_SIZE 512000U
#if SD_SPI_USE_DMA
#define MODE_STR "DMA"
#else
#define MODE_STR "Polling"
#endif

static uint32_t benchmark_write(const char *name)
{
    FIL file;
    uint8_t buffer[512];
    memset(buffer, 0xAA, sizeof(buffer));
    FRESULT result = f_open(&file, name, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK) {
        printf("Benchmark create failed: %d (use a new name or remove old bench.bin)\r\n", result);
        return 0;
    }
    uint32_t start = HAL_GetTick(), remaining = TEST_SIZE;
    while (remaining) {
        UINT written = 0;
        UINT amount = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;
        result = f_write(&file, buffer, amount, &written);
        if (result != FR_OK || written != amount) break;
        remaining -= written;
    }
    FRESULT closed = f_close(&file); /* Include metadata flush in measured time. */
    if (result != FR_OK || closed != FR_OK || remaining) return 0;
    uint32_t elapsed = HAL_GetTick() - start;
    return elapsed ? elapsed : 1;
}
static uint32_t benchmark_read(const char *name)
{
    FIL file;
    uint8_t buffer[512];
    FRESULT result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return 0;
    uint32_t start = HAL_GetTick(), remaining = TEST_SIZE;
    while (remaining) {
        UINT received = 0;
        UINT amount = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;
        result = f_read(&file, buffer, amount, &received);
        if (result != FR_OK || received != amount) break;
        for (UINT i = 0; i < received; ++i) {
            if (buffer[i] != 0xAA) { result = FR_DISK_ERR; break; }
        }
        if (result != FR_OK) break;
        remaining -= received;
    }
    FRESULT closed = f_close(&file);
    if (result != FR_OK || closed != FR_OK || remaining) return 0;
    uint32_t elapsed = HAL_GetTick() - start;
    return elapsed ? elapsed : 1;
}
void sd_benchmark(void)
{
    if (sd_mount() != FR_OK) { printf("Benchmark mount failed\r\n"); return; }
    uint32_t write_ms = benchmark_write("0:/bench.bin");
    uint32_t read_ms = write_ms ? benchmark_read("0:/bench.bin") : 0;
    if (write_ms && read_ms) {
        printf("[%s] Verified %lu bytes; write %lu KiB/s; read %lu KiB/s\r\n",
               MODE_STR, (unsigned long)TEST_SIZE,
               (unsigned long)((TEST_SIZE / 1024U * 1000U) / write_ms),
               (unsigned long)((TEST_SIZE / 1024U * 1000U) / read_ms));
    } else printf("Benchmark FAILED (no throughput result)\r\n");
    (void)sd_unmount();
}
