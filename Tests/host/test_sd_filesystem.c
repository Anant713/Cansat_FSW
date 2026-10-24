/* Exercises the real bundled FatFs, direct SD driver, helpers and SDCard facade
 * against an in-memory block device. No host disk/card is formatted. */
#include "drivers/SDCard.h"
#include "sd_functions.h"
#include "sd_spi.h"
#include "sd_diskio_spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define SECTORS 32768U
static uint8_t medium[SECTORS][512];
uint8_t card_initialized;
static unsigned initializations;
static int fail_sync;
SD_Status SD_SPI_Init(void) { ++initializations; card_initialized = 1; return SD_OK; }
uint32_t SD_GetSectorCount(void) { return SECTORS; }
uint8_t sd_is_sdhc(void) { return 0; }
SD_Status SD_ReadBlocks(uint8_t *data, uint32_t sector, uint32_t count)
{
    assert(sector < SECTORS && count <= SECTORS - sector);
    memcpy(data, medium[sector], count * 512U); return SD_OK;
}
SD_Status SD_WriteBlocks(const uint8_t *data, uint32_t sector, uint32_t count)
{
    assert(sector < SECTORS && count <= SECTORS - sector);
    memcpy(medium[sector], data, count * 512U); return SD_OK;
}
SD_Status SD_Sync(void) { return fail_sync ? SD_ERROR : SD_OK; }
int main(void)
{
    uint8_t work[512], sent[1537], readback[1600];
    for (unsigned i = 0; i < sizeof(sent); ++i) sent[i] = (uint8_t)(i * 37U + i / 251U);
    assert(sd_mount() == FR_NO_FILESYSTEM);
    assert(FATFS_GetAttachedDriversNbr() == 1);
    extern Disk_drvTypeDef disk;
    assert(disk.drv[0] == &SD_Driver);
    assert(f_mkfs(sd_path, FM_FAT | FM_SFD, 0, work, sizeof(work)) == FR_OK);
    assert(SDCard_Init());
    unsigned before = initializations;
    assert(SDCard_Init() && initializations == before);
    assert(SDCard_CreateFile("0:/telemetry long filename.bin"));
    assert(SDCard_Write(sent, sizeof(sent)) && SDCard_Sync() && SDCard_CloseFile());
    assert(!SDCard_CreateFile("0:/telemetry long filename.bin"));
    assert(SDCard_GetLastResult() == FR_EXIST);
    assert(SDCard_OpenAppend("0:/telemetry long filename.bin"));
    assert(SDCard_WriteString("APPEND") && SDCard_CloseFile());
    assert(SDCard_DeInit() && SDCard_Init());
    assert(FATFS_GetAttachedDriversNbr() == 1);
    uint32_t received;
    assert(SDCard_ReadFile("0:/telemetry long filename.bin", readback, sizeof(readback), &received));
    assert(received == sizeof(sent) + 6 && !memcmp(readback, sent, sizeof(sent)));
    assert(!memcmp(readback + sizeof(sent), "APPEND", 6));
    char text[32]; UINT length;
    assert(sd_read_file("missing", text, 0, &length) == FR_INVALID_PARAMETER);
    assert(sd_read_file("missing", text, sizeof(text), NULL) == FR_INVALID_PARAMETER);
    assert(sd_write_file("text.txt", "hello") == FR_OK);
    assert(sd_append_file("text.txt", " world") == FR_OK);
    assert(sd_read_file("text.txt", text, sizeof(text), &length) == FR_OK);
    assert(length == 11 && !strcmp(text, "hello world"));
    assert(sd_write_file("records.csv", "one,two,42\nthree,four,-1\n") == FR_OK);
    CsvRecord records[2]; int count;
    assert(sd_read_csv("records.csv", records, 2, &count) == FR_OK && count == 2);
    assert(records[0].value == 42 && records[1].value == -1);
    assert(sd_write_file("bad.csv", "1234567890123456789012345678901234,b,1\n") == FR_OK);
    assert(sd_read_csv("bad.csv", records, 2, &count) == FR_INVALID_PARAMETER);
    /* A metadata sync/close failure must not be reported as a successful write. */
    fail_sync = 1;
    assert(sd_write_file("fail.txt", "not durable") == FR_DISK_ERR);
    fail_sync = 0;
    assert(SDCard_DeInit() && SDCard_Init());
    assert(SDCard_CreateFile("recovery.bin"));
    assert(SDCard_Write(sent, sizeof(sent)));
    card_initialized = 0;
    assert(!SDCard_Sync() && SDCard_GetLastResult() == FR_NOT_READY);
    assert(!SDCard_DeInit()); /* Reports loss, but releases stale file and mount. */
    assert(SDCard_Init() && SDCard_CreateFile("after.bin") && SDCard_CloseFile());
    assert(SDCard_DeInit());
    puts("Filesystem tests passed: create, append, binary readback, remount, long filenames, CSV, invalid buffers, sync failures, recovery");
    return 0;
}
