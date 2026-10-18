/* Adapted from ControllersTech sd_diskio_spi.c, June 26, 2025.
 * https://controllerstech.com -- original permitted learning/development use. */
#include "sd_diskio_spi.h"
#include "sd_spi.h"

DSTATUS SD_disk_status(BYTE drv)
{
    return drv == 0 && card_initialized ? 0 : STA_NOINIT;
}
DSTATUS SD_disk_initialize(BYTE drv)
{
    return drv == 0 && SD_SPI_Init() == SD_OK ? 0 : STA_NOINIT;
}
DRESULT SD_disk_read(BYTE drv, BYTE *buffer, DWORD sector, UINT count)
{
    if (drv || !buffer || !count) return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    if (sector >= SD_GetSectorCount() || count > SD_GetSectorCount() - sector)
        return RES_PARERR;
    return SD_ReadBlocks(buffer, sector, count) == SD_OK ? RES_OK : RES_ERROR;
}
DRESULT SD_disk_write(BYTE drv, const BYTE *buffer, DWORD sector, UINT count)
{
    if (drv || !buffer || !count) return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    if (sector >= SD_GetSectorCount() || count > SD_GetSectorCount() - sector)
        return RES_PARERR;
    return SD_WriteBlocks(buffer, sector, count) == SD_OK ? RES_OK : RES_ERROR;
}
DRESULT SD_disk_ioctl(BYTE drv, BYTE cmd, void *buffer)
{
    if (drv) return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    if (cmd == CTRL_SYNC) return SD_Sync() == SD_OK ? RES_OK : RES_ERROR;
    if (!buffer) return RES_PARERR;
    switch (cmd) {
    case GET_SECTOR_SIZE:
        *(WORD *)buffer = SD_BLOCK_SIZE;
        return RES_OK;
    case GET_SECTOR_COUNT:
        *(DWORD *)buffer = SD_GetSectorCount();
        return RES_OK;
    case GET_BLOCK_SIZE:
        /* Unknown erase geometry: FatFs permits 1 as the fallback. */
        *(DWORD *)buffer = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}
/* Kept for compatibility. The project registers USER_Driver, whose callbacks
 * forward here; do not register this second table as another volume. */
const Diskio_drvTypeDef SD_Driver = {
    SD_disk_initialize, SD_disk_status, SD_disk_read,
#if _USE_WRITE
    SD_disk_write,
#endif
#if _USE_IOCTL
    SD_disk_ioctl,
#endif
};
