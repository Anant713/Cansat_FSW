#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>
#include "ff.h"

typedef enum
{
    SDCARD_IDLE = 0,
    SDCARD_BUSY,
    SDCARD_ERROR
} SDCardStatus;


bool SDCard_Init(void);

bool SDCard_CreateFile(const char *filename);
/* CreateFile refuses to overwrite. OpenAppend preserves an existing file. */
bool SDCard_OpenAppend(const char *filename);

bool SDCard_Write(const uint8_t *data,
                  uint32_t length);

bool SDCard_WriteString(const char *data);
bool SDCard_ReadFile(const char *filename, uint8_t *data, uint32_t capacity,
                     uint32_t *bytes_read);
bool SDCard_Sync(void);
bool SDCard_CloseFile(void);
bool SDCard_DeInit(void);
FRESULT SDCard_GetLastResult(void);

bool SDCard_IsReady(void);

bool SDCard_IsBusy(void);

SDCardStatus SDCard_GetStatus(void);


/* All operations are blocking, single-caller and main/thread-context only.
 * SPI3 uses polling only. Call Sync periodically and CloseFile before removal.
 * A successful Write can still leave data in FatFs's sector cache. */

#endif
