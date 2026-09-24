#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SDCARD_IDLE = 0,
    SDCARD_BUSY,
    SDCARD_ERROR
} SDCardStatus;


bool SDCard_Init(void);

bool SDCard_CreateFile(const char *filename);

bool SDCard_Write(const uint8_t *data,
                  uint32_t length);

bool SDCard_WriteString(const char *data);

bool SDCard_IsReady(void);

bool SDCard_IsBusy(void);

SDCardStatus SDCard_GetStatus(void);


/* DMA completion event */

void SDCard_DMACompleteCallback(void);

void SDCard_DMAErrorCallback(void);

#endif