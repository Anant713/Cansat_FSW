/******************************************************************************
 *  File        : sd_spi.h (SDSC/SDHC support)
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/

#ifndef __SD_SPI_H__
#define __SD_SPI_H__

#include "main.h"  // or your specific STM32 family header
#include <stdint.h>
#define SD_BLOCK_SIZE 512U

#define CMD0  (0)
#define CMD8  (8)
#define CMD17 (17)
#define CMD24 (24)
#define CMD55 (55)
#define CMD58 (58)
#define ACMD41 (41)

typedef enum {
    SD_OK = 0,
    SD_ERROR
} SD_Status;

extern uint8_t card_initialized;

typedef enum {
    SD_INIT_NOT_STARTED = 0, SD_INIT_CHECKS, SD_INIT_CLOCKS, SD_INIT_CMD0,
    SD_INIT_CMD8, SD_INIT_ACMD41, SD_INIT_OCR, SD_INIT_BLOCK_LENGTH,
    SD_INIT_CSD, SD_INIT_RUN_SPEED, SD_INIT_READY
} SD_InitStage;
typedef struct {
    SD_InitStage init_stage; /* Remains at the failing stage after init returns. */
    uint8_t command, response, token;
    uint8_t command_sent; /* 0 if wait-ready failed before transmitting command. */
    HAL_StatusTypeDef hal_status; /* First HAL failure since initialization. */
    uint32_t r7, ocr;
} SDSPIDiagnostics;
extern volatile SDSPIDiagnostics sd_spi_diagnostics;

SD_Status SD_SPI_Init(void);
SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t sd_is_sdhc(void);
uint32_t SD_GetSectorCount(void);
SD_Status SD_Sync(void);

/* Polling only, single caller in main context with HAL tick/interrupts active. */

#endif // __SD_SPI_H__
