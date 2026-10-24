/******************************************************************************
 *  File        : sd_spi.c (SDSC/SDHC support)
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

#include "sd_spi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * 🔧 USER-MODIFIABLE SECTION
 * You are free to edit anything below this line
 ***************************************************************/

#define USE_DMA 0

extern SPI_HandleTypeDef hspi3;
#define SD_SPI_HANDLE hspi3

#define SD_CS_LOW()     HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

/***************************************************************
 * 🚫 DO NOT MODIFY BELOW THIS LINE
 * Auto-generated/system-managed code. Changes may be lost.
 ***************************************************************/

/* Polling only. HAL failures are latched until the operation returns. */
static uint8_t io_failed;
static uint32_t sector_count;
volatile SDSPIDiagnostics sd_spi_diagnostics;
#define SD_IO_TIMEOUT_MS 100U

static uint8_t SD_Exchange(uint8_t tx) {
    uint8_t rx = 0xFF;
    if (!io_failed) {
        HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &tx, &rx, 1, SD_IO_TIMEOUT_MS);
        if (result != HAL_OK) {
            if (sd_spi_diagnostics.hal_status == HAL_OK) sd_spi_diagnostics.hal_status = result;
            io_failed = 1;
        }
    }
    return rx;
}
static void SD_TransmitByte(uint8_t data) { (void)SD_Exchange(data); }
static uint8_t SD_ReceiveByte(void) { return SD_Exchange(0xFF); }
static void SD_TransmitBuffer(const uint8_t *buffer, uint16_t len) {
    while (len-- && !io_failed) SD_TransmitByte(*buffer++);
}
static void SD_ReceiveBuffer(uint8_t *buffer, uint16_t len) {
    while (len-- && !io_failed) *buffer++ = SD_ReceiveByte();
}
static SD_Status SD_Finish(SD_Status result) {
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    if (io_failed || result != SD_OK) {
        card_initialized = 0;
        return SD_ERROR;
    }
    return SD_OK;
}
static int SD_SetSpeed(uint32_t maximum_hz) {
    uint32_t clock = HAL_RCC_GetPCLK1Freq(), divisor = 2, bits = 0;
    while (clock / divisor > maximum_hz && divisor < 256) {
        divisor <<= 1; bits += SPI_CR1_BR_0;
    }
    if (clock / divisor > maximum_hz) return 0;
    __HAL_SPI_DISABLE(&SD_SPI_HANDLE);
    MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_CR1_BR, bits);
    SD_SPI_HANDLE.Init.BaudRatePrescaler = bits;
    __HAL_SPI_ENABLE(&SD_SPI_HANDLE);
    return 1;
}
static SD_Status SD_WaitReady(void) {
    uint32_t started = HAL_GetTick();
    uint8_t resp;
    do {
        resp = SD_ReceiveByte();
        if (resp == 0xFF && !io_failed) return SD_OK;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < 500U);
    return SD_ERROR;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response, retry = 0xFF;

    sd_spi_diagnostics.command = cmd;
    sd_spi_diagnostics.response = 0xFF;
    sd_spi_diagnostics.command_sent = 0;

    /* CMD0 enters SPI mode from native SD mode. Do not require the SPI
     * ready indication before issuing the command that establishes it.
     * Still provide eight clocks after selecting the card. */
    if (cmd == CMD0) (void)SD_ReceiveByte();
    else if (SD_WaitReady() != SD_OK) return 0xFF;
    if (io_failed) return 0xFF;
    sd_spi_diagnostics.command_sent = 1;
    SD_TransmitByte(0x40 | cmd);
    SD_TransmitByte(arg >> 24);
    SD_TransmitByte(arg >> 16);
    SD_TransmitByte(arg >> 8);
    SD_TransmitByte(arg);
    SD_TransmitByte(crc);

    do {
        response = SD_ReceiveByte();
    } while ((response & 0x80) && --retry);

    sd_spi_diagnostics.response = io_failed ? 0xFF : response;
    return sd_spi_diagnostics.response;
}

static uint8_t sdhc = 0;
uint8_t sd_is_sdhc(void) {
    return sdhc;
}
uint8_t card_initialized = 0;

static SD_Status SD_ReadCapacity(void) {
    uint8_t csd[16], token;
    SD_CS_LOW();
    if (SD_SendCommand(9, 0, 0xFF) != 0) return SD_Finish(SD_ERROR);
    uint32_t started = HAL_GetTick();
    do {
        token = SD_ReceiveByte();
        if (token != 0xFF) break;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < 500U);
    sd_spi_diagnostics.token = token;
    if (token != 0xFE) return SD_Finish(SD_ERROR);
    SD_ReceiveBuffer(csd, sizeof(csd));
    SD_ReceiveByte(); SD_ReceiveByte();
    if (io_failed) return SD_Finish(SD_ERROR);
    uint64_t sectors;
    if ((csd[0] >> 6) == 1) {
        uint32_t size = ((uint32_t)(csd[7] & 63) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        sectors = ((uint64_t)size + 1) * 1024;
    } else if ((csd[0] >> 6) == 0) {
        uint32_t size = ((uint32_t)(csd[6] & 3) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        unsigned shift = ((csd[9] & 3) << 1) | (csd[10] >> 7);
        sectors = (((uint64_t)size + 1) << (shift + 2 + (csd[5] & 15))) / 512;
    } else return SD_Finish(SD_ERROR);
    if (!sectors || sectors > UINT32_MAX) return SD_Finish(SD_ERROR);
    sector_count = (uint32_t)sectors;
    return SD_Finish(SD_OK);
}

SD_Status SD_SPI_Init(void) {
    uint8_t response, r7[4] = {0};
    sd_spi_diagnostics = (SDSPIDiagnostics){.init_stage = SD_INIT_CHECKS,
        .command = 0xFF, .response = 0xFF, .token = 0xFF, .hal_status = HAL_OK};
    card_initialized = 0; sector_count = 0; sdhc = 0; io_failed = 0;
    if (__get_IPSR() || __get_PRIMASK() || SD_SPI_HANDLE.Instance != SPI3 ||
        HAL_SPI_GetState(&SD_SPI_HANDLE) != HAL_SPI_STATE_READY) return SD_ERROR;
    SD_CS_HIGH();
    sd_spi_diagnostics.init_stage = SD_INIT_CLOCKS;
    if (!SD_SetSpeed(400000U)) return SD_ERROR;
    HAL_Delay(2);
    for (unsigned i = 0; i < 10; ++i) SD_TransmitByte(0xFF);
    uint32_t started = HAL_GetTick();
    sd_spi_diagnostics.init_stage = SD_INIT_CMD0;
    do {
        SD_CS_LOW();
        response = SD_SendCommand(CMD0, 0, 0x95);
        SD_CS_HIGH(); SD_TransmitByte(0xFF);
        if (response == 1) break;
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < 2000U);
    if (response != 1) return SD_Finish(SD_ERROR);

    SD_CS_LOW();
    sd_spi_diagnostics.init_stage = SD_INIT_CMD8;
    response = SD_SendCommand(CMD8, 0x1AA, 0x87);
    int version2 = response == 1;
    if (version2) {
        SD_ReceiveBuffer(r7, sizeof(r7));
        sd_spi_diagnostics.r7 = ((uint32_t)r7[0] << 24) | ((uint32_t)r7[1] << 16) |
                               ((uint32_t)r7[2] << 8) | r7[3];
        if (r7[2] != 1 || r7[3] != 0xAA) return SD_Finish(SD_ERROR);
    } else if (response != 5) return SD_Finish(SD_ERROR);
    SD_CS_HIGH(); SD_TransmitByte(0xFF);
    started = HAL_GetTick();
    sd_spi_diagnostics.init_stage = SD_INIT_ACMD41;
    do {
        SD_CS_LOW();
        if (SD_SendCommand(CMD55, 0, 0xFF) > 1) return SD_Finish(SD_ERROR);
        response = SD_SendCommand(ACMD41, version2 ? 0x40000000U : 0, 0xFF);
        SD_CS_HIGH(); SD_TransmitByte(0xFF);
        if (response == 0) break;
        if (response != 1) return SD_Finish(SD_ERROR);
    } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < 2000U);
    if (response != 0) return SD_Finish(SD_ERROR);
    SD_CS_LOW();
    sd_spi_diagnostics.init_stage = SD_INIT_OCR;
    if (SD_SendCommand(CMD58, 0, 0xFF) != 0) return SD_Finish(SD_ERROR);
    uint8_t ocr[4] = {0};
    SD_ReceiveBuffer(ocr, sizeof(ocr));
    sd_spi_diagnostics.ocr = ((uint32_t)ocr[0] << 24) | ((uint32_t)ocr[1] << 16) |
                            ((uint32_t)ocr[2] << 8) | ocr[3];
    if (!(ocr[0] & 0x80) || !(ocr[1] & 0x30)) return SD_Finish(SD_ERROR);
    sdhc = version2 && (ocr[0] & 0x40);
    SD_CS_HIGH(); SD_TransmitByte(0xFF);
    if (!sdhc) {
        sd_spi_diagnostics.init_stage = SD_INIT_BLOCK_LENGTH;
        SD_CS_LOW();
        if (SD_SendCommand(16, 512, 0xFF) != 0) return SD_Finish(SD_ERROR);
        SD_CS_HIGH(); SD_TransmitByte(0xFF);
    }
    sd_spi_diagnostics.init_stage = SD_INIT_CSD;
    if (SD_ReadCapacity() != SD_OK || io_failed) return SD_Finish(SD_ERROR);
    sd_spi_diagnostics.init_stage = SD_INIT_RUN_SPEED;
    if (!SD_SetSpeed(8000000U))
        return SD_Finish(SD_ERROR);
    card_initialized = 1;
    sd_spi_diagnostics.init_stage = SD_INIT_READY;
    return SD_OK;
}
uint32_t SD_GetSectorCount(void) { return card_initialized ? sector_count : 0; }
SD_Status SD_Sync(void) {
    if (!card_initialized || __get_IPSR() || __get_PRIMASK()) return SD_ERROR;
    io_failed = 0;
    SD_CS_LOW();
    return SD_Finish(SD_WaitReady());
}
static int SD_ValidRequest(const void *buff, uint32_t sector, uint32_t count) {
    return buff && count && card_initialized && !__get_IPSR() && !__get_PRIMASK() &&
        sector < sector_count && count <= sector_count - sector &&
        count <= UINT32_MAX / 512U && (sdhc || sector + count - 1 <= UINT32_MAX / 512U);
}
SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!SD_ValidRequest(buff, sector, count)) return SD_ERROR;
    io_failed = 0;

    if (count == 1) {
        if (!sdhc) sector *= 512;
        SD_CS_LOW();
        if (SD_SendCommand(CMD17, sector, 0xFF) != 0x00) {
            return SD_Finish(SD_ERROR);
        }

        uint8_t token;
        uint32_t started = HAL_GetTick();
        do {
            token = SD_ReceiveByte();
            if (token == 0xFE) break;
        } while (!io_failed && (uint32_t)(HAL_GetTick() - started) < 500U);
        if (token != 0xFE) {
            return SD_Finish(SD_ERROR);
        }

        SD_ReceiveBuffer(buff, 512);
        SD_ReceiveByte();  // CRC
        SD_ReceiveByte();
        return SD_Finish(SD_OK);
    } else {
        return SD_ReadMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!SD_ValidRequest(buff, sector, count)) return SD_ERROR;
    while (count--) {
        if (SD_ReadBlocks(buff, sector++, 1) != SD_OK) return SD_ERROR;
        buff += 512;
    }
    return SD_OK;
}
SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!SD_ValidRequest(buff, sector, count)) return SD_ERROR;
    io_failed = 0;

    if (count == 1) {
        if (!sdhc) sector *= 512;
        SD_CS_LOW();
        if (SD_SendCommand(CMD24, sector, 0xFF) != 0x00) {
            return SD_Finish(SD_ERROR);
        }

        SD_TransmitByte(0xFE);
        SD_TransmitBuffer(buff, 512);
        SD_TransmitByte(0xFF);
        SD_TransmitByte(0xFF);

        uint8_t resp = SD_ReceiveByte();
        if ((resp & 0x1F) != 0x05) {
            return SD_Finish(SD_ERROR);
        }

        SD_ReceiveByte(); /* Eight clocks before checking the programming-busy phase. */
        if (SD_WaitReady() != SD_OK) return SD_Finish(SD_ERROR);
        return SD_Finish(SD_OK);
    } else {
        return SD_WriteMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!SD_ValidRequest(buff, sector, count)) return SD_ERROR;
    while (count--) {
        if (SD_WriteBlocks(buff, sector++, 1) != SD_OK) return SD_ERROR;
        buff += 512;
    }
    return SD_OK;
}
