#include "drivers/SDCard.h"
#include "sd_functions.h"
#include "sd_spi.h"
#include <limits.h>
#include <string.h>

static FIL file;
static bool mounted, file_open;
static SDCardStatus status = SDCARD_ERROR;
static FRESULT last_result = FR_NOT_READY;

static bool result(FRESULT value)
{
    last_result = value;
    status = value == FR_OK ? SDCARD_IDLE : SDCARD_ERROR;
    return value == FR_OK;
}

static bool begin(void)
{
    if (status == SDCARD_BUSY) return false;
    if (!mounted || !card_initialized) return result(FR_NOT_READY);
    status = SDCARD_BUSY;
    return true;
}

bool SDCard_Init(void)
{
    if (status == SDCARD_BUSY) return false;
    if (mounted && card_initialized) return result(FR_OK);
    if (file_open) return result(FR_LOCKED); /* DeInit explicitly after errors. */
    status = SDCARD_BUSY;
    FRESULT value = (FRESULT)sd_mount();
    mounted = value == FR_OK;
    return result(value);
}

static bool open_file(const char *name, bool append)
{
    if (!begin()) return false;
    if (!name || !*name) return result(FR_INVALID_PARAMETER);
    if (file_open) return result(FR_LOCKED);
    FRESULT value = f_open(&file, name, FA_WRITE | (append ? FA_OPEN_ALWAYS : FA_CREATE_NEW));
    if (value != FR_OK) return result(value);
    file_open = true;
    if (append) {
        value = f_lseek(&file, f_size(&file));
        if (value != FR_OK) {
            /* Keep failed-close state visible until explicit DeInit. */
            if (f_close(&file) == FR_OK) file_open = false;
        }
    }
    return result(value);
}

bool SDCard_CreateFile(const char *name) { return open_file(name, false); }
bool SDCard_OpenAppend(const char *name) { return open_file(name, true); }

bool SDCard_Write(const uint8_t *data, uint32_t length)
{
    if (!begin()) return false;
    if (!file_open) return result(FR_INVALID_OBJECT);
    if ((!data && length) || length > UINT_MAX) return result(FR_INVALID_PARAMETER);
    if (!length) return result(FR_OK);
    UINT written = 0;
    FRESULT value = f_write(&file, data, (UINT)length, &written);
    if (value == FR_OK && written != length) value = FR_DISK_ERR;
    return result(value);
}

bool SDCard_WriteString(const char *text)
{
    if (!text) return result(FR_INVALID_PARAMETER);
    size_t length = strlen(text);
    if (length > UINT32_MAX) return result(FR_INVALID_PARAMETER);
    return SDCard_Write((const uint8_t *)text, (uint32_t)length);
}

bool SDCard_ReadFile(const char *name, uint8_t *data, uint32_t capacity,
                     uint32_t *bytes_read)
{
    if (bytes_read) *bytes_read = 0;
    if (!begin()) return false;
    if (!name || !data || !bytes_read || capacity > UINT_MAX)
        return result(FR_INVALID_PARAMETER);
    /* Reads start at byte zero; caller decides whether EOF/short read is OK. */
    FIL input;
    FRESULT value = f_open(&input, name, FA_READ);
    if (value != FR_OK) return result(value);
    UINT received = 0;
    value = f_read(&input, data, (UINT)capacity, &received);
    *bytes_read = received;
    FRESULT closed = f_close(&input);
    return result(value == FR_OK ? closed : value);
}

bool SDCard_Sync(void)
{
    if (!begin()) return false;
    return result(file_open ? f_sync(&file) : FR_INVALID_OBJECT);
}

bool SDCard_CloseFile(void)
{
    if (status == SDCARD_BUSY) return false;
    if (!file_open) return result(FR_OK);
    status = SDCARD_BUSY;
    FRESULT value = f_close(&file);
    if (value == FR_OK) file_open = false;
    return result(value);
}

bool SDCard_DeInit(void)
{
    if (status == SDCARD_BUSY) return false;
    FRESULT closed = FR_OK;
    if (file_open) {
        (void)SDCard_CloseFile();
        closed = last_result;
    }
    status = SDCARD_BUSY;
    FRESULT value = (FRESULT)sd_unmount();
    if (value == FR_OK) { mounted = false; file_open = false; }
    /* Explicit recovery discards stale handles, but reports unflushed data. */
    return result(closed == FR_OK ? value : closed);
}

bool SDCard_IsReady(void) { return mounted && card_initialized && status == SDCARD_IDLE; }
bool SDCard_IsBusy(void) { return status == SDCARD_BUSY; }
SDCardStatus SDCard_GetStatus(void) { return status; }
FRESULT SDCard_GetLastResult(void) { return last_result; }
