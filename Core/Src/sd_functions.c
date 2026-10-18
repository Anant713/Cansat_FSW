/* Adapted from ControllersTech sd_functions.c (June/September 2025).
 * https://controllerstech.com -- original permitted learning/development use. */
#include "sd_functions.h"
#include "sd_diskio_spi.h"
#include "sd_spi.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>

static bool mounted;

int sd_mount(void)
{
    if (mounted && card_initialized) return FR_OK;
    /* CubeMX USER_Driver is the only registered volume. This also works when
     * MX_FATFS_Init has already been called by generated main(). */
    if (FATFS_GetAttachedDriversNbr() == 0) MX_FATFS_Init();
    if (retUSER != 0 || USERPath[0] != '0') return FR_NOT_ENABLED;
    (void)f_mount(NULL, USERPath, 0);
    mounted = false;
    /* ST disk_initialize caches success. Explicit physical init permits
     * recovery after removal, timeout or a previous failed mount. */
    if (SD_disk_initialize(0) != 0) return FR_NOT_READY;
    FRESULT result = f_mount(&USERFatFS, USERPath, 1);
    mounted = result == FR_OK;
    if (!mounted) (void)f_mount(NULL, USERPath, 0);
    return result;
}

int sd_unmount(void)
{
    /* Callers must sync/close all files first. Keep the driver registered:
     * repeated mount/unmount must not consume another FatFs volume slot. */
    FRESULT result = FR_OK;
    if (USERPath[0]) result = f_mount(NULL, USERPath, 0);
    if (result == FR_OK) mounted = false;
    return result;
}

static int write_text(const char *name, const char *text, bool append)
{
    if (!name || !text) return FR_INVALID_PARAMETER;
    size_t length = strlen(text);
    if (length > UINT_MAX) return FR_INVALID_PARAMETER;
    FIL file;
    UINT written = 0;
    FRESULT result = f_open(&file, name,
                           FA_WRITE | (append ? FA_OPEN_ALWAYS : FA_CREATE_ALWAYS));
    if (result != FR_OK) return result;
    if (append) result = f_lseek(&file, f_size(&file));
    if (result == FR_OK) result = f_write(&file, text, (UINT)length, &written);
    FRESULT closed = f_close(&file);
    if (result != FR_OK) return result;
    if (written != length) return FR_DISK_ERR; /* e.g. full volume */
    return closed;
}

int sd_write_file(const char *name, const char *text)
{
    return write_text(name, text, false);
}
int sd_append_file(const char *name, const char *text)
{
    return write_text(name, text, true);
}
int sd_read_file(const char *name, char *buffer, UINT size, UINT *bytes_read)
{
    if (bytes_read) *bytes_read = 0;
    if (!name || !buffer || !size || !bytes_read) return FR_INVALID_PARAMETER;
    buffer[0] = '\0';
    FIL file;
    FRESULT result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return result;
    result = f_read(&file, buffer, size - 1U, bytes_read);
    buffer[*bytes_read] = '\0';
    FRESULT closed = f_close(&file);
    return result == FR_OK ? closed : result;
}

int sd_read_csv(const char *name, CsvRecord *records, int limit, int *count)
{
    if (count) *count = 0;
    if (!name || !records || limit <= 0 || !count) return FR_INVALID_PARAMETER;
    FIL file;
    FRESULT result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return result;
    char line[128];
    /* Simple unquoted field1,field2,integer CSV, no header. Reject malformed
     * or truncated records instead of storing unterminated fields. */
    while (*count < limit && f_gets(line, sizeof(line), &file)) {
        if (!strchr(line, '\n') && !f_eof(&file)) { result = FR_INVALID_PARAMETER; break; }
        char *first = strchr(line, ',');
        char *second = first ? strchr(first + 1, ',') : NULL;
        if (!first || !second) { result = FR_INVALID_PARAMETER; break; }
        *first++ = '\0';
        *second++ = '\0';
        if (strlen(line) >= sizeof(records[0].field1) ||
            strlen(first) >= sizeof(records[0].field2)) {
            result = FR_INVALID_PARAMETER; break;
        }
        char *end;
        errno = 0;
        long value = strtol(second, &end, 10);
        if (end == second || errno == ERANGE || value < INT_MIN || value > INT_MAX) {
            result = FR_INVALID_PARAMETER; break;
        }
        while (*end == '\r' || *end == '\n') ++end;
        if (*end) { result = FR_INVALID_PARAMETER; break; }
        strcpy(records[*count].field1, line);
        strcpy(records[*count].field2, first);
        records[*count].value = (int)value;
        ++*count;
    }
    if (result == FR_OK && f_error(&file)) result = (FRESULT)f_error(&file);
    FRESULT closed = f_close(&file);
    return result == FR_OK ? closed : result;
}
int sd_delete_file(const char *name)
{
    return name ? f_unlink(name) : FR_INVALID_PARAMETER;
}
int sd_rename_file(const char *oldname, const char *newname)
{
    return oldname && newname ? f_rename(oldname, newname) : FR_INVALID_PARAMETER;
}
FRESULT sd_create_directory(const char *path)
{
    return path ? f_mkdir(path) : FR_INVALID_PARAMETER;
}
int sd_get_space_kb(void)
{
    FATFS *fs;
    DWORD free_clusters;
    FRESULT result = f_getfree(USERPath, &free_clusters, &fs);
    if (result != FR_OK) return result;
    printf("Total: %lu KiB, Free: %lu KiB\r\n",
           (unsigned long)(((fs->n_fatent - 2U) * fs->csize) / 2U),
           (unsigned long)((free_clusters * fs->csize) / 2U));
    return FR_OK;
}
void sd_list_directory_recursive(const char *path, int depth)
{
    if (!path || depth < 0 || depth > 3) return;
    DIR dir;
    FILINFO info = {0};
    FRESULT result = f_opendir(&dir, path);
    if (result != FR_OK) return;
    while (f_readdir(&dir, &info) == FR_OK && info.fname[0]) {
        const char *name = info.fname;
        if (!strcmp(name, ".") || !strcmp(name, "..")) continue;
        printf("%*s%s (%lu bytes)\r\n", depth * 2, "", name, (unsigned long)info.fsize);
        if ((info.fattrib & AM_DIR) && depth < 3) {
            char nested[256];
            int n = snprintf(nested, sizeof(nested), "%s/%s", path, name);
            if (n > 0 && (size_t)n < sizeof(nested))
                sd_list_directory_recursive(nested, depth + 1);
        }
    }
    (void)f_closedir(&dir);
}
void sd_list_files(void)
{
    sd_list_directory_recursive(USERPath, 0);
}
