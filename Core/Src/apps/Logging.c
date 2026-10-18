#include "apps/Logging.h"
#include "apps/Telemetry.h"
#include "drivers/SDCard.h"
#include <stdio.h>
#include <string.h>

static bool ready, fileOpen;
static char filename[16];
static char line[TELEMETRY_CSV_CAPACITY];

static bool write_and_sync(const char *data, size_t length)
{
    if (!SDCard_Write((const uint8_t *)data, (uint32_t)length) || !SDCard_Sync()) {
        ready = false;
        return false;
    }
    return true;
}

bool Logging_Init(void)
{
    if (fileOpen) return ready;
    ready = false;
    filename[0] = '\0';
    if (!SDCard_Init()) return false;
    /* A new file per boot/session prevents overwriting earlier flights or
     * appending to a possibly incomplete row left by an interrupted flight. */
    for (unsigned number = 0; number < 10000U; ++number) {
        (void)snprintf(filename, sizeof(filename), "0:/LOG%04u.CSV", number);
        if (SDCard_CreateFile(filename)) {
            fileOpen = true;
            const char *header = Telemetry_CSVHeader();
            if (!write_and_sync(header, strlen(header))) return false;
            ready = true;
            return true;
        }
        if (SDCard_GetLastResult() != FR_EXIST) break;
    }
    filename[0] = '\0';
    return false;
}

bool Logging_Write(const Cansat *cansat)
{
    if (!ready) return false;
    size_t length = Telemetry_FormatCSV(cansat, line, sizeof(line));
    if (length == 0U) return false;
    return write_and_sync(line, length);
}

bool Logging_IsReady(void) { return ready; }
const char *Logging_GetFilename(void) { return filename; }

bool Logging_Stop(void)
{
    ready = false;
    if (!fileOpen) return true;
    if (!SDCard_CloseFile()) return false;
    fileOpen = false;
    return true;
}
