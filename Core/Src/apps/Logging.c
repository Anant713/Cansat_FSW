#include "apps/Logging.h"
#include "apps/Telemetry.h"
#include "drivers/SDCard.h"
#include <stdio.h>

static bool ready, fileOpen, failed;
static char filename[16];
static char line[TELEMETRY_CSV_CAPACITY];

static bool write_and_close(const char *data)
{
    bool written = SDCard_WriteString(data);
    bool closed = SDCard_CloseFile(); /* Flush even after a failed/short write. */
    if (closed) fileOpen = false;
    if (!written || !closed) { ready = false; failed = true; }
    return written && closed;
}

bool Logging_Init(void)
{
    if (ready) return true;
    if (fileOpen || failed) return false;
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
            if (!write_and_close(header)) return false;
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
    if (!SDCard_OpenAppend(filename)) { ready = false; failed = true; return false; }
    fileOpen = true;
    return write_and_close(line);
}

bool Logging_IsReady(void) { return ready; }
const char *Logging_GetFilename(void) { return filename; }

bool Logging_Stop(void)
{
    ready = false;
    /* OpenAppend can fail during seek/close; driver retains that handle. */
    if ((fileOpen || failed) && !SDCard_CloseFile()) return false;
    fileOpen = false;
    failed = false;
    return true;
}
