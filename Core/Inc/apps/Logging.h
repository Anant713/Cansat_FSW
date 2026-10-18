#ifndef LOGGING_H
#define LOGGING_H
#include <stdbool.h>
typedef struct Cansat Cansat;
#define LOGGING_PERIOD_MS 1000U

/* Main-loop only, blocking. Mounts SD, creates the first unused LOG0000.CSV ..
 * LOG9999.CSV, writes/syncs a header. Never overwrites or appends to old flights.
 * Repeated Init while ready is harmless; after an error call Stop before Init. */
bool Logging_Init(void);
/* Serializes current Cansat through the shared telemetry encoder, writes one
 * scaled-integer CSV row and syncs it. Does not increment telemetry counters.
 * SD write/sync failure disables further writes to avoid extending a partial row. */
bool Logging_Write(const Cansat *cansat);
bool Logging_IsReady(void);
bool Logging_Stop(void); /* Close/flush; retry explicitly if close fails. */
const char *Logging_GetFilename(void); /* Last selected path, or empty before Init. */
#endif
