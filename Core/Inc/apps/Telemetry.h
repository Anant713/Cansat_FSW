#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stddef.h>
#include <stdint.h>
typedef struct Cansat Cansat;

#define TELEMETRY_BINARY_SIZE 151U
#define TELEMETRY_FIELD_COUNT 44U
#define TELEMETRY_CSV_CAPACITY 512U /* Includes CRLF and local NUL. */
#define TELEMETRY_BMP_MAX_AGE_MS 1000U
#define TELEMETRY_BMI_MAX_AGE_MS 250U
#define TELEMETRY_MAG_MAX_AGE_MS 250U
#define TELEMETRY_AHT_MAX_AGE_MS 3000U
#define TELEMETRY_ATTITUDE_MAX_AGE_MS 250U

/* Shared quantization: native Cansat units -> binary wire values/sentinels.
 * Does not read hardware, increment counters, or change Cansat. */
size_t Telemetry_Encode(const Cansat *cansat, uint8_t *buffer, size_t capacity);
/* Decimal representation of exactly those encoded fields, same order/units.
 * Signed int16 fields print signed values (including -32768 sentinel).
 * Hemisphere is N/S/E/W or 255 for unavailable; team ID padding is omitted.
 * Ends CRLF, then local NUL. Returns length excluding NUL; zero on failure.
 * On failure, an available output buffer is cleared; never log a partial row. */
size_t Telemetry_FormatCSV(const Cansat *cansat, char *buffer, size_t capacity);
const char *Telemetry_CSVHeader(void); /* 44 names, CRLF, NUL. */
#endif
