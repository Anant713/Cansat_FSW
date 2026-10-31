#include "drivers/CalibrationStore.h"
#include "drivers/MissionStore.h"
#include <math.h>
#include <string.h>
#include <float.h>

#define CAL_MAGIC 0x43414C31UL /* CAL1: version 1, IEEE-754 binary32 */
#define CAL_COMMIT 0xCA11B4A7UL
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "Calibration flash format requires binary32");

static bool valid(const SensorCalibrationRecord *r)
{
    if (!r || !isfinite(r->groundPressurePa) || r->groundPressurePa < 30000.0f ||
        r->groundPressurePa > 110000.0f) return false;
    /* Matches the stationary gyro collector's per-axis acceptance bound. */
    for (unsigned i = 0; i < 3; ++i)
        if (!isfinite(r->gyroBiasDps[i]) || fabsf(r->gyroBiasDps[i]) > 5.0f) return false;
    return true;
}

static uint32_t crc32(const uint32_t words[5])
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 5; ++i)
        for (unsigned byte = 0; byte < 4; ++byte) {
            crc ^= (words[i] >> (8U * byte)) & 0xffU;
            for (unsigned bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320UL : 0U);
        }
    return ~crc;
}

CalibrationStoreResult CalibrationStore_Load(SensorCalibrationRecord *out)
{
    bool seen = false, found = false;
    (void)MissionStore_Load(0); /* Reconstruct the common allocation cursor. */
    for (uint32_t offset = 0; offset + 32U <= MISSION_STORE_SIZE; offset += 16U) {
        if (MissionStore_ReadWord(offset) != CAL_MAGIC) continue;
        seen = true;
        uint32_t words[8];
        for (unsigned i = 0; i < 8; ++i) words[i] = MissionStore_ReadWord(offset + 4U * i);
        if (words[7] != CAL_COMMIT || words[6] != ~words[5] || words[5] != crc32(words)) continue;
        SensorCalibrationRecord record;
        memcpy(&record.groundPressurePa, &words[1], 4);
        memcpy(record.gyroBiasDps, &words[2], 12);
        if (!valid(&record)) continue;
        if (out) *out = record;
        found = true;
    }
    return found ? CAL_STORE_VALID : seen ? CAL_STORE_ERROR : CAL_STORE_EMPTY;
}

bool CalibrationStore_Save(const SensorCalibrationRecord *record)
{
    if (!valid(record)) return false;
    uint32_t words[8] = {CAL_MAGIC};
    memcpy(&words[1], &record->groundPressurePa, 4);
    memcpy(&words[2], record->gyroBiasDps, 12);
    words[5] = crc32(words);
    words[6] = ~words[5];
    words[7] = CAL_COMMIT;
    return MissionStore_AppendWords(words, 8U);
}
