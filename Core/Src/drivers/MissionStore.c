#include "drivers/MissionStore.h"
#define RECORD_SIZE 16U
#define MAGIC 0x43414E31UL
#define COMMIT 0x51A7C0DEUL
static uint32_t nextOffset;
static bool valid_payload(uint32_t data)
{
    uint32_t pressure = (data >> MISSION_PRESSURE_SHIFT) & MISSION_PRESSURE_MASK;
    return (data & ~0x07ffff07UL) == 0U && (data & 0xffU) <= 4U &&
           (pressure == 0U || (pressure >= 30000U && pressure <= 110000U));
}

MissionStoreResult MissionStore_Load(uint32_t *payload)
{
    bool occupied = false, found = false;
    nextOffset = 0;
    for (uint32_t offset = 0; offset < MISSION_STORE_SIZE; offset += RECORD_SIZE) {
        uint32_t magic = MissionStore_ReadWord(offset);
        uint32_t data = MissionStore_ReadWord(offset + 4U);
        uint32_t inverse = MissionStore_ReadWord(offset + 8U);
        uint32_t commit = MissionStore_ReadWord(offset + 12U);
        if ((magic & data & inverse & commit) != UINT32_MAX) {
            occupied = true;
            nextOffset = offset + RECORD_SIZE;
        }
        if (magic == MAGIC && inverse == ~data && commit == COMMIT &&
            valid_payload(data)) {
            if (payload) *payload = data;
            found = true;
        }
    }
    return found ? MISSION_STORE_VALID : occupied ? MISSION_STORE_ERROR : MISSION_STORE_EMPTY;
}
bool MissionStore_Save(uint32_t payload)
{
    if (!valid_payload(payload) ||
        nextOffset >= MISSION_STORE_SIZE) return false;
    uint32_t offset = nextOffset;
    nextOffset += RECORD_SIZE; /* A torn slot is never reused. */
    return MissionStore_WriteWord(offset, MAGIC) &&
        MissionStore_WriteWord(offset + 4U, payload) &&
        MissionStore_WriteWord(offset + 8U, ~payload) &&
        MissionStore_WriteWord(offset + 12U, COMMIT);
}
