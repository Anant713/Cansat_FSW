#include "drivers/CalibrationStore.h"
#include "drivers/MissionStore.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t flash[MISSION_STORE_SIZE / 4U];
static int writesLeft = -1;
uint32_t MissionStore_ReadWord(uint32_t offset)
{ assert(offset < MISSION_STORE_SIZE && !(offset & 3U)); return flash[offset / 4U]; }
bool MissionStore_WriteWord(uint32_t offset, uint32_t value)
{
    assert(offset < MISSION_STORE_SIZE && !(offset & 3U));
    if (writesLeft == 0) return false;
    if (writesLeft > 0) --writesLeft;
    if (flash[offset / 4U] != UINT32_MAX) return false;
    flash[offset / 4U] = value;
    return true;
}
static void fresh(void)
{
    memset(flash, 0xff, sizeof(flash)); writesLeft = -1;
    assert(MissionStore_Load(NULL) == MISSION_STORE_EMPTY);
}
static void equal(const SensorCalibrationRecord *a, const SensorCalibrationRecord *b)
{
    assert(a->groundPressurePa == b->groundPressurePa);
    for (unsigned i = 0; i < 3; ++i) assert(a->gyroBiasDps[i] == b->gyroBiasDps[i]);
}
int main(void)
{
    SensorCalibrationRecord a = {101325.25f, {0.125f, -0.25f, 0.375f}};
    SensorCalibrationRecord b = {99000.5f, {-1.25f, 2.5f, -3.75f}}, out;
    uint32_t state;
    fresh(); assert(CalibrationStore_Load(&out) == CAL_STORE_EMPTY);
    assert(MissionStore_Save(0));
    assert(CalibrationStore_Save(&a));
    assert(MissionStore_Save(1));
    assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &a);
    assert(MissionStore_Load(&state) == MISSION_STORE_VALID && state == 1);
    assert(CalibrationStore_Save(&b));
    assert(MissionStore_Save(2));
    assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &b);
    assert(MissionStore_Load(&state) == MISSION_STORE_VALID && state == 2);
    /* A reset at every word boundary leaves the previous complete tuple. */
    for (int cut = 0; cut < 8; ++cut) {
        fresh(); assert(MissionStore_Save(0)); assert(CalibrationStore_Save(&a));
        writesLeft = cut; assert(!CalibrationStore_Save(&b)); writesLeft = -1;
        assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &a);
        assert(MissionStore_Save(1));
        assert(CalibrationStore_Save(&b));
        assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &b);
        assert(MissionStore_Load(&state) == MISSION_STORE_VALID && state == 1);
    }
    for (unsigned word = 0; word < 8; ++word) {
        fresh(); assert(MissionStore_Save(0)); assert(CalibrationStore_Save(&a));
        assert(CalibrationStore_Save(&b));
        flash[12 + word] ^= 1U;
        assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &a);
    }
    fresh(); assert(MissionStore_Save(0));
    writesLeft = 3; assert(!CalibrationStore_Save(&a)); writesLeft = -1;
    assert(CalibrationStore_Load(&out) == CAL_STORE_ERROR);
    assert(MissionStore_Load(&state) == MISSION_STORE_VALID && state == 0);
    b = a; b.groundPressurePa = NAN; assert(!CalibrationStore_Save(&b));
    b = a; b.gyroBiasDps[0] = INFINITY; assert(!CalibrationStore_Save(&b));
    b = a; b.gyroBiasDps[1] = 5.1f; assert(!CalibrationStore_Save(&b));
    assert(!CalibrationStore_Save(NULL));
    fresh(); assert(MissionStore_Save(0)); assert(CalibrationStore_Save(&a));
    for (unsigned i = 3; i < MISSION_STORE_SIZE / 16U; ++i) assert(MissionStore_Save(1));
    assert(!CalibrationStore_Save(&a));
    assert(CalibrationStore_Load(&out) == CAL_STORE_VALID); equal(&out, &a);
    puts("Calibration journal: roundtrip, mixed records, torn writes, CRC, ranges and full flash passed");
    return 0;
}
