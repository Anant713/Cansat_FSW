#include "apps/SensorAcquisition.h"
#include "apps/Cansat.h"
#include "drivers/MissionStore.h"
#include <assert.h>
#include <stdio.h>

static MissionStoreResult journal;
static uint32_t mission, tick;
static unsigned starts, updates, missionWrites, refreshes;
static bool saved, fail;
static SensorCalibrationStatus status;
MissionStoreResult MissionStore_Load(uint32_t *out) { *out = mission; return journal; }
bool MissionStore_Save(uint32_t payload) { mission = payload; ++missionWrites; return true; }
bool SensorAcquisition_HasCalibration(void) { return saved; }
bool SensorAcquisition_StartCalibration(Cansat *c)
{ assert(c->missionState == CANSAT_STATE_LAUNCH_PAD); ++starts; updates = 0; status = SENSOR_CAL_RUNNING; return true; }
SensorCalibrationStatus SensorAcquisition_GetCalibrationStatus(void) { return status; }
void SensorAcquisition_Update(Cansat *c)
{
    (void)c;
    if (++updates == 3) {
        status = fail ? SENSOR_CAL_FAILED : SENSOR_CAL_SUCCESS;
        if (!fail) saved = true;
    }
}
bool SensorAcquisition_UpdateBMP(Cansat *c) { ++refreshes; c->bmp.heightValid = true; return true; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int main(void)
{
    Cansat c = {0};
    journal = MISSION_STORE_EMPTY;
    assert(SensorAcquisition_CalibrateAtStartup(&c));
    assert(missionWrites == 1 && starts == 1 && updates == 3 && saved && c.bmp.heightValid);
    assert(tick == 2 * ACQUISITION_PERIOD_MS);
    journal = MISSION_STORE_VALID;
    assert(SensorAcquisition_CalibrateAtStartup(&c)); /* Launch-pad reboot recalibrates. */
    assert(starts == 2 && missionWrites == 1);
    for (mission = 1; mission <= 4; ++mission) {
        assert(SensorAcquisition_CalibrateAtStartup(&c));
        assert(starts == 2); /* Airborne/touchdown recovery does not re-zero. */
    }
    mission = 1; saved = false;
    assert(!SensorAcquisition_CalibrateAtStartup(&c));
    journal = MISSION_STORE_ERROR;
    assert(!SensorAcquisition_CalibrateAtStartup(&c));
    journal = MISSION_STORE_VALID; mission = 0; fail = true;
    assert(!SensorAcquisition_CalibrateAtStartup(&c));
    assert(refreshes == 2 && starts == 3);
    assert(!SensorAcquisition_CalibrateAtStartup(NULL));
    puts("Startup calibration: launch-pad completion, recovery, missing calibration and failure passed");
    return 0;
}
