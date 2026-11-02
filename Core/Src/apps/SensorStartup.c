#include "apps/SensorAcquisition.h"
#include "apps/Cansat.h"
#include "drivers/MissionStore.h"
#include "main.h"

bool SensorAcquisition_CalibrateAtStartup(Cansat *c)
{
    if (!c) return false;
    uint32_t payload = 0;
    MissionStoreResult stored = MissionStore_Load(&payload);
    if (stored == MISSION_STORE_ERROR) return false;
    /* Keep a valid mission record in the shared journal before calibration. */
    if (stored == MISSION_STORE_EMPTY && !MissionStore_Save(CANSAT_STATE_LAUNCH_PAD))
        return false;
    if ((payload & 0xffU) != CANSAT_STATE_LAUNCH_PAD)
        return SensorAcquisition_HasCalibration();
    if (!SensorAcquisition_StartCalibration(c)) {
        (void)SensorAcquisition_UseFallbackGroundPressure(c);
        return false;
    }
    while (SensorAcquisition_GetCalibrationStatus() == SENSOR_CAL_RUNNING) {
        SensorAcquisition_Update(c);
        /* The existing calibration service enforces its gyro deadline and
         * commits ground pressure + gyro biases only on complete success. */
        if (SensorAcquisition_GetCalibrationStatus() == SENSOR_CAL_RUNNING)
            HAL_Delay(ACQUISITION_PERIOD_MS);
    }
    bool ok = SensorAcquisition_GetCalibrationStatus() == SENSOR_CAL_SUCCESS;
    /* Refresh height after the candidate reference becomes committed. */
    if (ok) (void)SensorAcquisition_UpdateBMP(c);
    else (void)SensorAcquisition_UseFallbackGroundPressure(c);
    return ok;
}
