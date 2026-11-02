#include "apps/StateMachine.h"
#include "apps/CameraControl.h"
#include "apps/Logging.h"
#include "drivers/MissionStore.h"
#include "drivers/Switches.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

static StateMachineStatus status;
static bool initialized;
static bool cameraReady = true;
void StateMachine_SetCameraReady(bool ready) { cameraReady = ready; }
static uint32_t hdrmStart;
static uint32_t lastHeightSampleMs;
static bool heightSampleSeen;
_Static_assert(STATE_HEIGHT_CONFIRM_SAMPLES > 0U &&
               STATE_HEIGHT_CONFIRM_SAMPLES <= UINT32_MAX, "Invalid height debounce count");

const StateMachineStatus *StateMachine_GetStatus(void) { return &status; }

static bool persist(CansatMissionState state, bool deployed)
{
    status.persistenceOK = MissionStore_Save((uint32_t)state |
        (deployed ? 0x200U : 0U));
    return status.persistenceOK;
}

static void enter(Cansat *c)
{
    status.heightConfirmCount = 0;
    if (cameraReady && c->missionState >= CANSAT_STATE_LAUNCH_PAD && c->missionState <= CANSAT_STATE_DESCENT_2)
        CameraControl_Start(&c->camera);
    if (c->missionState == CANSAT_STATE_DESCENT_2 && !status.deploymentComplete) {
        Switches_HDRM(true);
        hdrmStart = HAL_GetTick();
        status.hdrmActive = true;
    }
    if (c->missionState == CANSAT_STATE_TOUCHDOWN) {
        Switches_HDRM(false);
        status.hdrmActive = false;
        Switches_Buzzer(true);
        (void)Logging_Write(c); /* Preserve one final row with TOUCHDOWN state. */
        (void)Logging_Stop();
        CameraControl_Stop(&c->camera);
    }
    c->actuators.parafoilDeployed = status.deploymentComplete;
    Switches_Update(&c->switches);
}

bool StateMachine_Init(Cansat *c)
{
    if (!c) return false;
    initialized = false;
    heightSampleSeen = false;
    status = (StateMachineStatus){0};
    uint32_t payload = 0;
    c->navigation.groundPressurePa = 0;
    c->navigation.altitudeValid = false;
    MissionStoreResult result = MissionStore_Load(&payload);
    c->missionState = CANSAT_STATE_UNINITIALIZED;
    if (result == MISSION_STORE_ERROR) return false;
    if (result == MISSION_STORE_EMPTY && !persist(CANSAT_STATE_LAUNCH_PAD, false)) return false;
    status.recovered = result == MISSION_STORE_VALID;
    /* Legacy pressure bits are accepted by MissionStore but not used: the
     * complete calibration tuple is restored by SensorAcquisition. */
    status.persistenceOK = true;
    status.deploymentComplete = (payload & 0x200U) != 0;
    c->missionState = (CansatMissionState)(payload & 0xffU);
    initialized = true;
    enter(c);
    return true;
}

bool StateMachine_BeginNewMission(Cansat *c)
{
    if (!c) return false;
    uint32_t ignored;
    (void)MissionStore_Load(&ignored);
    if (!persist(CANSAT_STATE_LAUNCH_PAD, false)) return false;
    c->navigation.groundPressurePa = 0;
    c->navigation.altitudeValid = false;
    Switches_HDRM(false);
    Switches_Buzzer(false);
    CameraControl_Stop(&c->camera);
    status = (StateMachineStatus){.persistenceOK = true};
    heightSampleSeen = false;
    c->missionState = CANSAT_STATE_LAUNCH_PAD;
    initialized = true;
    enter(c);
    return true;
}

static void transition(Cansat *c, CansatMissionState next)
{
    /* Commit before actions: a reset replays entry for the committed state. */
    if (!persist(next, status.deploymentComplete)) return;
    c->missionState = next;
    enter(c);
}

static bool confirmed(bool condition)
{
    if (!condition) status.heightConfirmCount = 0;
    else if (status.heightConfirmCount < STATE_HEIGHT_CONFIRM_SAMPLES)
        ++status.heightConfirmCount;
    return status.heightConfirmCount >= STATE_HEIGHT_CONFIRM_SAMPLES;
}

void StateMachine_Update(Cansat *c)
{
    if (!c || !initialized) return;
    uint32_t now = HAL_GetTick();
    if (cameraReady && c->missionState >= CANSAT_STATE_LAUNCH_PAD &&
        c->missionState <= CANSAT_STATE_DESCENT_2)
        CameraControl_Start(&c->camera); /* Idempotent; also handles deferred startup. */
    CameraControl_Update(&c->camera);
    if (status.hdrmActive && (uint32_t)(now - hdrmStart) >= STATE_HDRM_PULSE_MS) {
        Switches_HDRM(false); /* Always turn off even if the subsequent flash write fails. */
        status.hdrmActive = false;
        status.deploymentComplete = true;
        c->actuators.parafoilDeployed = true;
        (void)persist(c->missionState, true);
    }
    if (!status.persistenceOK) {
        /* No automatic flash retry loop. Timed output shutdown above still runs. */
        Switches_Update(&c->switches);
        return;
    }
    if (c->missionState == CANSAT_STATE_TOUCHDOWN) {
        (void)Logging_Stop(); /* Also retry a failed close. */
        Switches_Update(&c->switches);
        return;
    }
    bool valid = c->bmp.valid && c->bmp.heightValid && isfinite(c->bmp.relativeHeightM) &&
        (uint32_t)(now - c->bmp.timestamp_ms) <= STATE_ALTITUDE_MAX_AGE_MS;
    if (!valid) {
        status.heightConfirmCount = 0;
        Switches_Update(&c->switches);
        return;
    }
    /* Acquisition retains its last sample between reads. Do not debounce loop
     * iterations: each BMP timestamp can contribute at most once. */
    if (heightSampleSeen && c->bmp.timestamp_ms == lastHeightSampleMs) {
        Switches_Update(&c->switches);
        return;
    }
    if (heightSampleSeen &&
        (uint32_t)(c->bmp.timestamp_ms - lastHeightSampleMs) > STATE_ALTITUDE_MAX_AGE_MS)
        status.heightConfirmCount = 0;
    lastHeightSampleMs = c->bmp.timestamp_ms;
    heightSampleSeen = true;
    float height = c->bmp.relativeHeightM;
    switch (c->missionState) {
    case CANSAT_STATE_LAUNCH_PAD:
        if (confirmed(height > STATE_LAUNCH_HEIGHT_M)) transition(c, CANSAT_STATE_ASCENT);
        break;
    case CANSAT_STATE_ASCENT:
        if (!status.maxHeightValid || height > status.maxHeightM) {
            status.maxHeightM = height;
            status.maxHeightValid = true;
        }
        if (confirmed(status.maxHeightM > STATE_DESCENT_MIN_PEAK_M &&
                      status.maxHeightM - height > STATE_DESCENT_DROP_M))
            transition(c, CANSAT_STATE_DESCENT_1);
        break;
    case CANSAT_STATE_DESCENT_1:
        if (confirmed(height < STATE_PARAFOIL_HEIGHT_M)) transition(c, CANSAT_STATE_DESCENT_2);
        break;
    case CANSAT_STATE_DESCENT_2:
        if (confirmed(height < STATE_TOUCHDOWN_HEIGHT_M)) transition(c, CANSAT_STATE_TOUCHDOWN);
        break;
    default: break;
    }
    Switches_Update(&c->switches);
}

