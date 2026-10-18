#include "apps/StateMachine.h"
#include "apps/CameraControl.h"
#include "apps/Logging.h"
#include "drivers/MissionStore.h"
#include "drivers/Switches.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

static StateMachineStatus status;
static bool initialized, windowActive;
static uint32_t windowStart, hdrmStart;
static float windowAltitude;
static uint32_t groundPressurePa;

const StateMachineStatus *StateMachine_GetStatus(void) { return &status; }

static bool persist(CansatMissionState state, bool crossed, bool deployed)
{
    status.persistenceOK = MissionStore_Save((uint32_t)state |
        (crossed ? 0x100U : 0U) | (deployed ? 0x200U : 0U) |
        (groundPressurePa << MISSION_PRESSURE_SHIFT));
    return status.persistenceOK;
}

static void enter(Cansat *c)
{
    windowActive = false;
    status.reactionWheelRequested = c->missionState == CANSAT_STATE_DESCENT_1 ||
                                   c->missionState == CANSAT_STATE_DESCENT_2;
    /* Software enable only; PWM generation on PB5 is added with control later. */
    c->actuators.reactionWheel.enabled = status.reactionWheelRequested;
    if (c->missionState >= CANSAT_STATE_ASCENT && c->missionState <= CANSAT_STATE_DESCENT_2)
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
    status = (StateMachineStatus){0};
    windowActive = false;
    uint32_t payload = 0;
    groundPressurePa = 0;
    c->navigation.groundPressurePa = 0;
    c->navigation.altitudeValid = false;
    MissionStoreResult result = MissionStore_Load(&payload);
    c->missionState = CANSAT_STATE_UNINITIALIZED;
    if (result == MISSION_STORE_ERROR) return false;
    if (result == MISSION_STORE_EMPTY && !persist(CANSAT_STATE_LAUNCH_PAD, false, false)) return false;
    status.recovered = result == MISSION_STORE_VALID;
    groundPressurePa = (payload >> MISSION_PRESSURE_SHIFT) & MISSION_PRESSURE_MASK;
    c->navigation.groundPressurePa = (float)groundPressurePa;
    status.persistenceOK = true;
    status.crossedArmHeight = (payload & 0x100U) != 0;
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
    uint32_t previousPressure = groundPressurePa;
    groundPressurePa = 0;
    if (!persist(CANSAT_STATE_LAUNCH_PAD, false, false)) {
        groundPressurePa = previousPressure;
        return false;
    }
    c->navigation.groundPressurePa = 0;
    c->navigation.altitudeValid = false;
    Switches_HDRM(false);
    Switches_Buzzer(false);
    CameraControl_Stop(&c->camera);
    status = (StateMachineStatus){.persistenceOK = true};
    c->missionState = CANSAT_STATE_LAUNCH_PAD;
    initialized = true;
    enter(c);
    return true;
}

bool StateMachine_SetGroundPressure(Cansat *c, float pressurePa)
{
    if (!c || !initialized || !status.persistenceOK || groundPressurePa != 0U ||
        c->missionState != CANSAT_STATE_LAUNCH_PAD || !isfinite(pressurePa) ||
        pressurePa < 30000.0f || pressurePa > 110000.0f) return false;
    groundPressurePa = (uint32_t)roundf(pressurePa);
    if (!persist(c->missionState, status.crossedArmHeight, status.deploymentComplete)) {
        groundPressurePa = 0;
        return false;
    }
    c->navigation.groundPressurePa = (float)groundPressurePa;
    return true;
}

static void transition(Cansat *c, CansatMissionState next)
{
    /* Commit before actions: a reset replays entry for the committed state. */
    if (!persist(next, status.crossedArmHeight, status.deploymentComplete)) return;
    c->missionState = next;
    enter(c);
}

void StateMachine_Update(Cansat *c)
{
    if (!c || !initialized) return;
    uint32_t now = HAL_GetTick();
    CameraControl_Update(&c->camera);
    if (status.hdrmActive && (uint32_t)(now - hdrmStart) >= STATE_HDRM_PULSE_MS) {
        Switches_HDRM(false); /* Always turn off even if the subsequent flash write fails. */
        status.hdrmActive = false;
        status.deploymentComplete = true;
        c->actuators.parafoilDeployed = true;
        (void)persist(c->missionState, status.crossedArmHeight, true);
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
    bool valid = c->navigation.altitudeValid && isfinite(c->navigation.altitude) &&
        (uint32_t)(now - c->navigation.altitudeTimestampMs) <= STATE_ALTITUDE_MAX_AGE_MS;
    if (!valid) { windowActive = false; Switches_Update(&c->switches); return; }
    float height = c->navigation.altitude;
    switch (c->missionState) {
    case CANSAT_STATE_LAUNCH_PAD:
        if (height > STATE_LAUNCH_HEIGHT_M) transition(c, CANSAT_STATE_ASCENT);
        break;
    case CANSAT_STATE_ASCENT:
        if (!status.crossedArmHeight && height > STATE_DESCENT_ARM_HEIGHT_M) {
            if (!persist(c->missionState, true, false)) break;
            status.crossedArmHeight = true;
        }
        if (!status.crossedArmHeight) break;
        if (!windowActive) {
            windowActive = true; windowStart = now; windowAltitude = height;
        } else if ((uint32_t)(now - windowStart) >= STATE_DESCENT_WINDOW_MS) {
            if (windowAltitude - height > STATE_DESCENT_DROP_M)
                transition(c, CANSAT_STATE_DESCENT_1);
            windowStart = now; windowAltitude = height;
        }
        break;
    case CANSAT_STATE_DESCENT_1:
        if (height < STATE_PARAFOIL_HEIGHT_M) transition(c, CANSAT_STATE_DESCENT_2);
        break;
    case CANSAT_STATE_DESCENT_2:
        if (height < STATE_TOUCHDOWN_HEIGHT_M) transition(c, CANSAT_STATE_TOUCHDOWN);
        break;
    default: break;
    }
    Switches_Update(&c->switches);
}
