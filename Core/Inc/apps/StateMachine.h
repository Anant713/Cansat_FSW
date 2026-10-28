#pragma once
#include "apps/Cansat.h"

#define STATE_LAUNCH_HEIGHT_M 30.0f
#define STATE_DESCENT_DROP_M 20.0f
#define STATE_DESCENT_MIN_PEAK_M 700.0f
#define STATE_PARAFOIL_HEIGHT_M 600.0f
#define STATE_TOUCHDOWN_HEIGHT_M 20.0f
#define STATE_HDRM_PULSE_MS 2000U
#define STATE_ALTITUDE_MAX_AGE_MS 1000U
#ifndef STATE_HEIGHT_CONFIRM_SAMPLES
#define STATE_HEIGHT_CONFIRM_SAMPLES 5U
#endif

typedef struct {
    bool recovered;
    bool persistenceOK;
    float maxHeightM; /* Highest fresh BMP height observed in ASCENT; RAM-only. */
    bool maxHeightValid;
    uint32_t heightConfirmCount; /* Consecutive qualifying fresh BMP samples. */
    bool hdrmActive;
    bool deploymentComplete;
} StateMachineStatus;

bool StateMachine_Init(Cansat *cansat);
void StateMachine_Update(Cansat *cansat);
const StateMachineStatus *StateMachine_GetStatus(void);
/* Explicit ground-only new-flight operation. Never called automatically at boot. */
bool StateMachine_BeginNewMission(Cansat *cansat);
/* Called once by altitude estimation on a fresh launch-pad mission. */
bool StateMachine_SetGroundPressure(Cansat *cansat, float pressurePa);
