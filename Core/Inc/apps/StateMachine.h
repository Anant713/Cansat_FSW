#pragma once
#include "apps/Cansat.h"

#define STATE_LAUNCH_HEIGHT_M 30.0f
#define STATE_DESCENT_ARM_HEIGHT_M 800.0f
#define STATE_DESCENT_DROP_M 2.0f
#define STATE_DESCENT_WINDOW_MS 5000U
#define STATE_PARAFOIL_HEIGHT_M 600.0f
#define STATE_TOUCHDOWN_HEIGHT_M 20.0f
#define STATE_HDRM_PULSE_MS 2000U
#define STATE_ALTITUDE_MAX_AGE_MS 1000U

typedef struct {
    bool recovered;
    bool persistenceOK;
    bool crossedArmHeight;
    bool hdrmActive;
    bool deploymentComplete;
    bool reactionWheelRequested; /* Driver integration pending; not hardware feedback. */
} StateMachineStatus;

bool StateMachine_Init(Cansat *cansat);
void StateMachine_Update(Cansat *cansat);
const StateMachineStatus *StateMachine_GetStatus(void);
/* Explicit ground-only new-flight operation. Never called automatically at boot. */
bool StateMachine_BeginNewMission(Cansat *cansat);
/* Called once by altitude estimation on a fresh launch-pad mission. */
bool StateMachine_SetGroundPressure(Cansat *cansat, float pressurePa);
