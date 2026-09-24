#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Forward declaration */
typedef struct Cansat Cansat;


/* Mission states */

typedef enum
{
    STATE_LAUNCH_PAD = 0,
    STATE_ASCENT,
    STATE_DESCENT_1,
    STATE_DESCENT_2,
    STATE_TOUCHDOWN
} MissionState;


/* Initialization */

void StateMachine_Init(Cansat *cansat);


/* Called periodically from main loop */

void StateMachine_Update(Cansat *cansat);


/* State transition */

void StateMachine_Transition(Cansat *cansat,
                             MissionState newState);


/* Entry/exit handling */

void StateMachine_Enter(Cansat *cansat,
                        MissionState state);

void StateMachine_Exit(Cansat *cansat,
                       MissionState state);


/* Optional query functions */

MissionState StateMachine_GetState(const Cansat *cansat);

bool StateMachine_IsState(const Cansat *cansat,
                          MissionState state);
