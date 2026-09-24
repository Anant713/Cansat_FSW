#ifndef REACTION_WHEEL_H
#define REACTION_WHEEL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float commandedSpeed;

    uint16_t pwmCommand;

    bool enabled;
    bool valid;

} ReactionWheelData;


bool ReactionWheel_Init(void);

void ReactionWheel_Enable(void);

void ReactionWheel_Disable(void);

void ReactionWheel_SetPWM(uint16_t pwm);

void ReactionWheel_SetSpeed(float speed);

bool ReactionWheel_GetData(ReactionWheelData *data);

#endif