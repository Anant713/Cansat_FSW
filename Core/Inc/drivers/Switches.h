#ifndef SWITCHES_H
#define SWITCHES_H

#include <stdbool.h>

typedef struct
{
    bool led;
    bool buzzer;
    bool sensors;
    bool dvr;
    bool gnss;
    bool xbeeWake;

} SwitchState;


bool Switches_Init(void);

void Switches_Update(SwitchState *state);

void Switches_LED(bool enable);

void Switches_Buzzer(bool enable);

void Switches_Sensors(bool enable);

void Switches_DVR(bool enable);

void Switches_GNSS(bool enable);

bool Switches_IsXBeeAwake(void);

#endif