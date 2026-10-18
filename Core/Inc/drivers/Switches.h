#ifndef SWITCHES_H
#define SWITCHES_H

#include <stdbool.h>

typedef struct
{
    bool led;
    bool buzzer;
    bool dvr;
    bool hdrm;
    bool capture; // PA9 / DVR_Control output level

} SwitchState;


bool Switches_Init(void);

/* Snapshot of output latches (not feedback from the attached loads).
 * NULL is ignored. Init must follow MX_GPIO_Init; it turns all outputs off. */
void Switches_Update(SwitchState *state);

void Switches_LED(bool enable);

void Switches_Buzzer(bool enable);

void Switches_DVR(bool enable);

void Switches_HDRM(bool enable);

/* Sets the control level; does not generate a timed capture pulse. */
void Switch_Capture(bool enable);

#endif
