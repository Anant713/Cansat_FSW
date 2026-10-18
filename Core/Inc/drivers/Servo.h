#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /* Requested angles, not measured positions. */
    float servo1Angle;
    float servo2Angle;

    bool enabled;

} ServoData;


bool Servo_Init(void);
/* Init follows MX_TIM4_Init. It validates 1 MHz counting / 20 ms PWM,
 * selects 90 degrees for both servos and leaves PWM disabled.
 * All calls are single-caller/main-context; TIM4 is reserved for this driver. */

void Servo_Enable(void);
/* GetData().enabled remains false if either HAL PWM start fails. */

void Servo_Disable(void);

void Servo1_SetAngle(float angle);
/* Finite angles are clamped to [0,180]. NaN/infinity are ignored.
 * SetAngles rejects the entire pair if either value is non-finite.
 * Setters also work while disabled; Enable applies the saved angles.
 * Running updates take effect on the next 20 ms timer update. */

void Servo2_SetAngle(float angle);

void Servo_SetAngles(float servo1,
                     float servo2);

bool Servo_GetData(ServoData *data);
/* Returns false for NULL or before a successful Init. No position feedback. */

#endif
