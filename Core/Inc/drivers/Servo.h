#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

#define SERVO_MIN_ANGLE_DEG 5.0f
#define SERVO_MAX_ANGLE_DEG 45.0f

typedef struct
{
    /* Requested angles, not measured positions. Servo 1 pulls the left brake
     * line; servo 2 pulls the right brake line. */
    float servo1Angle;
    float servo2Angle;

    bool enabled;

} ServoData;


bool Servo_Init(void);
/* Init follows MX_TIM4_Init. It validates 1 MHz counting / 20 ms PWM,
 * selects the 5-degree minimum for both servos and leaves PWM disabled.
 * All calls are single-caller/main-context; TIM4 is reserved for this driver. */

void Servo_Enable(void);
/* GetData().enabled remains false if either HAL PWM start fails. */

void Servo_Disable(void);

void Servo1_SetAngle(float angle);
/* Finite angles are clamped to [5,45] degrees. NaN/infinity are ignored.
 * SetAngles rejects the entire pair if either value is non-finite.
 * Setters also work while disabled; Enable applies the saved angles.
 * Running updates take effect on the next 20 ms timer update. */

void Servo2_SetAngle(float angle);

void Servo_SetAngles(float servo1,
                     float servo2);

bool Servo_GetData(ServoData *data);
/* Returns false for NULL or before a successful Init. No position feedback. */

#endif
