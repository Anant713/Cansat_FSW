#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float servo1Angle;
    float servo2Angle;

    bool enabled;

} ServoData;


bool Servo_Init(void);

void Servo_Enable(void);

void Servo_Disable(void);

void Servo1_SetAngle(float angle);

void Servo2_SetAngle(float angle);

void Servo_SetAngles(float servo1,
                     float servo2);

bool Servo_GetData(ServoData *data);

#endif