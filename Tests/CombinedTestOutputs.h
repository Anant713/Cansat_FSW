#pragma once
#include "apps/Cansat.h"
#include "drivers/Switches.h"
#include "drivers/Servo.h"

/* Test startup only, before blocking sensor/SD work. PWM stays enabled. */
static inline bool CombinedTestOutputs_Init(Cansat *c)
{
    bool ok = Switches_Init();
    Switches_LED(true);
    bool servoOK = Servo_Init();
    if (servoOK) {
        Servo_SetAngles(SERVO_MIN_ANGLE_DEG, SERVO_MIN_ANGLE_DEG);
        Servo_Enable();
    }
    Switches_Update(&c->switches);
    c->actuators.servoValid = Servo_GetData(&c->actuators.servo);
    return ok && servoOK && c->actuators.servoValid && c->actuators.servo.enabled;
}
