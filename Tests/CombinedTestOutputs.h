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
        Servo_SetAngles(0.0f, 0.0f);
        Servo_Enable();
    }
    Switches_Update(&c->switches);
    c->actuators.servoValid = Servo_GetData(&c->actuators.servo);
    return ok && servoOK && c->actuators.servoValid && c->actuators.servo.enabled;
}
