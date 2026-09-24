#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drivers/GNSS.h"
#include "drivers/BME.h"
#include "drivers/BMI.h"
#include "drivers/IIS2MDC.h"
#include "drivers/Battery.h"
#include "drivers/ReactionWheel.h"
#include "drivers/Servo.h"
#include "drivers/Switches.h"

/* ---------- Mission state ---------- */

typedef enum
{
    CANSAT_STATE_LAUNCH_PAD = 0,
    CANSAT_STATE_ASCENT,
    CANSAT_STATE_DESCENT_1,
    CANSAT_STATE_DESCENT_2,
    CANSAT_STATE_TOUCHDOWN
} CansatMissionState;


/* ---------- Attitude ---------- */

typedef struct
{
    float roll;
    float pitch;
    float yaw;

    float rollRate;
    float pitchRate;
    float yawRate;
} AttitudeData;


/* ---------- Navigation / state estimation ---------- */

typedef struct
{
    float altitude;
    float verticalVelocity;
    float horizontalVelocity;

    double latitude;
    double longitude;
} NavigationData;


/* ---------- Actuator state ---------- */

typedef struct
{
    ReactionWheelData reactionWheel;
    ServoData servo;
} ActuatorState;

/* ---------- System status ---------- */

typedef struct
{
    bool bmeOK;
    bool bmiOK;
    bool magOK;
    bool gnssOK;
    bool sdCardOK;
    bool xbeeOK;

    bool reactionWheelOK;
    bool servoOK;

} SystemStatus;

/* ---------- CANSAT system state ---------- */

typedef struct
{
    /* Sensor data */
    GNSSData gnss;
    BMEData bme;
    BMIData bmi;
    IIS2MDCData mag;
    BatteryData battery;

    /* Estimated state */
    AttitudeData attitude;
    NavigationData navigation;

    /* Mission state */
    CansatMissionState missionState;

    /* Hardware state */
    SwitchState switches;
    ActuatorState actuators;

} Cansat;


void Cansat_Init(Cansat *cansat);

#endif