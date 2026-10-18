#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drivers/GNSS.h"
#include "drivers/BMP.h"
#include "drivers/bmi270.h"
#include "drivers/AHT20.h"
#include "drivers/IIS2MDC.h"
#include "drivers/Battery.h"
#include "drivers/ReactionWheel.h"
#include "drivers/Servo.h"
#include "drivers/Switches.h"
#include "drivers/Camera.h"

/* ---------- Mission state ---------- */

typedef enum
{
    CANSAT_STATE_LAUNCH_PAD = 0,
    CANSAT_STATE_ASCENT,
    CANSAT_STATE_DESCENT_1,
    CANSAT_STATE_DESCENT_2,
    CANSAT_STATE_TOUCHDOWN,
    CANSAT_STATE_UNINITIALIZED = 255 /* Boot/recovery failure; not a flight state. */
} CansatMissionState;


/* ---------- Attitude ---------- */

typedef struct
{
    /* Euler angles in degrees; Euler rates in degrees/second. */
    float roll;
    float pitch;
    float yaw;

    float rollRate;
    float pitchRate;
    float yawRate;

    float quaternion[4];          /* Dimensionless, scalar first: w, x, y, z. */
    float angularVelocity[3];     /* Estimated body X/Y/Z rates, degrees/second. */
    bool quaternionValid;
    bool angularVelocityValid;
} AttitudeData;


/* ---------- Navigation / state estimation ---------- */

typedef struct
{
    float altitude;               /* metres relative to launch ground */
    float groundPressurePa;       /* Persisted barometric launch reference; zero = unset. */
    float verticalVelocity;       /* m/s */
    float horizontalVelocity;     /* m/s */

    double latitude;              /* signed decimal degrees */
    double longitude;             /* signed decimal degrees */

    float targetDistance;         /* metres to landing target */
    float headingError;           /* signed degrees */
    bool altitudeValid;
    uint32_t altitudeTimestampMs; /* Producer timestamps fresh launch-relative estimates. */
    bool targetDistanceValid;
    bool headingErrorValid;
} NavigationData;


/* ---------- Actuator state ---------- */

typedef struct
{
    ReactionWheelData reactionWheel;
    ServoData servo;
    bool servoValid;              /* A driver command snapshot is available. */
    bool parafoilDeployed;        /* Set by deployment/state-machine operation. */
} ActuatorState;

/* ---------- Telemetry / command bookkeeping ---------- */

#define CANSAT_TEAM_ID_MAX 27U
#define CANSAT_DEFAULT_TEAM_ID "2026-IN-SPACe-CAN-7USAT-016"

typedef struct
{
    char teamId[CANSAT_TEAM_ID_MAX + 1U]; /* Includes room for terminating NUL. */
    uint32_t uptimeMs;            /* milliseconds; packed as unsigned 24-bit on wire */
    uint16_t packetCount;         /* Packets accepted by UART TX, modulo 65536. */
    uint16_t receivedPacketCount; /* Complete ground packets, NOT UART chunks. */
    uint8_t lastReceivedCommand;  /* Command IDs to be defined. */
    bool commandExecuted;
    bool enabled;                /* Future SEND_TELEM/state logic controls this. */
} TelemetryState;

/* ---------- System status ---------- */

typedef struct
{
    bool bmpOK;
    bool ahtOK;
    bool bmiOK;
    bool magOK;
    bool gnssOK;
    bool sdCardOK;
    bool xbeeOK;

    bool reactionWheelOK;
    bool servoOK;

} SystemStatus;

/* ---------- CANSAT system state ---------- */

typedef struct Cansat
{
    /* Native sensor units; shared Telemetry serialization scales for XBee and CSV logging. */
    GNSSData gnss;     /* UTC HHMMSS + ms, signed degrees, altitude m MSL, speed m/s, course deg */
    BMP280Data bmp;    /* temperature degrees C, pressure Pa */
    AHT20Data env;     /* humidity %RH */
    BMI270Data bmi;    /* acceleration m/s^2 (includes gravity), gyro degrees/s */
    IIS2MDCData mag;   /* magnetic field microtesla */
    BatteryData battery; /* voltage V, raw ADC counts */

    /* Estimated state */
    AttitudeData attitude;
    NavigationData navigation;

    /* Mission state */
    CansatMissionState missionState;
    TelemetryState telemetry;

    /* Hardware state */
    CameraState camera;
    SwitchState switches;
    ActuatorState actuators;

} Cansat;


/* Clear data and set the default team ID; hardware startup and mission recovery
 * are separate. Camera flags start false; telemetry stays disabled until
 * explicitly enabled. */
void Cansat_Init(Cansat *cansat);
