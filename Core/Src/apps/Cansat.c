#include "apps/Cansat.h"

typedef struct
{
    SensorData sensors;

    GNSSData gnss;

    AttitudeData attitude;

    NavigationData navigation;

    BatteryData battery;

    ActuatorState actuators;

    SwitchState switches;

    MissionState mission;

    SystemStatus system;

} Cansat;