#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint16_t adcRaw;
    float voltage;

    bool valid;

} BatteryData;


bool Battery_Init(void);

bool Battery_Read(void);

bool Battery_GetData(BatteryData *data);

float Battery_GetVoltage(void);

#endif