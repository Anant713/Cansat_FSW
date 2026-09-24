#ifndef BME_H
#define BME_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float temperature;
    float pressure;
    float humidity;
    float altitude;

    bool valid;

} BMEData;


bool BME_Init(void);

bool BME_ReceiveData(void);

bool BME_GetData(BMEData *data);

#endif