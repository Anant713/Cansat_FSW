#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include <stdbool.h>

typedef struct Cansat Cansat;

void SensorAcquisition_Init(void);

void SensorAcquisition_Update(Cansat *cansat);

/* Optional individual updates */

bool SensorAcquisition_UpdateBME(Cansat *cansat);

bool SensorAcquisition_UpdateBMI(Cansat *cansat);

bool SensorAcquisition_UpdateMag(Cansat *cansat);

bool SensorAcquisition_UpdateGNSS(Cansat *cansat);

bool SensorAcquisition_UpdateBattery(Cansat *cansat);

#endif