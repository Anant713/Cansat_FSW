#ifndef BMI_H
#define BMI_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    bool valid;

} BMIData;


bool BMI_Init(void);

bool BMI_ReceiveData(void);

bool BMI_GetData(BMIData *data);

#endif