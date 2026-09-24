#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float magX;
    float magY;
    float magZ;

    bool valid;

} IIS2MDCData;


bool IIS2MDC_Init(void);

bool IIS2MDC_ReceiveData(void);

bool IIS2MDC_GetData(IIS2MDCData *data);

#endif