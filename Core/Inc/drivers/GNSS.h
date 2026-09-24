#ifndef GNSS_H
#define GNSS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t utcTime;

    double latitude;
    double longitude;

    char latitudeHemisphere;
    char longitudeHemisphere;

    float altitude;
    float speed;

    uint8_t satellites;

    bool fixValid;
    bool valid;

} GNSSData;


bool GNSS_Init(void);


/* Start/restart UART DMA reception */

bool GNSS_StartReception(void);


/* Called when UART/DMA receives data */

void GNSS_RxEventCallback(uint16_t size);


/* Parse data accumulated in the RX buffer */

void GNSS_Process(void);


/* Get latest complete GNSS solution */

bool GNSS_GetData(GNSSData *data);


/* Status */

bool GNSS_IsDataReady(void);

bool GNSS_IsFixValid(void);

#endif