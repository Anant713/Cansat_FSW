#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct Cansat Cansat;


/* Initialization */

void Communication_Init(void);


/* Process incoming XBee commands */

void Communication_Update(Cansat *cansat);


/* Telemetry */

bool Communication_SendTelemetry(const Cansat *cansat);


/* Command processing */

bool Communication_CommandAvailable(void);

void Communication_ProcessCommand(Cansat *cansat);

#endif