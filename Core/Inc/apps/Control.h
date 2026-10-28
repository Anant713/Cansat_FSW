#include "apps/Cansat.h"

#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

typedef struct Cansat Cansat;


bool Control_Init(void);

void Control_Update(Cansat *cansat);


/* Parafoil / DCS guidance */

bool DCSGuidance_Init(void);

void DCSGuidance_Update(Cansat *cansat);


/* Parafoil controller */

bool DCSController_Init(void);

void DCSController_Update(Cansat *cansat);

#endif
