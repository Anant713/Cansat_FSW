#pragma once

#include "apps/Estimation.h"
#include "dcs_flight_config.h"
#include "dcs_runtime.h"

bool Estimation_MakeRuntimeInput(const Cansat *cansat, DcsRuntimeInput *input);
const DcsFlightConfig *Estimation_GetFlightConfig(void);
