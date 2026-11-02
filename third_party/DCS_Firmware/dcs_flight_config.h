#ifndef DCS_FLIGHT_CONFIG_H
#define DCS_FLIGHT_CONFIG_H

#include "dcs_attitude.h"
#include "dcs_nav.h"
#include "dcs_runtime.h"

typedef struct {
    DcsNavConfig navigation;
    DcsAttitudeConfig attitude;
    DcsRuntimeConfig runtime;
} DcsFlightConfig;

/* Loads the single frozen configuration intended for the current flight. */
void dcs_flight_config_load(DcsFlightConfig *config);

#endif
