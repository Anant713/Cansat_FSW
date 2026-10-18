#include "apps/BarometricAltitude.h"
#include "apps/StateMachine.h"
#include "main.h"
#include <math.h>

void BarometricAltitude_Update(Cansat *c)
{
    if (!c) return;
    c->navigation.altitudeValid = false;
    float pressure = c->bmp.pressure;
    if (!c->bmp.valid || !isfinite(pressure) || pressure < 30000.0f ||
        pressure > 110000.0f ||
        (uint32_t)(HAL_GetTick() - c->bmp.timestamp_ms) > BAROMETRIC_MAX_SAMPLE_AGE_MS)
        return;
    if (c->navigation.groundPressurePa == 0.0f &&
        !StateMachine_SetGroundPressure(c, pressure)) return;
    float reference = c->navigation.groundPressurePa;
    if (!isfinite(reference) || reference < 30000.0f || reference > 110000.0f) return;
    float altitude = 44330.0f * (1.0f - powf(pressure / reference, 0.19029495f));
    if (!isfinite(altitude)) return;
    c->navigation.altitude = altitude;
    c->navigation.altitudeTimestampMs = c->bmp.timestamp_ms;
    c->navigation.altitudeValid = true;
}
