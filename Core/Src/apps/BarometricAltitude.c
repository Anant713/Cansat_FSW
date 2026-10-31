#include "apps/BarometricAltitude.h"
#include "main.h"
#include <math.h>

void BarometricAltitude_Update(Cansat *c)
{
    if (!c) return;
    c->navigation.altitudeValid = false;
    float pressure = c->bmp.pressure;
    c->navigation.groundPressurePa = c->bmp.heightValid ? c->bmp.groundPressurePa : 0.0f;
    if (!c->bmp.valid || !c->bmp.heightValid || !isfinite(pressure) || pressure < 30000.0f ||
        pressure > 110000.0f ||
        (uint32_t)(HAL_GetTick() - c->bmp.timestamp_ms) > BAROMETRIC_MAX_SAMPLE_AGE_MS)
        return;
    /* Same restored driver reference as telemetry and state-machine height. */
    float reference = c->navigation.groundPressurePa;
    if (!isfinite(reference) || reference < 30000.0f || reference > 110000.0f) return;
    float altitude = 44330.0f * (1.0f - powf(pressure / reference, 0.1903f));
    if (!isfinite(altitude)) return;
    c->navigation.altitude = altitude;
    c->navigation.altitudeTimestampMs = c->bmp.timestamp_ms;
    c->navigation.altitudeValid = true;
}
