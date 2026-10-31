#pragma once
#include "apps/Cansat.h"
#define BAROMETRIC_MAX_SAMPLE_AGE_MS 1000U
/* ISA pressure-ratio estimate relative to a fixed launch pressure.
 * Uses the same explicitly calibrated/restored BMP reference as height gates.
 * No automatic ground calibration or flash writes. */
void BarometricAltitude_Update(Cansat *cansat);
