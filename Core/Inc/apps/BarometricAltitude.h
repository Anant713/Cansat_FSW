#pragma once
#include "apps/Cansat.h"
#define BAROMETRIC_MAX_SAMPLE_AGE_MS 1000U
/* ISA pressure-ratio estimate relative to a fixed launch pressure.
 * First fresh launch-pad sample establishes and persists the reference. */
void BarometricAltitude_Update(Cansat *cansat);
