#pragma once
#include "apps/Cansat.h"
#define BAROMETRIC_MAX_SAMPLE_AGE_MS 1000U
/* ISA pressure-ratio estimate relative to a fixed launch pressure.
 * First fresh launch-pad sample supplies the BMP280 startup average to persist.
 * A previously persisted mission reference takes priority after a reset. */
void BarometricAltitude_Update(Cansat *cansat);
