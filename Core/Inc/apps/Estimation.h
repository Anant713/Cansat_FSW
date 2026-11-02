#ifndef ESTIMATION_H
#define ESTIMATION_H

#include <stdbool.h>

#define ESTIMATION_BMI_MAX_AGE_MS 50U
#define ESTIMATION_VERTICAL_RATE_MAX_INTERVAL_MS 500U

typedef struct Cansat Cansat;


bool Estimation_Init(void);

void Estimation_Update(Cansat *cansat);


/* Attitude estimation */

bool AttitudeEstimator_Init(void);

void AttitudeEstimator_Update(Cansat *cansat);


/* Optional navigation/state estimation */

bool NavigationEstimator_Init(void);

void NavigationEstimator_Update(Cansat *cansat);

#endif
