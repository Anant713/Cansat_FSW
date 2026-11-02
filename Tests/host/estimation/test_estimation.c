#include "apps/Cansat.h"
#include "apps/EstimationDcs.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

static void close_to(float actual, float expected, float tolerance)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    Cansat c;
    Cansat_Init(&c);
    tick = 100U;
    assert(Estimation_Init());

    c.bmi = (BMI270Data){
        .accelZ = -BMI270_GRAVITY_MPS2,
        .gyroX = 1.0f, .gyroY = 2.0f, .gyroZ = 3.0f,
        .timestamp_ms = 110U, .valid = true,
    };
    c.gnss = (GNSSData){
        .latitude = 26.717535, .longitude = 84.301543,
        .speed = 10.0f, .course = 0.0f,
        .ggaReceivedMs = 110U, .vtgReceivedMs = 110U,
        .valid = true, .fixValid = true, .speedValid = true,
        .courseValid = true, .vtgValid = true,
    };
    c.navigation.altitude = 100.0f;
    c.navigation.altitudeTimestampMs = 110U;
    c.navigation.altitudeValid = true;
    tick = 110U;
    Estimation_Update(&c);

    assert(c.attitude.quaternionValid && c.attitude.angularVelocityValid);
    float norm = 0.0f;
    for (unsigned i = 0; i < 4U; ++i) norm += c.attitude.quaternion[i] * c.attitude.quaternion[i];
    close_to(sqrtf(norm), 1.0f, 0.001f);
    close_to(c.attitude.angularVelocity[0], 0.5f - 1.7320508f, 0.001f);
    close_to(c.attitude.angularVelocity[1], -0.8660254f - 1.0f, 0.001f);
    close_to(c.attitude.angularVelocity[2], -3.0f, 0.001f);
    assert(c.attitude.timestampMs == 110U);
    assert(c.navigation.targetDistanceValid);
    assert(c.navigation.horizontalVelocityValid);
    close_to(c.navigation.horizontalVelocity, 10.0f, 0.001f);
    assert(!c.navigation.verticalVelocityValid);

    c.bmi.timestamp_ms = 120U;
    c.navigation.altitude = 101.0f;
    c.navigation.altitudeTimestampMs = 120U;
    tick = 120U;
    Estimation_Update(&c);
    assert(c.navigation.verticalVelocityValid);
    close_to(c.navigation.verticalVelocity, 100.0f, 0.01f);

    DcsRuntimeInput input;
    assert(Estimation_MakeRuntimeInput(&c, &input));
    assert(input.heading_valid == c.attitude.quaternionValid);
    assert(input.gnss_epoch_ms == c.gnss.vtgReceivedMs);
    assert(input.gnss_system_time_ms == c.gnss.vtgReceivedMs);

    tick = c.bmi.timestamp_ms + ESTIMATION_BMI_MAX_AGE_MS + 1U;
    Estimation_Update(&c);
    assert(!c.attitude.quaternionValid && !c.attitude.angularVelocityValid);
    assert(Estimation_MakeRuntimeInput(&c, &input) && !input.heading_valid);

    puts("Estimation integration passed: quaternion, body rates, navigation velocities and stale invalidation");
    return 0;
}
