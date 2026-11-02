#include "../dcs_circle_fit.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265358979323846f

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    const DcsCircleFitConfig config = {
        .minimum_samples = 12u,
        .maximum_samples = 32u,
        .minimum_direction_coverage_rad = 4.5f,
        .minimum_velocity_spread_mps = 1.0f,
        .maximum_rms_fraction = 0.1f,
        .minimum_airspeed_mps = 1.0f,
        .maximum_airspeed_mps = 10.0f,
        .minimum_turn_radius_m = 1.0f,
        .maximum_turn_radius_m = 100.0f,
        .minimum_turn_direction_consistency = 0.8f,
    };
    DcsCircleFitState state;
    DcsCircleFitResult result;
    assert(dcs_circle_fit_init(&state, &config));
    for (uint32_t i = 0u; i < 24u; ++i) {
        const float angle = 2.0f * PI_F * (float)i / 24.0f;
        assert(dcs_circle_fit_add(&state, &config, 200u * i,
            2.0f + 4.0f * cosf(angle), -3.0f + 4.0f * sinf(angle)));
    }
    assert(dcs_circle_fit_solve(&state, &config, &result));
    assert(near(result.wind_north_mps, 2.0f, 1.0e-3f));
    assert(near(result.wind_east_mps, -3.0f, 1.0e-3f));
    assert(near(result.maneuver_airspeed_mps, 4.0f, 1.0e-3f));
    assert(result.direction_coverage_rad > 5.5f);
    assert(near(result.turn_rate_rad_s, 1.308997f, 1.0e-3f));
    assert(near(result.turn_radius_m, 3.055775f, 1.0e-3f));
    assert(result.turn_direction_consistency > 0.99f);

    assert(!dcs_circle_fit_add(&state, &config, 4600u, 0.0f, 0.0f));

    DcsCircleFitState line;
    assert(dcs_circle_fit_init(&line, &config));
    for (uint32_t i = 0u; i < 12u; ++i) {
        assert(dcs_circle_fit_add(&line, &config, i, (float)i, 0.0f));
    }
    assert(!dcs_circle_fit_solve(&line, &config, &result));

    puts("dcs_circle_fit tests passed");
    return 0;
}
