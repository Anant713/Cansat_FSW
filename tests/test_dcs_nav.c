#include "../dcs_nav.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) < tolerance;
}

int main(void)
{
    const DcsNavConfig config = {
        .target_lat_e7 = 190000000,
        .target_lon_e7 = 730000000,
        .max_gnss_age_ms = 1500U,
        .max_baro_age_ms = 200U,
    };
    DcsNavState state;
    DcsNavGnssFix fix = {
        .gnss_epoch_ms = 1000U,
        .system_time_ms = 1000U,
        .lat_e7 = 190000000,
        .lon_e7 = 730000000,
        .ground_v_north_mps = 5.0f,
        .ground_v_east_mps = -2.0f,
        .valid = true,
    };
    DcsNavBaroSample baro = {
        .system_time_ms = 1000U,
        .height_above_target_m = 600.0f,
        .valid = true,
    };

    assert(dcs_nav_init(&state, &config));
    dcs_nav_step(&state, &config, 1000U, &fix, &baro);
    assert(state.output.horizontal_valid && state.output.vertical_valid);
    assert(near(state.output.north_m, 0.0f, 0.01f));
    assert(near(state.output.east_m, 0.0f, 0.01f));
    assert(near(state.output.down_m, -600.0f, 0.01f));

    dcs_nav_step(&state, &config, 1100U, NULL, NULL);
    assert(near(state.output.north_m, 0.5f, 0.01f));
    assert(near(state.output.east_m, -0.2f, 0.01f));
    assert(near(state.output.down_m, -600.0f, 0.01f));

    fix.gnss_epoch_ms = 2000U;
    fix.system_time_ms = 2000U;
    fix.lat_e7 += 9010; /* Approximately 100 metres north. */
    baro.system_time_ms = 2000U;
    baro.height_above_target_m = 597.0f;
    dcs_nav_step(&state, &config, 2000U, &fix, &baro);
    assert(near(state.output.north_m, 100.0f, 1.0f));
    assert(near(state.output.down_m, -597.0f, 0.01f));

    fix.gnss_epoch_ms = 2100U;
    fix.system_time_ms = 2050U;
    dcs_nav_step(&state, &config, 2100U, &fix, NULL);
    assert(near(state.output.north_m, 100.25f, 1.0f));

    dcs_nav_step(&state, &config, 2300U, NULL, NULL);
    assert(state.output.horizontal_valid);
    assert(!state.output.vertical_valid);
    dcs_nav_step(&state, &config, 3600U, NULL, NULL);
    assert(state.output.horizontal_valid);
    assert(!state.output.gnss_fresh);
    assert(state.output.horizontal_propagated);
    assert(near(state.output.north_m, 107.75f, 1.0f));

    puts("dcs_nav tests passed");
    return 0;
}
