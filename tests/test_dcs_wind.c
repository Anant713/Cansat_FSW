#include "../dcs_wind.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 0.01f;
}

int main(void)
{
    const DcsWindConfig config = {
        .horizontal_airspeed_mps = 6.5f,
        .ema_alpha = 0.25f,
        .max_heading_rate_rad_s = 0.1f,
        .max_brake_command = 0.1f,
        .max_course_rate_rad_s = 0.1f,
        .min_ground_speed_mps = 0.5f,
        .max_heading_age_ms = 200U,
        .max_fix_interval_ms = 2000U,
    };
    DcsWindState state;
    DcsWindAttitude attitude = {
        .system_time_ms = 1000U,
        .payload_heading_rad = 0.0f,
        .heading_rate_rad_s = 0.0f,
        .valid = true,
    };
    DcsWindGnssFix fix = {
        .gnss_epoch_ms = 1000U,
        .system_time_ms = 1000U,
        .ground_v_north_mps = 7.5f,
        .ground_v_east_mps = 0.0f,
        .valid = true,
    };

    assert(dcs_wind_init(&state, &config));
    assert(!state.has_estimate && state.last_update_ms == 0U);
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(!state.has_estimate);
    assert(state.last_update_ms == 0U);
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));

    fix.gnss_epoch_ms = 2000U;
    fix.system_time_ms = 2000U;
    attitude.system_time_ms = 2000U;
    assert(dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(near(state.north_mps, 0.25f));
    assert(near(state.east_mps, 0.0f));
    assert(state.has_estimate && state.last_update_ms == 2000U);
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(state.last_update_ms == 2000U); /* Duplicate fixes do not refresh wind. */

    fix.gnss_epoch_ms = 3000U;
    fix.system_time_ms = 3000U;
    fix.ground_v_north_mps = 9.5f;
    attitude.system_time_ms = 3000U;
    assert(dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(near(state.north_mps, 0.9375f));
    assert(state.last_update_ms == 3000U);

    fix.gnss_epoch_ms = 4000U;
    fix.system_time_ms = 4000U;
    attitude.system_time_ms = 4000U;
    attitude.heading_rate_rad_s = 0.2f;
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(state.last_update_ms == 3000U);
    assert(near(state.north_mps, 0.9375f));

    fix.gnss_epoch_ms = 5000U;
    fix.system_time_ms = 5000U;
    attitude.system_time_ms = 5000U;
    attitude.heading_rate_rad_s = 0.0f;
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.2f));
    assert(state.last_update_ms == 3000U);
    assert(near(state.north_mps, 0.9375f));

    fix.gnss_epoch_ms = 6000U;
    fix.system_time_ms = 6000U;
    attitude.system_time_ms = 5500U;
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(state.last_update_ms == 3000U);
    assert(near(state.north_mps, 0.9375f));

    /* A course change across +/-pi is a small turn, not a full rotation. */
    assert(dcs_wind_init(&state, &config));
    fix.gnss_epoch_ms = 7000U;
    fix.system_time_ms = 7000U;
    fix.ground_v_north_mps = -7.5f;
    fix.ground_v_east_mps = 0.1f;
    attitude.system_time_ms = 7000U;
    attitude.payload_heading_rad = 3.14159265f;
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    fix.gnss_epoch_ms = 8000U;
    fix.system_time_ms = 8000U;
    fix.ground_v_east_mps = -0.1f;
    attitude.system_time_ms = 8000U;
    assert(dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));

    /* A valid zero-wind estimate differs from no estimate, even at tick zero. */
    assert(dcs_wind_init(&state, &config));
    fix = (DcsWindGnssFix){
        .gnss_epoch_ms = 1000U,
        .system_time_ms = UINT32_MAX - 999U,
        .ground_v_north_mps = config.horizontal_airspeed_mps,
        .valid = true,
    };
    attitude = (DcsWindAttitude){
        .system_time_ms = fix.system_time_ms,
        .valid = true,
    };
    assert(!dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    fix.gnss_epoch_ms = 2000U;
    fix.system_time_ms = attitude.system_time_ms = 0U;
    assert(dcs_wind_update(&state, &config, &fix, &attitude, 0.0f));
    assert(state.has_estimate && state.last_update_ms == 0U);
    assert(near(state.north_mps, 0.0f) && near(state.east_mps, 0.0f));
    assert((uint32_t)(250U - state.last_update_ms) == 250U);

    puts("dcs_wind tests passed");
    return 0;
}
