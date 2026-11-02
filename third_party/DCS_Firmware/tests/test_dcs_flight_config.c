#include "../dcs_flight_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    DcsFlightConfig config;
    DcsNavState nav;
    DcsAttitudeState attitude;
    DcsRuntimeState runtime;
    DcsGuidanceState guidance;
    const DcsGuidanceInput guidance_input = {
        .position = {200.0f, 50.0f},
        .ground_v_north_mps = -6.0f,
        .ground_v_east_mps = -1.0f,
        .height_above_target_m = 600.0f,
        .horizontal_valid = true,
        .vertical_valid = true,
    };

    dcs_flight_config_load(&config);
    assert(config.navigation.target_lat_e7 == 267175350);
    assert(config.navigation.target_lon_e7 == 843015430);
    assert(fabsf(config.attitude.magnetic_declination_rad -
                 0.0082379541f) < 1.0e-9f);
    assert(dcs_nav_init(&nav, &config.navigation));
    assert(dcs_attitude_init(&attitude, &config.attitude, 0u));
    assert(dcs_runtime_start(&runtime, &config.runtime, 0u));
    assert(dcs_guidance_start(&guidance, &config.runtime.guidance,
                              &guidance_input));
    assert(config.runtime.left_servo.neutral_pulse_us == 556u);
    assert(config.runtime.left_servo.maximum_pull_pulse_us == 1000u);
    assert(config.runtime.right_servo.neutral_pulse_us == 556u);
    assert(fabsf(config.attitude.initial_heading_rad - 0.2617993878f) < 1.0e-6f);
    assert(fabsf(config.attitude.calibrated_mag_strength - 48.4249f) < 1.0e-4f);
    assert(config.runtime.right_servo.maximum_pull_pulse_us == 1000u);
    puts("dcs_flight_config tests passed");
    return 0;
}
