#include "../dcs_attitude.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    DcsAttitudeConfig config = {
        .accel_gain_per_s = 1.0f,
        .mag_gain_per_s = 1.0f,
        .max_accel_error_mps2 = 1.0f,
        .calibrated_mag_strength = 50.0f,
        .max_mag_relative_error = 0.2f,
        .magnetic_declination_rad = 0.0f,
        .max_step_s = 0.02f,
        .max_heading_reference_age_ms = 1000U,
    };
    DcsAttitudeState state;
    DcsAttitudeSample sample = {
        .system_time_ms = 1000U,
        .gyro_rad_s = {0.0f, 0.0f, 0.0f},
        .accel_mps2 = {0.0f, 0.0f, -9.80665f},
        .mag_calibrated = {40.0f, 0.0f, 30.0f},
        .gyro_valid = true,
        .accel_valid = true,
        .mag_valid = true,
        .accel_correction_allowed = true,
        .mag_correction_allowed = true,
    };

    assert(dcs_attitude_init(&state, &config, 990U));
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.heading_valid);
    assert(fabsf(state.payload_heading_rad) < 0.01f);

    sample.system_time_ms += 10U;
    sample.accel_mps2.x = 9.80665f;
    sample.accel_mps2.z = 0.0f;
    sample.accel_correction_allowed = false;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(!state.accel_correction_used);

    /* Gyro integration remains available during invalid accel/mag samples. */
    sample.accel_valid = false;
    sample.mag_valid = false;
    sample.gyro_rad_s.z = 1.57079633f;
    for (int i = 0; i < 101; ++i) {
        sample.system_time_ms += 10U;
        assert(dcs_attitude_update(&state, &config, &sample));
    }
    assert(fabsf(state.payload_heading_rad - 1.57079633f) < 0.03f);
    assert(!state.heading_valid); /* No recent magnetic reference. */

    /* Known East pose agrees with the supplied magnetic reference. */
    config.initial_heading_rad = 1.57079633f;
    assert(dcs_attitude_init(&state, &config, 2990U));
    sample.system_time_ms = 3000U;
    sample.gyro_rad_s.z = 0.0f;
    sample.accel_mps2 = (DcsVec3){0.0f, 0.0f, -9.80665f};
    sample.mag_calibrated = (DcsVec3){0.0f, -40.0f, 30.0f};
    sample.accel_valid = true;
    sample.mag_valid = true;
    sample.accel_correction_allowed = true;
    sample.mag_correction_allowed = true;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(fabsf(state.payload_heading_rad - 1.57079633f) < 0.01f);

    /* Level magnetic reference pulls an incorrect yaw toward North. */
    sample.system_time_ms += 10U;
    sample.mag_calibrated = (DcsVec3){40.0f, 0.0f, 30.0f};
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.payload_heading_rad < 1.57079633f);

    /* Known 30-degree roll agrees with tilted sensor observations. */
    config.initial_heading_rad = 0.0f;
    config.initial_roll_rad = 0.52359878f;
    assert(dcs_attitude_init(&state, &config, 3990U));
    sample.system_time_ms = 4000U;
    sample.accel_mps2 = (DcsVec3){0.0f, -4.903325f, -8.49281f};
    sample.mag_calibrated = (DcsVec3){40.0f, 15.0f, 25.980762f};
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(fabsf(state.payload_heading_rad) < 0.01f);
    assert(fabsf(state.body_to_ned.x - 0.258819f) < 0.01f);

    sample.system_time_ms += 10U;
    sample.accel_mps2 = (DcsVec3){0.0f, 0.0f, -9.80665f};
    sample.mag_valid = false;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.body_to_ned.x < 0.258819f);

    sample.system_time_ms += 10U;
    sample.gyro_valid = false;
    assert(!dcs_attitude_update(&state, &config, &sample));
    assert(!state.heading_valid);

    /* Known pose initializes without accelerometer or magnetometer samples. */
    config.initial_heading_rad = 1.57079633f;
    config.initial_pitch_rad = 0.0f;
    config.initial_roll_rad = 0.0f;
    assert(dcs_attitude_init(&state, &config, 5000U));
    assert(state.initialized && state.orientation_valid && state.heading_valid);
    assert(fabsf(state.payload_heading_rad - 1.57079633f) < 0.0001f);
    assert(fabsf(state.body_to_ned.w - 0.70710678f) < 0.0001f);
    assert(fabsf(state.body_to_ned.z - 0.70710678f) < 0.0001f);
    sample = (DcsAttitudeSample){
        .system_time_ms = 5010U,
        .gyro_rad_s = {0.0f, 0.0f, 0.5f},
        .gyro_valid = true,
    };
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.heading_valid);
    assert(fabsf(state.payload_heading_rad - (1.57079633f + 0.005f)) < 0.0001f);

    /* Repeated timestamps must not integrate or invalidate the estimate. */
    {
        DcsAttitudeState previous = state;
        sample.gyro_rad_s.z = 10.0f;
        assert(!dcs_attitude_update(&state, &config, &sample));
        assert(state.initialized && state.orientation_valid && state.heading_valid);
        assert(state.last_sample_ms == previous.last_sample_ms);
        assert(state.last_heading_reference_ms == previous.last_heading_reference_ms);
        assert(state.body_to_ned.w == previous.body_to_ned.w);
        assert(state.body_to_ned.x == previous.body_to_ned.x);
        assert(state.body_to_ned.y == previous.body_to_ned.y);
        assert(state.body_to_ned.z == previous.body_to_ned.z);
        assert(state.payload_heading_rad == previous.payload_heading_rad);
        assert(state.payload_heading_rate_rad_s == previous.payload_heading_rate_rad_s);
    }
    sample.system_time_ms += 10U;
    sample.gyro_rad_s.z = 0.5f;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(fabsf(state.payload_heading_rad - (1.57079633f + 0.010f)) < 0.0001f);

    /* A large gap cannot silently restore the launch-pad pose in flight. */
    sample.system_time_ms += 100U;
    assert(!dcs_attitude_update(&state, &config, &sample));
    assert(!state.initialized && !state.orientation_valid && !state.heading_valid);
    sample.system_time_ms += 10U;
    assert(!dcs_attitude_update(&state, &config, &sample));

    config.initial_heading_rad = NAN;
    assert(!dcs_attitude_init(&state, &config, 6000U));

    puts("dcs_attitude tests passed");
    return 0;
}
