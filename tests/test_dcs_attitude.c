#include "../dcs_attitude.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define G 9.80665f

static void close_to(float actual, float expected, float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual %.9g, expected %.9g, tolerance %.9g\n",
                actual, expected, tolerance);
        assert(false);
    }
}

static DcsAttitudeConfig bench_config(void)
{
    return (DcsAttitudeConfig){
        .sample_rate_hz = 100.0f,
        .gyro_range_deg_s = 2000.0f,
        .gain = 0.5f,
        .accel_rejection_deg = 10.0f,
        .mag_rejection_deg = 10.0f,
        .recovery_timeout_s = 5.0f,
        .min_accel_g = 0.8f,
        .max_accel_g = 1.2f,
        .calibrated_mag_strength = 50.0f,
        .max_mag_relative_error = 0.2f,
        .max_step_s = 0.02f,
        .max_heading_reference_age_ms = 1000U,
    };
}

static DcsAttitudeSample level_sample(uint32_t time_ms)
{
    return (DcsAttitudeSample){
        .system_time_ms = time_ms,
        .accel_mps2 = {0.0f, 0.0f, -G},
        .mag_calibrated = {40.0f, 0.0f, 30.0f},
        .gyro_valid = true,
        .accel_valid = true,
        .mag_valid = true,
        .accel_correction_allowed = true,
        .mag_correction_allowed = true,
    };
}

/* Independent Euler rotation matrix transpose: synthesize body observations. */
static DcsVec3 body_from_ned(float roll, float pitch, float yaw, DcsVec3 v)
{
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);
    return (DcsVec3){
        cp * cy * v.x + cp * sy * v.y - sp * v.z,
        (sr * sp * cy - cr * sy) * v.x +
            (sr * sp * sy + cr * cy) * v.y + sr * cp * v.z,
        (cr * sp * cy + sr * sy) * v.x +
            (cr * sp * sy - sr * cy) * v.y + cr * cp * v.z,
    };
}

static void test_known_pose_and_declination(void)
{
    DcsAttitudeConfig config = bench_config();
    DcsAttitudeState state;
    DcsAttitudeSample sample = level_sample(1000U);
    config.initial_roll_rad = PI / 6.0f;
    config.initial_pitch_rad = -PI / 9.0f;
    config.initial_heading_rad = PI / 3.0f;
    config.magnetic_declination_rad = PI / 10.0f;
    assert(dcs_attitude_init(&state, &config, 1000U));
    assert(state.orientation_valid && state.heading_valid && state.initialized);
    assert(!state.fusion_startup);
    close_to(state.payload_heading_rad, config.initial_heading_rad, 0.0001f);
    /* Build the magnetic field in TRUE NED, then transform into body axes. */
    sample.accel_mps2 = body_from_ned(
        config.initial_roll_rad, config.initial_pitch_rad,
        config.initial_heading_rad, (DcsVec3){0.0f, 0.0f, -G});
    sample.mag_calibrated = body_from_ned(
        config.initial_roll_rad, config.initial_pitch_rad,
        config.initial_heading_rad,
        (DcsVec3){40.0f * cosf(config.magnetic_declination_rad),
                  40.0f * sinf(config.magnetic_declination_rad), 30.0f});
    DcsQuaternion initial = state.body_to_ned;
    for (int i = 0; i < 200; ++i) {
        sample.system_time_ms += 10U;
        assert(dcs_attitude_update(&state, &config, &sample));
        assert(state.accel_correction_used && state.mag_correction_used);
    }
    close_to(state.payload_heading_rad, config.initial_heading_rad, 0.0001f);
    close_to(state.body_to_ned.w, initial.w, 0.0001f);
    close_to(state.body_to_ned.x, initial.x, 0.0001f);
    close_to(state.body_to_ned.y, initial.y, 0.0001f);
    close_to(state.body_to_ned.z, initial.z, 0.0001f);
}

static void test_gyro_rotation_all_axes(void)
{
    DcsAttitudeConfig config = bench_config();
    for (int axis = 0; axis < 3; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            DcsAttitudeState state;
            DcsAttitudeSample sample = level_sample(0U);
            float *rate = axis == 0 ? &sample.gyro_rad_s.x :
                          axis == 1 ? &sample.gyro_rad_s.y : &sample.gyro_rad_s.z;
            sample.accel_valid = false;
            sample.mag_valid = false;
            *rate = (float)sign * PI / 2.0f;
            assert(dcs_attitude_init(&state, &config, 0U));
            for (int i = 0; i < 100; ++i) {
                sample.system_time_ms += 10U;
                assert(dcs_attitude_update(&state, &config, &sample));
                assert(!state.accel_correction_used && !state.mag_correction_used);
            }
            close_to(state.body_to_ned.w, sqrtf(0.5f), 0.0001f);
            close_to(state.body_to_ned.x, axis == 0 ? sign * sqrtf(0.5f) : 0.0f, 0.0001f);
            close_to(state.body_to_ned.y, axis == 1 ? sign * sqrtf(0.5f) : 0.0f, 0.0001f);
            close_to(state.body_to_ned.z, axis == 2 ? sign * sqrtf(0.5f) : 0.0f, 0.0001f);
            assert(state.orientation_valid);
            if (axis == 1) {
                assert(!state.heading_valid); /* Forward axis is now vertical. */
            }
        }
    }
}

static void test_timestamps_and_failures(void)
{
    DcsAttitudeConfig config = bench_config();
    DcsAttitudeState state;
    DcsAttitudeSample sample = level_sample(1010U);
    unsigned char previous[sizeof state];
    config.initial_heading_rad = PI / 2.0f;
    assert(dcs_attitude_init(&state, &config, 1000U));
    sample.accel_valid = sample.mag_valid = false;
    sample.gyro_rad_s.z = 0.5f;
    assert(dcs_attitude_update(&state, &config, &sample));
    close_to(state.payload_heading_rad, PI / 2.0f + 0.005f, 0.0001f);
    memcpy(previous, &state, sizeof state);
    sample.gyro_rad_s.z = 10.0f;
    assert(!dcs_attitude_update(&state, &config, &sample));
    assert(memcmp(previous, &state, sizeof state) == 0);

    sample.system_time_ms += 20U; /* Exactly the permitted maximum interval. */
    sample.gyro_rad_s.z = 0.5f;
    assert(dcs_attitude_update(&state, &config, &sample));
    close_to(state.payload_heading_rad, PI / 2.0f + 0.015f, 0.0001f);
    sample.system_time_ms += 21U;
    assert(!dcs_attitude_update(&state, &config, &sample));
    assert(!state.initialized && !state.orientation_valid && !state.heading_valid);
    sample.system_time_ms += 10U;
    assert(!dcs_attitude_update(&state, &config, &sample));

    assert(dcs_attitude_init(&state, &config, UINT32_MAX - 4U));
    sample.system_time_ms = 5U; /* Ten milliseconds across counter rollover. */
    sample.gyro_rad_s.z = 0.5f;
    assert(dcs_attitude_update(&state, &config, &sample));
    close_to(state.payload_heading_rad, PI / 2.0f + 0.005f, 0.0001f);

    sample.system_time_ms += 10U;
    sample.gyro_valid = false;
    assert(!dcs_attitude_update(&state, &config, &sample));
    assert(!state.orientation_valid && !state.heading_valid);
    assert(!state.accel_correction_used && !state.mag_correction_used);
    sample.gyro_valid = true;
    sample.gyro_rad_s.z = FLT_MAX; /* rad/s -> degrees/s must not overflow into AHRS. */
    assert(!dcs_attitude_update(&state, &config, &sample));
}

static void test_reference_checks(void)
{
    DcsAttitudeConfig config = bench_config();
    DcsAttitudeState state;
    DcsAttitudeSample sample = level_sample(0U);
    assert(dcs_attitude_init(&state, &config, 0U));
    for (int i = 0; i < 6000; ++i) { /* One minute of noisy near-free-fall inputs. */
        sample.system_time_ms += 10U;
        sample.accel_mps2 = (DcsVec3){0.001f, -0.002f, 0.003f};
        sample.mag_valid = false;
        assert(dcs_attitude_update(&state, &config, &sample));
        assert(!state.accel_reference_available && !state.accel_correction_used);
        assert(!state.accel_recovery && state.orientation_valid);
    }
    close_to(state.body_to_ned.w, 1.0f, 0.0001f);
    assert(!state.heading_valid);
    sample.system_time_ms += 10U;
    sample.accel_mps2 = (DcsVec3){0.0f, 0.0f, -2.0f * G};
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(!state.accel_reference_available);
    sample.system_time_ms += 10U;
    sample.accel_mps2 = (DcsVec3){NAN, 0.0f, -G};
    sample.mag_valid = true;
    sample.mag_calibrated = (DcsVec3){INFINITY, 0.0f, 30.0f};
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(!state.accel_reference_available && !state.mag_reference_available);
    sample.system_time_ms += 10U;
    sample.accel_mps2 = (DcsVec3){0.0f, 0.0f, -G};
    sample.mag_calibrated = (DcsVec3){0.0f, 0.0f, 50.0f}; /* No horizontal field. */
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.accel_correction_used && !state.mag_reference_available);
    sample.system_time_ms += 10U;
    sample.mag_calibrated = (DcsVec3){40.0f, 0.0f, 30.0f};
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.mag_correction_used && state.heading_valid);
    sample.system_time_ms += 10U;
    sample.accel_correction_allowed = sample.mag_correction_allowed = false;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(!state.accel_correction_used && !state.mag_correction_used);
}

static void test_angular_rejection_and_recovery(void)
{
    DcsAttitudeConfig config = bench_config();
    for (int magnetic = 0; magnetic < 2; ++magnetic) {
        DcsAttitudeState state;
        DcsAttitudeSample sample = level_sample(0U);
        assert(dcs_attitude_init(&state, &config, 0U));
        if (magnetic) {
            sample.mag_calibrated = (DcsVec3){40.0f * cosf(PI / 6.0f), -20.0f, 30.0f};
        } else {
            sample.accel_mps2 = (DcsVec3){G * 0.5f, 0.0f, -G * cosf(PI / 6.0f)};
            sample.mag_valid = false;
        }
        for (int i = 0; i < 500; ++i) {
            sample.system_time_ms += 10U;
            assert(dcs_attitude_update(&state, &config, &sample));
            if (magnetic) {
                assert(state.mag_reference_available && !state.mag_correction_used);
                assert(!state.mag_recovery);
            } else {
                assert(state.accel_reference_available && !state.accel_correction_used);
                assert(!state.accel_recovery);
            }
        }
        close_to(state.body_to_ned.w, 1.0f, 0.0001f);
        sample.system_time_ms += 10U;
        assert(dcs_attitude_update(&state, &config, &sample));
        if (magnetic) {
            assert(state.mag_recovery && state.mag_correction_used);
            assert(state.payload_heading_rad > 0.0f);
            sample.mag_correction_allowed = false;
        } else {
            assert(state.accel_recovery && state.accel_correction_used);
            assert(state.body_to_ned.y > 0.0f);
            sample.accel_mps2 = (DcsVec3){0.0f, 0.0f, -2.0f * G};
        }
        sample.system_time_ms += 10U;
        assert(dcs_attitude_update(&state, &config, &sample));
        assert(magnetic ? !state.mag_correction_used : !state.accel_correction_used);
    }
}

static void test_overrange_and_configuration(void)
{
    DcsAttitudeConfig config = bench_config();
    DcsAttitudeState state;
    DcsAttitudeSample sample = level_sample(10U);
    config.initial_heading_rad = 0.7f;
    assert(dcs_attitude_init(&state, &config, 0U));
    sample.accel_valid = sample.mag_valid = false;
    sample.gyro_rad_s.z = 2001.0f * PI / 180.0f;
    assert(dcs_attitude_update(&state, &config, &sample));
    assert(state.gyro_overrange_recovery && state.fusion_startup);
    assert(state.payload_heading_rad > 0.7f); /* Preserves pose rather than resetting North. */
    sample.gyro_rad_s.z = 0.0f;
    for (int i = 0; i < 301; ++i) {
        sample.system_time_ms += 10U;
        assert(dcs_attitude_update(&state, &config, &sample));
    }
    assert(!state.fusion_startup && !state.gyro_overrange_recovery);

    config.initial_heading_rad = NAN;
    assert(!dcs_attitude_init(&state, &config, 0U));
    config = bench_config();
    config.recovery_timeout_s = 0.0f; /* Would silently disable upstream rejection. */
    assert(!dcs_attitude_init(&state, &config, 0U));
    config = bench_config();
    config.recovery_timeout_s = FLT_MAX; /* Protect upstream float -> int conversion. */
    assert(!dcs_attitude_init(&state, &config, 0U));
}

static void test_measured_intervals_and_heading_rate(void)
{
    DcsAttitudeConfig config = bench_config();
    DcsAttitudeState state;
    DcsAttitudeSample sample = level_sample(0U);
    sample.accel_valid = sample.mag_valid = false;
    sample.gyro_rad_s.z = 1.0f;
    assert(dcs_attitude_init(&state, &config, 0U));
    const uint32_t intervals[] = {5U, 10U, 15U};
    for (unsigned int i = 0U; i < sizeof intervals / sizeof intervals[0]; ++i) {
        sample.system_time_ms += intervals[i];
        assert(dcs_attitude_update(&state, &config, &sample));
    }
    close_to(state.payload_heading_rad, 0.030f, 0.00001f);

    config.initial_roll_rad = PI / 6.0f;
    assert(dcs_attitude_init(&state, &config, 0U));
    sample.system_time_ms = 10U;
    sample.gyro_rad_s = (DcsVec3){0.0f, 0.5f, 0.0f};
    assert(dcs_attitude_update(&state, &config, &sample));
    /* Body Y rotation contributes to geographic heading when rolled. */
    close_to(state.payload_heading_rate_rad_s, 0.25f, 0.0001f);
}

int main(void)
{
    test_known_pose_and_declination();
    test_gyro_rotation_all_axes();
    test_timestamps_and_failures();
    test_reference_checks();
    test_angular_rejection_and_recovery();
    test_overrange_and_configuration();
    test_measured_intervals_and_heading_rate();
    puts("dcs_attitude Fusion integration tests passed");
    return 0;
}
