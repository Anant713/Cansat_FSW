#include "dcs_attitude.h"

#include <math.h>
#include <stddef.h>

#define DCS_GRAVITY_MPS2 9.80665f

static float norm3(DcsVec3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static bool finite3(DcsVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static DcsVec3 cross3(DcsVec3 a, DcsVec3 b)
{
    DcsVec3 result = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
    return result;
}

static DcsQuaternion multiply(DcsQuaternion a, DcsQuaternion b)
{
    DcsQuaternion result = {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
    return result;
}

static DcsVec3 rotate(DcsQuaternion q, DcsVec3 v)
{
    DcsVec3 qv = {q.x, q.y, q.z};
    DcsVec3 t = cross3(qv, v);
    DcsVec3 u;

    t.x *= 2.0f;
    t.y *= 2.0f;
    t.z *= 2.0f;
    u = cross3(qv, t);
    v.x += q.w * t.x + u.x;
    v.y += q.w * t.y + u.y;
    v.z += q.w * t.z + u.z;
    return v;
}

static DcsVec3 rotate_inverse(DcsQuaternion q, DcsVec3 v)
{
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return rotate(q, v);
}

static DcsQuaternion from_angles(float yaw, float pitch, float roll)
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    DcsQuaternion q = {
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    };
    return q;
}

static bool config_valid(const DcsAttitudeConfig *config)
{
    return config != NULL &&
           isfinite(config->initial_roll_rad) &&
           isfinite(config->initial_pitch_rad) &&
           isfinite(config->initial_heading_rad) &&
           isfinite(config->accel_gain_per_s) &&
           config->accel_gain_per_s >= 0.0f &&
           isfinite(config->mag_gain_per_s) &&
           config->mag_gain_per_s >= 0.0f &&
           isfinite(config->max_accel_error_mps2) &&
           config->max_accel_error_mps2 > 0.0f &&
           isfinite(config->calibrated_mag_strength) &&
           config->calibrated_mag_strength > 0.0f &&
           isfinite(config->max_mag_relative_error) &&
           config->max_mag_relative_error > 0.0f &&
           isfinite(config->magnetic_declination_rad) &&
           isfinite(config->max_step_s) && config->max_step_s > 0.0f &&
           config->max_heading_reference_age_ms > 0U;
}

static bool accel_usable(const DcsAttitudeConfig *config,
                         const DcsAttitudeSample *sample,
                         DcsVec3 accel_mps2)
{
    float magnitude;
    if (!sample->accel_valid || !sample->accel_correction_allowed ||
        !finite3(accel_mps2)) {
        return false;
    }
    magnitude = norm3(accel_mps2);
    return isfinite(magnitude) &&
           fabsf(magnitude - DCS_GRAVITY_MPS2) <=
               config->max_accel_error_mps2;
}

static bool mag_usable(const DcsAttitudeConfig *config,
                       const DcsAttitudeSample *sample,
                       DcsVec3 mag_calibrated)
{
    float magnitude;
    if (!sample->mag_valid || !sample->mag_correction_allowed ||
        !finite3(mag_calibrated)) {
        return false;
    }
    magnitude = norm3(mag_calibrated);
    return isfinite(magnitude) &&
           fabsf(magnitude - config->calibrated_mag_strength) <=
               config->calibrated_mag_strength *
                   config->max_mag_relative_error;
}

static void update_outputs(DcsAttitudeState *state, DcsVec3 gyro_rad_s)
{
    const DcsVec3 forward_body = {1.0f, 0.0f, 0.0f};
    DcsVec3 forward_ned = rotate(state->body_to_ned, forward_body);
    DcsVec3 omega_ned = rotate(state->body_to_ned, gyro_rad_s);
    DcsVec3 forward_dot = cross3(omega_ned, forward_ned);
    float horizontal_sq = forward_ned.x * forward_ned.x +
                          forward_ned.y * forward_ned.y;

    state->orientation_valid = true;
    if (horizontal_sq < 0.04f) {
        state->heading_valid = false;
        return;
    }
    state->heading_valid = true;
    state->payload_heading_rad = atan2f(forward_ned.y, forward_ned.x);
    state->payload_heading_rate_rad_s =
        (forward_ned.x * forward_dot.y -
         forward_ned.y * forward_dot.x) / horizontal_sq;
}

bool dcs_attitude_init(DcsAttitudeState *state,
                       const DcsAttitudeConfig *config,
                       uint32_t system_time_ms)
{
    if (state == NULL || !config_valid(config)) {
        return false;
    }
    *state = (DcsAttitudeState){0};
    state->body_to_ned = from_angles(config->initial_heading_rad,
                                    config->initial_pitch_rad,
                                    config->initial_roll_rad);
    state->last_sample_ms = system_time_ms;
    state->last_heading_reference_ms = system_time_ms;
    state->initialized = true;
    update_outputs(state, (DcsVec3){0.0f, 0.0f, 0.0f});
    return true;
}

bool dcs_attitude_update(DcsAttitudeState *state,
                         const DcsAttitudeConfig *config,
                         const DcsAttitudeSample *sample)
{
    bool use_accel;
    bool use_mag;
    uint32_t elapsed_ms;
    float dt_s;
    DcsVec3 gyro_rad_s;
    DcsVec3 accel_mps2;
    DcsVec3 mag_calibrated;
    DcsVec3 omega;

    if (state == NULL || !config_valid(config) || sample == NULL) {
        return false;
    }
    if (!state->initialized) {
        return false;
    }
    elapsed_ms = sample->system_time_ms - state->last_sample_ms;
    if (elapsed_ms == 0U) {
        return false;
    }
    if (!sample->gyro_valid || !finite3(sample->gyro_rad_s)) {
        state->orientation_valid = false;
        state->heading_valid = false;
        return false;
    }
    gyro_rad_s = sample->gyro_rad_s;
    accel_mps2 = sample->accel_mps2;
    mag_calibrated = sample->mag_calibrated;
    use_accel = accel_usable(config, sample, accel_mps2);
    use_mag = mag_usable(config, sample, mag_calibrated);
    state->accel_correction_used = false;
    state->mag_correction_used = false;

    if ((float)elapsed_ms * 0.001f > config->max_step_s) {
        state->orientation_valid = false;
        state->heading_valid = false;
        state->initialized = false;
        return false;
    }
    state->last_sample_ms = sample->system_time_ms;
    dt_s = (float)elapsed_ms * 0.001f;
    omega = gyro_rad_s;

    if (use_accel) {
        const DcsVec3 up_ned = {0.0f, 0.0f, -1.0f};
        DcsVec3 predicted_up = rotate_inverse(state->body_to_ned, up_ned);
        DcsVec3 measured_up = accel_mps2;
        DcsVec3 error;
        float magnitude = norm3(measured_up);
        measured_up.x /= magnitude;
        measured_up.y /= magnitude;
        measured_up.z /= magnitude;
        error = cross3(measured_up, predicted_up);
        omega.x += config->accel_gain_per_s * error.x;
        omega.y += config->accel_gain_per_s * error.y;
        omega.z += config->accel_gain_per_s * error.z;
        state->accel_correction_used = true;
    }

    if (use_mag) {
        DcsVec3 magnetic_ned = rotate(state->body_to_ned,
                                      mag_calibrated);
        float horizontal = hypotf(magnetic_ned.x, magnetic_ned.y);
        if (horizontal >= 0.1f * config->calibrated_mag_strength) {
            float azimuth = atan2f(magnetic_ned.y, magnetic_ned.x);
            float error = atan2f(
                sinf(config->magnetic_declination_rad - azimuth),
                cosf(config->magnetic_declination_rad - azimuth));
            const DcsVec3 correction_ned = {
                0.0f, 0.0f, config->mag_gain_per_s * error,
            };
            DcsVec3 correction_body = rotate_inverse(state->body_to_ned,
                                                       correction_ned);
            omega.x += correction_body.x;
            omega.y += correction_body.y;
            omega.z += correction_body.z;
            state->last_heading_reference_ms = sample->system_time_ms;
            state->mag_correction_used = true;
        }
    }

    {
        DcsQuaternion angular = {0.0f, omega.x, omega.y, omega.z};
        DcsQuaternion derivative = multiply(state->body_to_ned, angular);
        float qnorm;
        state->body_to_ned.w += 0.5f * dt_s * derivative.w;
        state->body_to_ned.x += 0.5f * dt_s * derivative.x;
        state->body_to_ned.y += 0.5f * dt_s * derivative.y;
        state->body_to_ned.z += 0.5f * dt_s * derivative.z;
        qnorm = sqrtf(state->body_to_ned.w * state->body_to_ned.w +
                      state->body_to_ned.x * state->body_to_ned.x +
                      state->body_to_ned.y * state->body_to_ned.y +
                      state->body_to_ned.z * state->body_to_ned.z);
        if (!isfinite(qnorm) || qnorm == 0.0f) {
            state->orientation_valid = false;
            state->heading_valid = false;
            state->initialized = false;
            return false;
        }
        state->body_to_ned.w /= qnorm;
        state->body_to_ned.x /= qnorm;
        state->body_to_ned.y /= qnorm;
        state->body_to_ned.z /= qnorm;
    }

    update_outputs(state, gyro_rad_s);
    state->heading_valid = state->heading_valid &&
        sample->system_time_ms - state->last_heading_reference_ms <=
            config->max_heading_reference_age_ms;
    return true;
}
