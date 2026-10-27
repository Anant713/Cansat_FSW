#include "dcs_attitude.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define DCS_GRAVITY_MPS2 9.80665f
#define DCS_DEG_PER_RAD 57.29577951308232f

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
    return (DcsVec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

static DcsQuaternion multiply(DcsQuaternion a, DcsQuaternion b)
{
    return (DcsQuaternion){
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
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
    return (DcsVec3){
        v.x + q.w * t.x + u.x,
        v.y + q.w * t.y + u.y,
        v.z + q.w * t.z + u.z,
    };
}

static DcsQuaternion from_angles(float yaw, float pitch, float roll)
{
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    return (DcsQuaternion){
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    };
}

static bool config_valid(const DcsAttitudeConfig *config)
{
    double recovery_samples;
    if (config == NULL) {
        return false;
    }
    recovery_samples = (double)config->sample_rate_hz *
                       (double)config->recovery_timeout_s;
    return isfinite(config->initial_roll_rad) &&
           isfinite(config->initial_pitch_rad) &&
           isfinite(config->initial_heading_rad) &&
           isfinite(config->sample_rate_hz) && config->sample_rate_hz > 0.0f &&
           isfinite(config->gyro_range_deg_s) && config->gyro_range_deg_s > 0.0f &&
           isfinite(config->gain) && config->gain > 0.0f && config->gain <= 10.0f &&
           isfinite(config->accel_rejection_deg) &&
           config->accel_rejection_deg > 0.0f && config->accel_rejection_deg <= 90.0f &&
           isfinite(config->mag_rejection_deg) &&
           config->mag_rejection_deg > 0.0f && config->mag_rejection_deg <= 90.0f &&
           isfinite(config->recovery_timeout_s) && config->recovery_timeout_s > 0.0f &&
           recovery_samples >= 1.0 && recovery_samples <= (double)INT32_MAX - 9.0 &&
           isfinite(config->min_accel_g) && config->min_accel_g > 0.0f &&
           config->min_accel_g < 1.0f &&
           isfinite(config->max_accel_g) && config->max_accel_g > 1.0f &&
           isfinite(config->calibrated_mag_strength) &&
           config->calibrated_mag_strength > 0.0f &&
           isfinite(config->max_mag_relative_error) &&
           config->max_mag_relative_error > 0.0f &&
           isfinite(config->magnetic_declination_rad) &&
           isfinite(config->max_step_s) && config->max_step_s > 0.0f &&
           config->max_heading_reference_age_ms > 0U;
}

static FusionVector to_fusion(DcsVec3 v, float scale)
{
    return (FusionVector){.axis = {v.x * scale, v.y * scale, v.z * scale}};
}

static void invalidate(DcsAttitudeState *state)
{
    state->orientation_valid = false;
    state->heading_valid = false;
    state->accel_correction_used = false;
    state->mag_correction_used = false;
    state->accel_reference_available = false;
    state->mag_reference_available = false;
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
    /* Existing provisional heading geometry policy; does not stop the AHRS. */
    if (horizontal_sq < 0.04f) {
        state->heading_valid = false;
        return;
    }
    state->heading_valid = true;
    state->payload_heading_rad = atan2f(forward_ned.y, forward_ned.x);
    state->payload_heading_rate_rad_s =
        (forward_ned.x * forward_dot.y - forward_ned.y * forward_dot.x) /
        horizontal_sq;
}

bool dcs_attitude_init(DcsAttitudeState *state,
                       const DcsAttitudeConfig *config,
                       uint32_t system_time_ms)
{
    DcsQuaternion magnetic_pose;
    FusionAhrsSettings settings;
    if (state == NULL || !config_valid(config)) {
        return false;
    }

    *state = (DcsAttitudeState){0};
    FusionAhrsInitialise(&state->fusion);
    settings = (FusionAhrsSettings){
        .sampleRate = config->sample_rate_hz,
        .convention = FusionConventionNed,
        .gain = config->gain,
        .gyroscopeRange = config->gyro_range_deg_s,
        .accelerationRejection = config->accel_rejection_deg,
        .magneticRejection = config->mag_rejection_deg,
        .rejectionTimeout = config->recovery_timeout_s,
    };
    FusionAhrsSetSettings(&state->fusion, &settings);
    /* East-positive declination: true heading = magnetic heading + declination. */
    magnetic_pose = from_angles(
        config->initial_heading_rad - config->magnetic_declination_rad,
        config->initial_pitch_rad, config->initial_roll_rad);
    FusionAhrsSetQuaternion(&state->fusion, (FusionQuaternion){.element = {
        magnetic_pose.w, magnetic_pose.x, magnetic_pose.y, magnetic_pose.z,
    }});
    FusionAhrsSkipStartup(&state->fusion);
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
    FusionVector gyro;
    FusionVector accel = FUSION_VECTOR_ZERO;
    FusionVector mag = FUSION_VECTOR_ZERO;
    FusionQuaternion magnetic_pose;
    FusionAhrsInternalStates diagnostics;
    FusionAhrsFlags flags;
    uint32_t elapsed_ms;
    float dt_s, magnitude, qnorm;

    if (state == NULL || !config_valid(config) || sample == NULL ||
        !state->initialized) {
        return false;
    }
    elapsed_ms = sample->system_time_ms - state->last_sample_ms;
    if (elapsed_ms == 0U) {
        return false; /* Duplicate sample: preserve every state field. */
    }
    dt_s = (float)elapsed_ms / 1000.0f;
    if (dt_s > config->max_step_s) {
        invalidate(state);
        state->initialized = false;
        return false;
    }
    gyro = to_fusion(sample->gyro_rad_s, DCS_DEG_PER_RAD);
    if (!sample->gyro_valid || !finite3(sample->gyro_rad_s) ||
        !isfinite(gyro.axis.x) || !isfinite(gyro.axis.y) || !isfinite(gyro.axis.z)) {
        invalidate(state);
        return false;
    }

    state->accel_reference_available = false;
    state->mag_reference_available = false;
    if (sample->accel_valid && sample->accel_correction_allowed &&
        finite3(sample->accel_mps2)) {
        magnitude = norm3(sample->accel_mps2) / DCS_GRAVITY_MPS2;
        if (isfinite(magnitude) && magnitude >= config->min_accel_g &&
            magnitude <= config->max_accel_g) {
            accel = to_fusion(sample->accel_mps2, 1.0f / DCS_GRAVITY_MPS2);
            state->accel_reference_available = true;
        }
    }
    if (sample->mag_valid && sample->mag_correction_allowed &&
        finite3(sample->mag_calibrated)) {
        magnitude = norm3(sample->mag_calibrated);
        if (isfinite(magnitude) && magnitude > 0.0f &&
            fabsf(magnitude - config->calibrated_mag_strength) <=
                config->calibrated_mag_strength * config->max_mag_relative_error) {
            FusionVector candidate = to_fusion(sample->mag_calibrated, 1.0f / magnitude);
            FusionVector horizontal = FusionVectorCross(
                FusionAhrsGetGravity(&state->fusion), candidate);
            float horizontal_sq = FusionVectorNormSquared(horizontal);
            /* Numerical guard against normalizing a zero cross product.
             * No former 10%-of-field horizontal-strength cutoff is applied. */
            if (isfinite(horizontal_sq) && horizontal_sq > 0.0f) {
                mag = candidate;
                state->mag_reference_available = true;
            }
        }
    }

    FusionAhrsSetSamplePeriod(&state->fusion, dt_s);
    FusionAhrsUpdate(&state->fusion, gyro, accel, mag);
    state->last_sample_ms = sample->system_time_ms;
    diagnostics = FusionAhrsGetInternalStates(&state->fusion);
    flags = FusionAhrsGetFlags(&state->fusion);
    state->accel_correction_used =
        state->accel_reference_available && !diagnostics.accelerometerIgnored;
    state->mag_correction_used =
        state->mag_reference_available && !diagnostics.magnetometerIgnored;
    /* Upstream residuals persist when inputs are zero; availability qualifies them. */
    state->accel_error_deg = state->accel_reference_available ? diagnostics.accelerationError : 0.0f;
    state->mag_error_deg = state->mag_reference_available ? diagnostics.magneticError : 0.0f;
    state->accel_recovery = flags.accelerationRecovery;
    state->mag_recovery = flags.magneticRecovery;
    state->gyro_overrange_recovery = flags.overrangeRecovery;
    state->fusion_startup = flags.startup;

    magnetic_pose = FusionAhrsGetQuaternion(&state->fusion);
    qnorm = sqrtf(magnetic_pose.element.w * magnetic_pose.element.w +
                  magnetic_pose.element.x * magnetic_pose.element.x +
                  magnetic_pose.element.y * magnetic_pose.element.y +
                  magnetic_pose.element.z * magnetic_pose.element.z);
    if (!isfinite(qnorm) || qnorm == 0.0f) {
        invalidate(state);
        state->initialized = false;
        return false;
    }
    /* Rotate magnetic NED into true NED for both quaternion and heading outputs. */
    state->body_to_ned = multiply(
        from_angles(config->magnetic_declination_rad, 0.0f, 0.0f),
        (DcsQuaternion){magnetic_pose.element.w / qnorm,
                        magnetic_pose.element.x / qnorm,
                        magnetic_pose.element.y / qnorm,
                        magnetic_pose.element.z / qnorm});
    if (state->mag_correction_used) {
        state->last_heading_reference_ms = sample->system_time_ms;
    }
    update_outputs(state, sample->gyro_rad_s);
    state->heading_valid = state->heading_valid &&
        sample->system_time_ms - state->last_heading_reference_ms <=
            config->max_heading_reference_age_ms;
    return true;
}
