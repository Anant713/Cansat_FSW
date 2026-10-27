#include "dcs_wind.h"

#include <math.h>
#include <stddef.h>

static bool config_valid(const DcsWindConfig *config)
{
    return config != NULL &&
           isfinite(config->horizontal_airspeed_mps) &&
           config->horizontal_airspeed_mps > 0.0f &&
           isfinite(config->ema_alpha) &&
           config->ema_alpha > 0.0f && config->ema_alpha <= 1.0f &&
           isfinite(config->max_heading_rate_rad_s) &&
           config->max_heading_rate_rad_s >= 0.0f &&
           isfinite(config->max_brake_command) &&
           config->max_brake_command >= 0.0f &&
           config->max_brake_command <= 1.0f &&
           isfinite(config->max_course_rate_rad_s) &&
           config->max_course_rate_rad_s >= 0.0f &&
           isfinite(config->min_ground_speed_mps) &&
           config->min_ground_speed_mps >= 0.0f &&
           config->max_heading_age_ms > 0U &&
           config->max_fix_interval_ms > 0U;
}

bool dcs_wind_init(DcsWindState *state, const DcsWindConfig *config)
{
    if (state == NULL || !config_valid(config)) {
        return false;
    }

    *state = (DcsWindState){0};
    return true;
}

bool dcs_wind_update(DcsWindState *state, const DcsWindConfig *config,
                     const DcsWindGnssFix *fix,
                     const DcsWindAttitude *attitude,
                     float brake_command)
{
    float course_rad;
    float course_delta_rad;
    float course_rate_rad_s;
    float measured_north_mps;
    float measured_east_mps;
    float ground_speed_mps;
    uint32_t interval_ms;

    if (state == NULL || !config_valid(config) || fix == NULL ||
        attitude == NULL || !fix->valid ||
        !isfinite(fix->ground_v_north_mps) ||
        !isfinite(fix->ground_v_east_mps)) {
        return false;
    }

    if (state->has_previous_course &&
        fix->gnss_epoch_ms == state->previous_gnss_epoch_ms) {
        return false;
    }

    ground_speed_mps = hypotf(fix->ground_v_north_mps,
                              fix->ground_v_east_mps);
    if (!isfinite(ground_speed_mps) ||
        ground_speed_mps < config->min_ground_speed_mps ||
        ground_speed_mps == 0.0f) {
        return false;
    }

    course_rad = atan2f(fix->ground_v_east_mps,
                        fix->ground_v_north_mps);
    if (!state->has_previous_course) {
        state->previous_course_rad = course_rad;
        state->previous_fix_system_time_ms = fix->system_time_ms;
        state->previous_gnss_epoch_ms = fix->gnss_epoch_ms;
        state->has_previous_course = true;
        return false;
    }

    interval_ms = fix->system_time_ms - state->previous_fix_system_time_ms;
    course_delta_rad = atan2f(sinf(course_rad - state->previous_course_rad),
                              cosf(course_rad - state->previous_course_rad));
    state->previous_course_rad = course_rad;
    state->previous_fix_system_time_ms = fix->system_time_ms;
    state->previous_gnss_epoch_ms = fix->gnss_epoch_ms;

    if (interval_ms == 0U || interval_ms > config->max_fix_interval_ms ||
        !attitude->valid || !isfinite(attitude->payload_heading_rad) ||
        !isfinite(attitude->heading_rate_rad_s) ||
        !isfinite(brake_command) ||
        fix->system_time_ms - attitude->system_time_ms >
            config->max_heading_age_ms) {
        return false;
    }

    course_rate_rad_s = fabsf(course_delta_rad) * 1000.0f / (float)interval_ms;
    if (fabsf(attitude->heading_rate_rad_s) >
            config->max_heading_rate_rad_s ||
        fabsf(brake_command) > config->max_brake_command ||
        course_rate_rad_s > config->max_course_rate_rad_s) {
        return false;
    }

    measured_north_mps = fix->ground_v_north_mps -
                         config->horizontal_airspeed_mps *
                             cosf(attitude->payload_heading_rad);
    measured_east_mps = fix->ground_v_east_mps -
                        config->horizontal_airspeed_mps *
                            sinf(attitude->payload_heading_rad);
    if (!isfinite(measured_north_mps) || !isfinite(measured_east_mps)) {
        return false;
    }

    state->north_mps += config->ema_alpha *
                        (measured_north_mps - state->north_mps);
    state->east_mps += config->ema_alpha *
                       (measured_east_mps - state->east_mps);
    state->has_estimate = true;
    state->last_update_ms = fix->system_time_ms;

    return true;
}
