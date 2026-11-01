#include "dcs_nav.h"

#include <math.h>
#include <stddef.h>

#define DCS_NAV_RAD_PER_E7 1.745329252e-9f
#define DCS_NAV_WGS84_A_M 6378137.0f
#define DCS_NAV_WGS84_E2 0.00669438f

bool dcs_nav_init(DcsNavState *state, const DcsNavConfig *config)
{
    float latitude_rad;
    float sine_lat;
    float denominator;
    float root;

    if (state == NULL || config == NULL ||
        config->target_lat_e7 < -900000000 ||
        config->target_lat_e7 > 900000000 ||
        config->target_lon_e7 < -1800000000 ||
        config->target_lon_e7 > 1800000000 ||
        config->max_gnss_age_ms == 0U || config->max_baro_age_ms == 0U) {
        return false;
    }

    *state = (DcsNavState){0};
    state->target_lat_e7 = config->target_lat_e7;
    state->target_lon_e7 = config->target_lon_e7;

    latitude_rad = (float)config->target_lat_e7 * DCS_NAV_RAD_PER_E7;
    sine_lat = sinf(latitude_rad);
    denominator = 1.0f - DCS_NAV_WGS84_E2 * sine_lat * sine_lat;
    root = sqrtf(denominator);
    state->north_m_per_lat_e7 =
        DCS_NAV_WGS84_A_M * (1.0f - DCS_NAV_WGS84_E2) /
        (denominator * root) * DCS_NAV_RAD_PER_E7;
    state->east_m_per_lon_e7 =
        DCS_NAV_WGS84_A_M / root * cosf(latitude_rad) *
        DCS_NAV_RAD_PER_E7;

    return true;
}

void dcs_nav_step(DcsNavState *state, const DcsNavConfig *config,
                  uint32_t now_ms, const DcsNavGnssFix *gnss,
                  const DcsNavBaroSample *baro)
{
    uint32_t step_ms = 0U;

    if (state == NULL || config == NULL) {
        return;
    }

    if (state->has_step_time) {
        step_ms = now_ms - state->last_step_ms;
    }
    state->last_step_ms = now_ms;
    state->has_step_time = true;

    if (state->has_gnss) {
        float dt_s = (float)step_ms * 0.001f;
        state->output.north_m += state->output.ground_v_north_mps * dt_s;
        state->output.east_m += state->output.ground_v_east_mps * dt_s;
    }

    if (gnss != NULL && gnss->valid &&
        gnss->lat_e7 >= -900000000 && gnss->lat_e7 <= 900000000 &&
        gnss->lon_e7 >= -1800000000 && gnss->lon_e7 <= 1800000000 &&
        isfinite(gnss->ground_v_north_mps) &&
        isfinite(gnss->ground_v_east_mps) &&
        now_ms - gnss->system_time_ms <= config->max_gnss_age_ms &&
        (!state->has_gnss ||
         gnss->gnss_epoch_ms != state->last_gnss_epoch_ms)) {
        int64_t delta_lat_e7 = (int64_t)gnss->lat_e7 -
                               (int64_t)state->target_lat_e7;
        int64_t delta_lon_e7 = (int64_t)gnss->lon_e7 -
                               (int64_t)state->target_lon_e7;

        state->output.north_m = (float)delta_lat_e7 *
                                state->north_m_per_lat_e7;
        state->output.east_m = (float)delta_lon_e7 *
                               state->east_m_per_lon_e7;
        state->output.ground_v_north_mps = gnss->ground_v_north_mps;
        state->output.ground_v_east_mps = gnss->ground_v_east_mps;
        /* Advance a delayed, but still fresh, fix to the present loop time. */
        state->output.north_m += state->output.ground_v_north_mps *
                                 (float)(now_ms - gnss->system_time_ms) * 0.001f;
        state->output.east_m += state->output.ground_v_east_mps *
                                (float)(now_ms - gnss->system_time_ms) * 0.001f;
        state->last_gnss_system_time_ms = gnss->system_time_ms;
        state->last_gnss_epoch_ms = gnss->gnss_epoch_ms;
        state->has_gnss = true;
    }

    if (baro != NULL && baro->valid &&
        isfinite(baro->height_above_target_m) &&
        now_ms - baro->system_time_ms <= config->max_baro_age_ms &&
        (!state->has_baro ||
         baro->system_time_ms != state->last_baro_system_time_ms)) {
        state->output.down_m = -baro->height_above_target_m;
        state->last_baro_system_time_ms = baro->system_time_ms;
        state->has_baro = true;
    }

    state->output.horizontal_valid = state->has_gnss;
    state->output.gnss_age_ms = state->has_gnss
        ? now_ms - state->last_gnss_system_time_ms : 0u;
    state->output.gnss_fresh = state->has_gnss &&
        state->output.gnss_age_ms <= config->max_gnss_age_ms;
    state->output.horizontal_propagated = state->has_gnss &&
        !state->output.gnss_fresh;
    state->output.vertical_valid =
        state->has_baro &&
        now_ms - state->last_baro_system_time_ms <=
            config->max_baro_age_ms;
}
