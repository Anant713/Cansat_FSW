#ifndef DCS_NAV_H
#define DCS_NAV_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t target_lat_e7;
    int32_t target_lon_e7;
    uint32_t max_gnss_age_ms;
    uint32_t max_baro_age_ms;
} DcsNavConfig;

typedef struct {
    uint32_t gnss_epoch_ms;
    uint32_t system_time_ms;
    int32_t lat_e7;
    int32_t lon_e7;
    float ground_v_north_mps;
    float ground_v_east_mps;
    bool valid;
} DcsNavGnssFix;

typedef struct {
    uint32_t system_time_ms;
    float height_above_target_m;
    bool valid;
} DcsNavBaroSample;

typedef struct {
    float north_m;
    float east_m;
    float down_m;
    float ground_v_north_mps;
    float ground_v_east_mps;
    bool horizontal_valid;
    bool vertical_valid;
    /* Measurement freshness is separate from propagated-state availability. */
    bool gnss_fresh;
    bool horizontal_propagated;
    uint32_t gnss_age_ms;
} DcsNavOutput;

typedef struct {
    DcsNavOutput output;
    int32_t target_lat_e7;
    int32_t target_lon_e7;
    float north_m_per_lat_e7;
    float east_m_per_lon_e7;
    uint32_t last_step_ms;
    uint32_t last_gnss_system_time_ms;
    uint32_t last_gnss_epoch_ms;
    uint32_t last_baro_system_time_ms;
    bool has_step_time;
    bool has_gnss;
    bool has_baro;
} DcsNavState;

/* Fixed target-centred North/East/Down frame. No hardware access or allocation. */
bool dcs_nav_init(DcsNavState *state, const DcsNavConfig *config);

/*
 * Pass NULL for either sample when no new measurement is available. The GNSS
 * driver supplies coherent position and velocity from a valid solution,
 * timestamped at measurement acquisition and supplied in chronological order.
 * The
 * barometer provider supplies height relative to target ground level, not
 * height relative to the launch pad. Timestamps share one monotonic clock.
 * After the first accepted fix, horizontal position continues with held GNSS
 * velocity when fixes are unavailable. gnss_fresh distinguishes a measurement-
 * supported state from this simple propagation. Propagation is deliberately
 * not fed back into measurement-only wind or circle identification.
 * Vertical position is corrected by barometer; it is not inertially propagated.
 */
void dcs_nav_step(DcsNavState *state, const DcsNavConfig *config,
                  uint32_t now_ms, const DcsNavGnssFix *gnss,
                  const DcsNavBaroSample *baro);

#endif
