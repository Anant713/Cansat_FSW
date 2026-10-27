#ifndef DCS_WIND_H
#define DCS_WIND_H

#include <stdbool.h>
#include <stdint.h>

/* Angles are radians, velocities are m/s, and time is a monotonic millisecond tick. */
typedef struct {
    float horizontal_airspeed_mps;
    float ema_alpha;
    float max_heading_rate_rad_s;
    float max_brake_command;
    float max_course_rate_rad_s;
    float min_ground_speed_mps;
    uint32_t max_heading_age_ms;
    uint32_t max_fix_interval_ms;
} DcsWindConfig;

typedef struct {
    uint32_t gnss_epoch_ms;
    uint32_t system_time_ms;
    float ground_v_north_mps;
    float ground_v_east_mps;
    bool valid;
} DcsWindGnssFix;

typedef struct {
    uint32_t system_time_ms;
    float payload_heading_rad;
    float heading_rate_rad_s;
    bool valid;
} DcsWindAttitude;

typedef struct {
    float north_mps;
    float east_mps;
    bool has_estimate;
    /* Measurement timestamp of last accepted wind update; use only if has_estimate. */
    uint32_t last_update_ms;
    bool has_previous_course;
    uint32_t previous_gnss_epoch_ms;
    uint32_t previous_fix_system_time_ms;
    float previous_course_rad;
} DcsWindState;

/* Returns false if any configuration field is outside its usable range. */
bool dcs_wind_init(DcsWindState *state, const DcsWindConfig *config);

/*
 * Call once per received GNSS solution. A valid first fix establishes the course
 * baseline; later fixes can update wind only when all straight-flight gates pass.
 * The heading is a PAYLOAD measurement. Wind is provisional until its relation
 * to canopy heading is established by flight testing. Never feed this estimate
 * back as a substitute for a valid GNSS ground-velocity measurement.
 * Returns true when a wind measurement passes all gates and is EMA-weighted.
 * Only accepted updates refresh last_update_ms, using fix->system_time_ms.
 * No automatic expiry is applied. Consumers can compute age as unsigned
 * now_ms - last_update_ms; has_estimate distinguishes missing from zero wind.
 * The driver must supply GNSS fixes in chronological order.
 */
bool dcs_wind_update(DcsWindState *state, const DcsWindConfig *config,
                     const DcsWindGnssFix *fix,
                     const DcsWindAttitude *attitude,
                     float brake_command);

#endif
