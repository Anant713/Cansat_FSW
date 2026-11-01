#ifndef DCS_CIRCLE_FIT_H
#define DCS_CIRCLE_FIT_H

#include <stdbool.h>
#include <stdint.h>

#define DCS_CIRCLE_FIT_MAX_SAMPLES 64u

typedef struct {
    uint8_t minimum_samples;
    uint8_t maximum_samples;
    float minimum_direction_coverage_rad;
    float minimum_velocity_spread_mps;
    float maximum_rms_fraction;
    float minimum_airspeed_mps;
    float maximum_airspeed_mps;
} DcsCircleFitConfig;

typedef struct {
    float north_mps[DCS_CIRCLE_FIT_MAX_SAMPLES];
    float east_mps[DCS_CIRCLE_FIT_MAX_SAMPLES];
    uint32_t last_epoch_ms;
    uint8_t sample_count;
    bool has_epoch;
} DcsCircleFitState;

typedef struct {
    float wind_north_mps;
    float wind_east_mps;
    float maneuver_airspeed_mps;
    float radial_rms_mps;
    float direction_coverage_rad;
    uint8_t sample_count;
} DcsCircleFitResult;

bool dcs_circle_fit_config_valid(const DcsCircleFitConfig *config);

bool dcs_circle_fit_init(DcsCircleFitState *state,
                         const DcsCircleFitConfig *config);

/* Duplicate epochs and invalid/propagated velocities must not be added. */
bool dcs_circle_fit_add(DcsCircleFitState *state,
                        const DcsCircleFitConfig *config,
                        uint32_t gnss_epoch_ms,
                        float ground_v_north_mps,
                        float ground_v_east_mps);

/* Returns true only when all configured sample-spread and residual gates pass. */
bool dcs_circle_fit_solve(const DcsCircleFitState *state,
                          const DcsCircleFitConfig *config,
                          DcsCircleFitResult *result);

#endif
