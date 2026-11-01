#ifndef DCS_RUNTIME_H
#define DCS_RUNTIME_H

#include "dcs_circle_fit.h"
#include "dcs_control.h"
#include "dcs_guidance.h"
#include "dcs_wind.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DCS_RUNTIME_INACTIVE,
    DCS_RUNTIME_CANOPY_SETTLING,
    DCS_RUNTIME_IDENTIFICATION_TURN,
    DCS_RUNTIME_TRIM_SETTLING,
    DCS_RUNTIME_TRIM_SAMPLING,
    DCS_RUNTIME_GUIDANCE,
    DCS_RUNTIME_FLARE
} DcsRuntimeMode;

typedef struct {
    uint32_t canopy_settling_ms;
    float identification_steering_command;
    uint32_t identification_timeout_ms;
    float minimum_identification_height_m;
    uint32_t trim_settling_ms;
    uint8_t trim_sample_count;
    uint32_t trim_timeout_ms;
    float fallback_horizontal_airspeed_mps;
    bool fallback_has_wind;
    float fallback_wind_north_mps;
    float fallback_wind_east_mps;
    float flare_brake_demand;
    DcsCircleFitConfig circle_fit;
    DcsWindConfig wind;
    DcsGuidanceConfig guidance;
    DcsHeadingControllerConfig controller;
    DcsServoMapConfig left_servo;
    DcsServoMapConfig right_servo;
} DcsRuntimeConfig;

typedef struct {
    uint32_t now_ms;
    DcsGuidanceInput navigation;
    bool has_fresh_gnss_velocity;
    uint32_t gnss_epoch_ms;
    uint32_t gnss_system_time_ms;
    float heading_rad;
    float heading_rate_rad_s;
    bool heading_valid;
} DcsRuntimeInput;

typedef struct {
    DcsRuntimeMode mode;
    DcsGuidanceOutput guidance;
    float steering_command;
    DcsBrakeDemand brake_demand;
    DcsServoPulseOutput servo;
    bool servo_output_valid;
    bool identification_complete;
    bool identification_failed;
    bool trim_speed_identified;
    float horizontal_airspeed_mps;
    bool has_wind_estimate;
    float wind_north_mps;
    float wind_east_mps;
} DcsRuntimeOutput;

typedef struct {
    DcsRuntimeMode mode;
    uint32_t mode_start_ms;
    DcsCircleFitState circle;
    DcsCircleFitResult circle_result;
    DcsWindState wind;
    DcsWindConfig wind_config;
    DcsGuidanceState guidance;
    DcsHeadingControllerState controller;
    float current_airspeed_mps;
    float trim_speed_sum_mps;
    uint8_t trim_samples;
    uint32_t last_trim_epoch_ms;
    bool has_trim_epoch;
    float last_steering_command;
    uint32_t last_runtime_update_ms;
    bool has_runtime_update_time;
    bool identification_complete;
    bool identification_failed;
    bool trim_speed_identified;
    bool guidance_started;
    bool initialized;
} DcsRuntimeState;

/* Called by main FSW on entry to its secondary-descent state. */
bool dcs_runtime_start(DcsRuntimeState *state,
                       const DcsRuntimeConfig *config,
                       uint32_t now_ms);

/* One bounded, nonblocking call from the cooperative super-loop. */
void dcs_runtime_update(DcsRuntimeState *state,
                        const DcsRuntimeConfig *config,
                        const DcsRuntimeInput *input,
                        DcsRuntimeOutput *output);

#endif
