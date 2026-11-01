#ifndef DCS_CONTROL_H
#define DCS_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float kd;
    float max_command;
    float command_rate_per_s;
    float max_step_s;
} DcsHeadingControllerConfig;

typedef struct {
    float command;
    uint32_t last_update_ms;
    bool has_time;
} DcsHeadingControllerState;

typedef struct {
    float left;
    float right;
} DcsBrakeDemand;

typedef struct {
    /* Calibrated installed endpoints. Reversal is represented by endpoint order. */
    uint16_t neutral_pulse_us;
    uint16_t maximum_pull_pulse_us;
    uint16_t absolute_min_pulse_us;
    uint16_t absolute_max_pulse_us;
} DcsServoMapConfig;

typedef struct {
    uint16_t left_pulse_us;
    uint16_t right_pulse_us;
} DcsServoPulseOutput;

bool dcs_heading_controller_config_valid(
    const DcsHeadingControllerConfig *config);

bool dcs_heading_controller_init(DcsHeadingControllerState *state,
                                 const DcsHeadingControllerConfig *config);

/*
 * Implements u = Kp*wrap(desired-heading) - Kd*measured_heading_rate.
 * On invalid feedback, the output becomes neutral immediately. The derivative
 * term uses the measured geographic heading rate, not a differentiated error.
 */
float dcs_heading_controller_update(DcsHeadingControllerState *state,
                                    const DcsHeadingControllerConfig *config,
                                    uint32_t now_ms,
                                    float desired_heading_rad,
                                    float measured_heading_rad,
                                    float measured_heading_rate_rad_s,
                                    bool feedback_valid);

/* Positive steering pulls right; negative steering pulls left. */
DcsBrakeDemand dcs_allocate_single_sided_brakes(float steering_command);

/* Flare bypasses heading control and applies equal normalized brake demand. */
DcsBrakeDemand dcs_allocate_flare(float flare_demand);

bool dcs_servo_map_valid(const DcsServoMapConfig *config);
uint16_t dcs_servo_map_pulse(const DcsServoMapConfig *config,
                             float normalized_pull);
bool dcs_servo_map_pair(const DcsServoMapConfig *left,
                        const DcsServoMapConfig *right,
                        DcsBrakeDemand demand,
                        DcsServoPulseOutput *output);

#endif
