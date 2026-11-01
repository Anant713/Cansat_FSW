#include "dcs_control.h"

#include <math.h>
#include <stddef.h>

#define DCS_CONTROL_PI_F 3.14159265358979323846f
#define DCS_CONTROL_TWO_PI_F (2.0f * DCS_CONTROL_PI_F)

static float clampf(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static float wrap_signed(float angle)
{
    float wrapped = fmodf(angle + DCS_CONTROL_PI_F, DCS_CONTROL_TWO_PI_F);
    if (wrapped < 0.0f) {
        wrapped += DCS_CONTROL_TWO_PI_F;
    }
    return wrapped - DCS_CONTROL_PI_F;
}

bool dcs_heading_controller_config_valid(
    const DcsHeadingControllerConfig *config)
{
    return config != NULL &&
        isfinite(config->kp) && config->kp >= 0.0f &&
        isfinite(config->kd) && config->kd >= 0.0f &&
        isfinite(config->max_command) &&
        config->max_command > 0.0f && config->max_command <= 1.0f &&
        isfinite(config->command_rate_per_s) && config->command_rate_per_s > 0.0f &&
        isfinite(config->max_step_s) && config->max_step_s > 0.0f;
}

bool dcs_heading_controller_init(DcsHeadingControllerState *state,
                                 const DcsHeadingControllerConfig *config)
{
    if (state == NULL || !dcs_heading_controller_config_valid(config)) {
        return false;
    }
    *state = (DcsHeadingControllerState){0};
    return true;
}

float dcs_heading_controller_update(DcsHeadingControllerState *state,
                                    const DcsHeadingControllerConfig *config,
                                    uint32_t now_ms,
                                    float desired_heading_rad,
                                    float measured_heading_rad,
                                    float measured_heading_rate_rad_s,
                                    bool feedback_valid)
{
    if (state == NULL || !dcs_heading_controller_config_valid(config)) {
        return 0.0f;
    }
    if (!feedback_valid || !isfinite(desired_heading_rad) ||
        !isfinite(measured_heading_rad) || !isfinite(measured_heading_rate_rad_s)) {
        state->command = 0.0f;
        state->last_update_ms = now_ms;
        state->has_time = true;
        return 0.0f;
    }

    float target = config->kp * wrap_signed(desired_heading_rad - measured_heading_rad) -
                   config->kd * measured_heading_rate_rad_s;
    target = clampf(target, -config->max_command, config->max_command);

    if (state->has_time) {
        float dt = (float)(now_ms - state->last_update_ms) * 0.001f;
        dt = fminf(dt, config->max_step_s);
        const float maximum_change = config->command_rate_per_s * dt;
        target = state->command + clampf(target - state->command,
                                         -maximum_change, maximum_change);
    }
    state->command = target;
    state->last_update_ms = now_ms;
    state->has_time = true;
    return target;
}

DcsBrakeDemand dcs_allocate_single_sided_brakes(float steering_command)
{
    DcsBrakeDemand result = {0.0f, 0.0f};
    if (!isfinite(steering_command)) {
        return result;
    }
    const float command = clampf(steering_command, -1.0f, 1.0f);
    if (command > 0.0f) {
        result.right = command;
    } else {
        result.left = -command;
    }
    return result;
}

DcsBrakeDemand dcs_allocate_flare(float flare_demand)
{
    const float demand = isfinite(flare_demand) ? clampf(flare_demand, 0.0f, 1.0f) : 0.0f;
    const DcsBrakeDemand result = {demand, demand};
    return result;
}

bool dcs_servo_map_valid(const DcsServoMapConfig *config)
{
    if (config == NULL || config->absolute_min_pulse_us >= config->absolute_max_pulse_us) {
        return false;
    }
    return config->neutral_pulse_us >= config->absolute_min_pulse_us &&
        config->neutral_pulse_us <= config->absolute_max_pulse_us &&
        config->maximum_pull_pulse_us >= config->absolute_min_pulse_us &&
        config->maximum_pull_pulse_us <= config->absolute_max_pulse_us;
}

uint16_t dcs_servo_map_pulse(const DcsServoMapConfig *config,
                             float normalized_pull)
{
    if (!dcs_servo_map_valid(config) || !isfinite(normalized_pull)) {
        return 0u;
    }
    const float demand = clampf(normalized_pull, 0.0f, 1.0f);
    const float pulse = (float)config->neutral_pulse_us + demand *
        ((float)config->maximum_pull_pulse_us - (float)config->neutral_pulse_us);
    const float bounded = clampf(pulse, (float)config->absolute_min_pulse_us,
                                 (float)config->absolute_max_pulse_us);
    return (uint16_t)lroundf(bounded);
}

bool dcs_servo_map_pair(const DcsServoMapConfig *left,
                        const DcsServoMapConfig *right,
                        DcsBrakeDemand demand,
                        DcsServoPulseOutput *output)
{
    if (output == NULL || !dcs_servo_map_valid(left) || !dcs_servo_map_valid(right) ||
        !isfinite(demand.left) || !isfinite(demand.right)) {
        return false;
    }
    output->left_pulse_us = dcs_servo_map_pulse(left, demand.left);
    output->right_pulse_us = dcs_servo_map_pulse(right, demand.right);
    return true;
}
