#include "dcs_runtime.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static bool valid_config(const DcsRuntimeConfig *config)
{
    return config != NULL && config->canopy_settling_ms > 0u &&
        isfinite(config->identification_steering_command) &&
        fabsf(config->identification_steering_command) > 0.0f &&
        fabsf(config->identification_steering_command) <= 1.0f &&
        config->identification_timeout_ms > 0u &&
        isfinite(config->minimum_identification_height_m) &&
        config->minimum_identification_height_m >= 0.0f &&
        config->trim_settling_ms > 0u && config->trim_sample_count > 0u &&
        config->trim_timeout_ms > 0u &&
        isfinite(config->fallback_horizontal_airspeed_mps) &&
        config->fallback_horizontal_airspeed_mps > 0.0f &&
        (!config->fallback_has_wind ||
         (isfinite(config->fallback_wind_north_mps) &&
          isfinite(config->fallback_wind_east_mps))) &&
        isfinite(config->flare_brake_demand) &&
        config->flare_brake_demand >= 0.0f && config->flare_brake_demand <= 1.0f &&
        dcs_servo_map_valid(&config->left_servo) &&
        dcs_servo_map_valid(&config->right_servo) &&
        dcs_circle_fit_config_valid(&config->circle_fit) &&
        dcs_heading_controller_config_valid(&config->controller) &&
        dcs_wind_config_valid(&config->wind);
}

bool dcs_runtime_start(DcsRuntimeState *state,
                       const DcsRuntimeConfig *config,
                       uint32_t now_ms)
{
    if (state == NULL || !valid_config(config)) {
        return false;
    }
    *state = (DcsRuntimeState){0};
    if (!dcs_circle_fit_init(&state->circle, &config->circle_fit) ||
        !dcs_heading_controller_init(&state->controller, &config->controller) ||
        !dcs_wind_init(&state->wind, &config->wind)) {
        return false;
    }
    state->wind_config = config->wind;
    state->current_airspeed_mps = config->fallback_horizontal_airspeed_mps;
    state->current_turn_radius_m = config->guidance.planner.turn_radius_m;
    state->controller.has_time = true;
    state->controller.last_update_ms = now_ms;
    state->last_runtime_update_ms = now_ms;
    state->has_runtime_update_time = true;
    state->mode = DCS_RUNTIME_CANOPY_SETTLING;
    state->mode_start_ms = now_ms;
    state->initialized = true;
    return true;
}

static DcsGuidanceConfig current_guidance_config(const DcsRuntimeState *state,
                                                 const DcsRuntimeConfig *config)
{
    DcsGuidanceConfig guidance = config->guidance;
    guidance.planner.horizontal_airspeed_mps = state->current_airspeed_mps;
    guidance.planner.turn_radius_m = state->current_turn_radius_m;
    if (guidance.loiter_radius_m < state->current_turn_radius_m) {
        guidance.loiter_radius_m = state->current_turn_radius_m;
    }
    return guidance;
}

static void seed_fallback(DcsRuntimeState *state,
                          const DcsRuntimeConfig *config,
                          uint32_t now_ms)
{
    state->current_airspeed_mps = config->fallback_horizontal_airspeed_mps;
    state->wind_config.horizontal_airspeed_mps = state->current_airspeed_mps;
    if (config->fallback_has_wind) {
        (void)dcs_wind_seed(&state->wind, config->fallback_wind_north_mps,
                            config->fallback_wind_east_mps, now_ms);
    }
}

static void finish_identification(DcsRuntimeState *state,
                                  const DcsRuntimeConfig *config,
                                  uint32_t now_ms,
                                  bool trim_valid)
{
    if (trim_valid) {
        state->current_airspeed_mps = state->trim_speed_sum_mps /
                                      (float)state->trim_samples;
        state->trim_speed_identified = true;
        state->wind_config.horizontal_airspeed_mps = state->current_airspeed_mps;
        (void)dcs_wind_seed(&state->wind, state->circle_result.wind_north_mps,
                            state->circle_result.wind_east_mps, now_ms);
    } else {
        state->identification_failed = true;
        seed_fallback(state, config, now_ms);
        if (state->circle_result.sample_count > 0u) {
            (void)dcs_wind_seed(&state->wind, state->circle_result.wind_north_mps,
                                state->circle_result.wind_east_mps, now_ms);
        }
    }
    state->identification_complete = true;
    state->mode = DCS_RUNTIME_GUIDANCE;
    state->mode_start_ms = now_ms;
}

static bool identification_height_available(const DcsRuntimeConfig *config,
                                             const DcsRuntimeInput *input)
{
    return input->navigation.vertical_valid &&
        input->navigation.height_above_target_m >
            config->minimum_identification_height_m;
}

static void update_wind(DcsRuntimeState *state,
                        const DcsRuntimeInput *input)
{
    if (!input->has_fresh_gnss_velocity) {
        return;
    }
    const DcsWindGnssFix fix = {
        .gnss_epoch_ms = input->gnss_epoch_ms,
        .system_time_ms = input->gnss_system_time_ms,
        .ground_v_north_mps = input->navigation.ground_v_north_mps,
        .ground_v_east_mps = input->navigation.ground_v_east_mps,
        .valid = input->navigation.horizontal_valid,
    };
    const DcsWindAttitude attitude = {
        .system_time_ms = input->now_ms,
        .payload_heading_rad = input->heading_rad,
        .heading_rate_rad_s = input->heading_rate_rad_s,
        .valid = input->heading_valid,
    };
    (void)dcs_wind_update(&state->wind, &state->wind_config, &fix,
                          &attitude, state->last_steering_command);
}

static void update_identification(DcsRuntimeState *state,
                                  const DcsRuntimeConfig *config,
                                  const DcsRuntimeInput *input)
{
    switch (state->mode) {
    case DCS_RUNTIME_CANOPY_SETTLING:
        if (input->now_ms - state->mode_start_ms >= config->canopy_settling_ms) {
            state->mode = DCS_RUNTIME_IDENTIFICATION_TURN;
            state->mode_start_ms = input->now_ms;
        }
        break;
    case DCS_RUNTIME_IDENTIFICATION_TURN:
        if (!identification_height_available(config, input) ||
            input->now_ms - state->mode_start_ms >= config->identification_timeout_ms) {
            finish_identification(state, config, input->now_ms, false);
            break;
        }
        if (input->has_fresh_gnss_velocity && input->navigation.horizontal_valid) {
            (void)dcs_circle_fit_add(&state->circle, &config->circle_fit,
                input->gnss_epoch_ms, input->navigation.ground_v_north_mps,
                input->navigation.ground_v_east_mps);
            if (dcs_circle_fit_solve(&state->circle, &config->circle_fit,
                                     &state->circle_result)) {
                state->current_turn_radius_m = state->circle_result.turn_radius_m;
                state->turn_radius_identified = true;
                state->mode = DCS_RUNTIME_TRIM_SETTLING;
                state->mode_start_ms = input->now_ms;
            }
        }
        break;
    case DCS_RUNTIME_TRIM_SETTLING:
        if (input->now_ms - state->mode_start_ms >= config->trim_settling_ms) {
            state->mode = DCS_RUNTIME_TRIM_SAMPLING;
            state->mode_start_ms = input->now_ms;
        }
        break;
    case DCS_RUNTIME_TRIM_SAMPLING:
        if (!identification_height_available(config, input) ||
            input->now_ms - state->mode_start_ms >= config->trim_timeout_ms) {
            finish_identification(state, config, input->now_ms, false);
            break;
        }
        if (input->has_fresh_gnss_velocity && input->navigation.horizontal_valid &&
            (!state->has_trim_epoch || input->gnss_epoch_ms != state->last_trim_epoch_ms)) {
            const float air_n = input->navigation.ground_v_north_mps -
                                state->circle_result.wind_north_mps;
            const float air_e = input->navigation.ground_v_east_mps -
                                state->circle_result.wind_east_mps;
            const float speed = hypotf(air_n, air_e);
            if (isfinite(speed) && speed >= config->circle_fit.minimum_airspeed_mps &&
                speed <= config->circle_fit.maximum_airspeed_mps) {
                state->trim_speed_sum_mps += speed;
                state->trim_samples++;
            }
            state->last_trim_epoch_ms = input->gnss_epoch_ms;
            state->has_trim_epoch = true;
            if (state->trim_samples >= config->trim_sample_count) {
                finish_identification(state, config, input->now_ms, true);
            }
        }
        break;
    default:
        break;
    }
}

void dcs_runtime_update(DcsRuntimeState *state,
                        const DcsRuntimeConfig *config,
                        const DcsRuntimeInput *input,
                        DcsRuntimeOutput *output)
{
    if (output == NULL) {
        return;
    }
    *output = (DcsRuntimeOutput){0};
    if (state == NULL || input == NULL || !state->initialized || !valid_config(config)) {
        return;
    }

    update_identification(state, config, input);
    DcsBrakeDemand brakes = {0.0f, 0.0f};
    float steering = 0.0f;

    if (state->mode == DCS_RUNTIME_IDENTIFICATION_TURN) {
        const float target = clampf(config->identification_steering_command, -1.0f, 1.0f);
        float dt = (float)(input->now_ms - state->last_runtime_update_ms) * 0.001f;
        dt = fminf(dt, config->controller.max_step_s);
        const float maximum_change = config->controller.command_rate_per_s * dt;
        steering = state->last_steering_command +
            clampf(target - state->last_steering_command,
                   -maximum_change, maximum_change);
        brakes = dcs_allocate_single_sided_brakes(steering);
    } else if (state->mode == DCS_RUNTIME_GUIDANCE ||
               state->mode == DCS_RUNTIME_FLARE) {
        update_wind(state, input);
        DcsGuidanceConfig guidance_config = current_guidance_config(state, config);
        DcsGuidanceInput guidance_input = input->navigation;
        guidance_input.now_ms = input->now_ms;
        guidance_input.has_wind_estimate = state->wind.has_estimate;
        guidance_input.wind_north_mps = state->wind.north_mps;
        guidance_input.wind_east_mps = state->wind.east_mps;
        if (!state->guidance_started) {
            state->guidance_started = dcs_guidance_start(&state->guidance,
                &guidance_config, &guidance_input);
        }
        if (state->guidance_started) {
            dcs_guidance_update(&state->guidance, &guidance_config,
                                &guidance_input, &output->guidance);
            if (output->guidance.flare_requested) {
                state->mode = DCS_RUNTIME_FLARE;
                brakes = dcs_allocate_flare(config->flare_brake_demand);
            } else {
                steering = dcs_heading_controller_update(&state->controller,
                    &config->controller, input->now_ms,
                    output->guidance.desired_heading_rad,
                    input->heading_rad, input->heading_rate_rad_s,
                    output->guidance.heading_command_valid && input->heading_valid);
                brakes = dcs_allocate_single_sided_brakes(steering);
            }
        }
    }

    state->last_steering_command = steering;
    state->last_runtime_update_ms = input->now_ms;
    state->has_runtime_update_time = true;
    output->mode = state->mode;
    output->steering_command = steering;
    output->brake_demand = brakes;
    output->servo_output_valid = dcs_servo_map_pair(&config->left_servo,
        &config->right_servo, brakes, &output->servo);
    output->identification_complete = state->identification_complete;
    output->identification_failed = state->identification_failed;
    output->trim_speed_identified = state->trim_speed_identified;
    output->turn_radius_identified = state->turn_radius_identified;
    output->horizontal_airspeed_mps = state->current_airspeed_mps;
    output->turn_radius_m = state->current_turn_radius_m;
    output->has_wind_estimate = state->wind.has_estimate;
    output->wind_north_mps = state->wind.north_mps;
    output->wind_east_mps = state->wind.east_mps;
}
