#include "../dcs_runtime.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265358979323846f

static DcsRuntimeConfig test_config(void)
{
    DcsRuntimeConfig config = {
        .canopy_settling_ms = 10u,
        .identification_steering_command = 0.4f,
        .identification_timeout_ms = 10000u,
        .minimum_identification_height_m = 100.0f,
        .trim_settling_ms = 10u,
        .trim_sample_count = 3u,
        .trim_timeout_ms = 1000u,
        .fallback_horizontal_airspeed_mps = 8.0f,
        .fallback_has_wind = false,
        .flare_brake_demand = 0.8f,
        .circle_fit = {
            .minimum_samples = 12u,
            .maximum_samples = 24u,
            .minimum_direction_coverage_rad = 4.5f,
            .minimum_velocity_spread_mps = 1.0f,
            .maximum_rms_fraction = 0.1f,
            .minimum_airspeed_mps = 1.0f,
            .maximum_airspeed_mps = 10.0f,
            .minimum_turn_radius_m = 1.0f,
            .maximum_turn_radius_m = 100.0f,
            .minimum_turn_direction_consistency = 0.8f,
        },
        .wind = {
            .horizontal_airspeed_mps = 8.0f,
            .ema_alpha = 0.2f,
            .max_heading_rate_rad_s = 0.2f,
            .max_brake_command = 0.1f,
            .max_course_rate_rad_s = 0.2f,
            .min_ground_speed_mps = 0.2f,
            .max_heading_age_ms = 500u,
            .max_fix_interval_ms = 1000u,
        },
        .guidance = {
            .planner = {
                .turn_radius_m = 20.0f,
                .horizontal_airspeed_mps = 8.0f,
                .straight_sink_rate_mps = 3.0f,
                .turn_loss_fraction = 0.2f,
                .final_approach_length_m = 30.0f,
                .energy_point_upstream_m = 20.0f,
                .height_reserve_m = 15.0f,
                .height_tolerance_fraction = 0.02f,
                .weak_wind_threshold_mps = 1.0f,
            },
            .los_lookahead_m = 10.0f,
            .horizontal_error_threshold_m = 10.0f,
            .height_shortage_threshold_m = 5.0f,
            .wind_change_fraction = 0.3f,
            .horizontal_persistence_ms = 1000u,
            .height_persistence_ms = 1000u,
            .wind_persistence_ms = 1000u,
            .reassessment_min_interval_ms = 2000u,
            .segment_end_tolerance_m = 0.5f,
            .loiter_radius_m = 25.0f,
            .loiter_search_steps = 16u,
            .prefer_right_loiter = true,
            .flare_height_m = 5.0f,
        },
        .controller = {
            .kp = 1.0f,
            .kd = 0.2f,
            .max_command = 0.7f,
            .command_rate_per_s = 2.0f,
            .max_step_s = 0.2f,
        },
        .left_servo = {1000u, 1500u, 900u, 2100u},
        .right_servo = {2000u, 1400u, 900u, 2100u},
    };
    return config;
}

int main(void)
{
    DcsRuntimeConfig config = test_config();
    DcsRuntimeState state;
    DcsRuntimeOutput output;
    DcsRuntimeInput input = {
        .now_ms = 0u,
        .navigation = {
            .position = {200.0f, 0.0f},
            .ground_v_north_mps = -4.0f,
            .ground_v_east_mps = 0.0f,
            .height_above_target_m = 600.0f,
            .horizontal_valid = true,
            .vertical_valid = true,
        },
        .heading_valid = true,
        .gnss_system_time_ms = 0u,
    };
    assert(dcs_runtime_start(&state, &config, 0u));
    dcs_runtime_update(&state, &config, &input, &output);
    assert(output.mode == DCS_RUNTIME_CANOPY_SETTLING);
    assert(output.servo.left_pulse_us == 1000u);
    assert(output.servo.right_pulse_us == 2000u);

    input.now_ms = 10u;
    dcs_runtime_update(&state, &config, &input, &output);
    assert(output.mode == DCS_RUNTIME_IDENTIFICATION_TURN);
    assert(output.brake_demand.right > 0.0f);

    for (uint32_t i = 0u; i < 12u; ++i) {
        const float angle = 2.0f * PI_F * (float)i / 12.0f;
        input.now_ms += 500u;
        input.has_fresh_gnss_velocity = true;
        input.gnss_epoch_ms = 500u * (i + 1u);
        input.gnss_system_time_ms = input.now_ms;
        input.navigation.ground_v_north_mps = 2.0f + 4.0f * cosf(angle);
        input.navigation.ground_v_east_mps = -3.0f + 4.0f * sinf(angle);
        dcs_runtime_update(&state, &config, &input, &output);
    }
    assert(output.mode == DCS_RUNTIME_TRIM_SETTLING);
    assert(output.turn_radius_identified);
    assert(output.turn_radius_m > 3.0f && output.turn_radius_m < 5.0f);
    assert(output.brake_demand.left == 0.0f && output.brake_demand.right == 0.0f);

    input.now_ms += 10u;
    input.has_fresh_gnss_velocity = false;
    dcs_runtime_update(&state, &config, &input, &output);
    assert(output.mode == DCS_RUNTIME_TRIM_SAMPLING);
    for (uint32_t i = 0u; i < 3u; ++i) {
        input.now_ms += 100u;
        input.has_fresh_gnss_velocity = true;
        input.gnss_epoch_ms += 100u;
        input.gnss_system_time_ms = input.now_ms;
        input.navigation.ground_v_north_mps = 6.0f;
        input.navigation.ground_v_east_mps = -3.0f;
        dcs_runtime_update(&state, &config, &input, &output);
    }
    assert(output.mode == DCS_RUNTIME_GUIDANCE);
    assert(output.identification_complete);
    assert(output.trim_speed_identified);
    assert(fabsf(output.horizontal_airspeed_mps - 4.0f) < 1.0e-3f);
    assert(fabsf(output.wind_north_mps - 2.0f) < 1.0e-3f);
    assert(fabsf(output.wind_east_mps + 3.0f) < 1.0e-3f);
    assert(sizeof(DcsRuntimeState) < 4096u);

    /* Failed identification continues with an explicit fallback. */
    DcsRuntimeConfig fallback_config = test_config();
    fallback_config.canopy_settling_ms = 1u;
    fallback_config.identification_timeout_ms = 1u;
    DcsRuntimeState fallback_state;
    DcsRuntimeInput fallback_input = input;
    fallback_input.now_ms = 0u;
    fallback_input.has_fresh_gnss_velocity = false;
    fallback_input.navigation.height_above_target_m = 600.0f;
    fallback_input.heading_valid = false;
    assert(dcs_runtime_start(&fallback_state, &fallback_config, 0u));
    dcs_runtime_update(&fallback_state, &fallback_config, &fallback_input, &output);
    fallback_input.now_ms = 1u;
    dcs_runtime_update(&fallback_state, &fallback_config, &fallback_input, &output);
    fallback_input.now_ms = 2u;
    dcs_runtime_update(&fallback_state, &fallback_config, &fallback_input, &output);
    assert(output.mode == DCS_RUNTIME_GUIDANCE);
    assert(output.identification_failed);
    assert(output.steering_command == 0.0f);
    assert(output.brake_demand.left == 0.0f && output.brake_demand.right == 0.0f);

    fallback_input.now_ms = 3u;
    fallback_input.navigation.height_above_target_m = 4.0f;
    dcs_runtime_update(&fallback_state, &fallback_config, &fallback_input, &output);
    assert(output.mode == DCS_RUNTIME_FLARE);
    assert(fabsf(output.brake_demand.left - 0.8f) < 1.0e-5f);
    assert(fabsf(output.brake_demand.right - 0.8f) < 1.0e-5f);

    puts("dcs_runtime tests passed");
    return 0;
}
