#include "../dcs_guidance.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static DcsGuidanceConfig test_config(void)
{
    return (DcsGuidanceConfig){
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
    };
}

int main(void)
{
    DcsGuidanceConfig config = test_config();
    DcsGuidanceInput input = {
        .now_ms = 1000u,
        .position = {200.0f, 50.0f},
        .ground_v_north_mps = -6.0f,
        .ground_v_east_mps = -1.0f,
        .height_above_target_m = 600.0f,
        .horizontal_valid = true,
        .vertical_valid = true,
    };
    DcsGuidanceState state;
    DcsGuidanceOutput output;
    assert(dcs_guidance_start(&state, &config, &input));
    assert(state.mode == DCS_GUIDANCE_TRACK_TO_ENERGY);
    dcs_guidance_update(&state, &config, &input, &output);
    assert(output.heading_command_valid);
    assert(output.wind_correction_achievable);

    /* Put progress at qE to exercise the operational excess-height decision. */
    input.now_ms += 100u;
    input.position = state.active_plan.segments[3].geometry.straight.start;
    input.ground_v_north_mps = cosf(state.active_plan.selected_final_course_rad);
    input.ground_v_east_mps = sinf(state.active_plan.selected_final_course_rad);
    state.active_segment = 3u;
    state.completed_distance_m = state.active_plan.energy_point_distance_m;
    state.segment_progress_m = 0.0f;
    state.has_previous_arc_angle = false;
    dcs_guidance_update(&state, &config, &input, &output);
    assert(state.energy_handled);
    assert(state.mode == DCS_GUIDANCE_LOITER ||
           state.mode == DCS_GUIDANCE_TRACK_FINAL);

    input.now_ms += 100u;
    input.height_above_target_m = 4.0f;
    dcs_guidance_update(&state, &config, &input, &output);
    assert(output.mode == DCS_GUIDANCE_FLARE);
    assert(output.flare_requested);

    /* Final-line LOS remains ahead after target passage; it must not reverse. */
    DcsGuidanceState final_state;
    input.height_above_target_m = 600.0f;
    input.position = (DcsPathPoint){200.0f, 50.0f};
    input.ground_v_north_mps = -6.0f;
    input.ground_v_east_mps = -1.0f;
    input.now_ms += 100u;
    assert(dcs_guidance_start(&final_state, &config, &input));
    const float final_course = final_state.active_plan.selected_final_course_rad;
    input.position.north_m = 10.0f * cosf(final_course);
    input.position.east_m = 10.0f * sinf(final_course);
    input.ground_v_north_mps = cosf(final_course);
    input.ground_v_east_mps = sinf(final_course);
    final_state.mode = DCS_GUIDANCE_TRACK_FINAL;
    final_state.energy_handled = true;
    final_state.final_entry_handled = true;
    final_state.active_segment = final_state.active_plan.segment_count;
    final_state.target_passed = true;
    final_state.target_reassessment_done = true;
    dcs_guidance_update(&final_state, &config, &input, &output);
    assert(output.heading_command_valid);
    assert(cosf(output.desired_ground_course_rad - final_course) > 0.999f);

    puts("dcs_guidance tests passed");
    return 0;
}
