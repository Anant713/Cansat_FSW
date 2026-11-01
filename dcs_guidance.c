#include "dcs_guidance.h"

#include <math.h>
#include <stddef.h>

#define DCS_GUIDANCE_PI_F 3.14159265358979323846f
#define DCS_GUIDANCE_TWO_PI_F (2.0f * DCS_GUIDANCE_PI_F)

static float clampf(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static float wrap_positive(float angle)
{
    float wrapped = fmodf(angle, DCS_GUIDANCE_TWO_PI_F);
    return wrapped < 0.0f ? wrapped + DCS_GUIDANCE_TWO_PI_F : wrapped;
}

static float wrap_signed(float angle)
{
    float wrapped = wrap_positive(angle + DCS_GUIDANCE_PI_F);
    return wrapped - DCS_GUIDANCE_PI_F;
}

static bool finite_point(DcsPathPoint point)
{
    return isfinite(point.north_m) && isfinite(point.east_m);
}

static bool valid_config(const DcsGuidanceConfig *config)
{
    return config != NULL &&
        isfinite(config->los_lookahead_m) && config->los_lookahead_m > 0.0f &&
        isfinite(config->horizontal_error_threshold_m) &&
        config->horizontal_error_threshold_m > 0.0f &&
        isfinite(config->height_shortage_threshold_m) &&
        config->height_shortage_threshold_m > 0.0f &&
        isfinite(config->wind_change_fraction) && config->wind_change_fraction > 0.0f &&
        config->horizontal_persistence_ms > 0u &&
        config->height_persistence_ms > 0u &&
        config->wind_persistence_ms > 0u &&
        config->reassessment_min_interval_ms > 0u &&
        isfinite(config->segment_end_tolerance_m) &&
        config->segment_end_tolerance_m >= 0.0f &&
        isfinite(config->loiter_radius_m) &&
        config->loiter_radius_m >= config->planner.turn_radius_m &&
        config->loiter_search_steps >= 4u &&
        config->loiter_search_steps <= DCS_GUIDANCE_MAX_LOITER_SEARCH_STEPS &&
        isfinite(config->flare_height_m) && config->flare_height_m >= 0.0f;
}

static float segment_length(const DcsPathSegment *segment)
{
    if (segment->type == DCS_PATH_STRAIGHT) {
        return hypotf(segment->geometry.straight.end.north_m -
                      segment->geometry.straight.start.north_m,
                      segment->geometry.straight.end.east_m -
                      segment->geometry.straight.start.east_m);
    }
    return segment->geometry.arc.radius_m * fabsf(segment->geometry.arc.sweep_rad);
}

static bool ground_course(const DcsGuidanceInput *input, float *course)
{
    if (!isfinite(input->ground_v_north_mps) || !isfinite(input->ground_v_east_mps) ||
        hypotf(input->ground_v_north_mps, input->ground_v_east_mps) < 0.05f) {
        return false;
    }
    *course = wrap_positive(atan2f(input->ground_v_east_mps,
                                   input->ground_v_north_mps));
    return true;
}

static DcsPlannerInput planner_input(const DcsGuidanceInput *input, float course)
{
    return (DcsPlannerInput){
        .position = input->position,
        .ground_course_rad = course,
        .height_above_target_m = input->height_above_target_m,
        .wind_north_mps = input->wind_north_mps,
        .wind_east_mps = input->wind_east_mps,
        .has_wind_estimate = input->has_wind_estimate,
    };
}

static bool usable_plan(DcsPlannerResult result)
{
    return result == DCS_PLAN_NORMAL_APPROACH || result == DCS_PLAN_EXCESS_HEIGHT;
}

static void activate_plan(DcsGuidanceState *state,
                          const DcsPlannerOutput *plan,
                          bool after_energy)
{
    state->active_plan = *plan;
    state->active_segment = 0u;
    state->completed_distance_m = 0.0f;
    state->segment_progress_m = 0.0f;
    state->has_previous_arc_angle = false;
    state->energy_handled = after_energy || !plan->has_energy_point;
    state->final_entry_handled = false;
    state->target_passed = false;
    state->target_reassessment_done = false;
    state->mode = state->energy_handled ? DCS_GUIDANCE_TRACK_FINAL
                                       : DCS_GUIDANCE_TRACK_TO_ENERGY;
}

static DcsPlannerResult generate_plan(const DcsGuidanceState *state,
                                      const DcsGuidanceConfig *config,
                                      const DcsGuidanceInput *input,
                                      bool after_energy,
                                      DcsPlannerOutput *plan)
{
    float course;
    if (!input->horizontal_valid || !input->vertical_valid ||
        !finite_point(input->position) || !isfinite(input->height_above_target_m) ||
        !ground_course(input, &course)) {
        return DCS_PLAN_INVALID_INPUT;
    }
    DcsPlannerConfig planner_config = config->planner;
    planner_config.connect_directly_to_final_entry = after_energy;
    if (after_energy && state != NULL && state->initialized) {
        planner_config.use_final_course_override = true;
        planner_config.final_course_override_rad =
            state->active_plan.selected_final_course_rad;
    }
    const DcsPlannerInput input_plan = planner_input(input, course);
    return dcs_planner_generate(&input_plan, &planner_config, plan);
}

bool dcs_guidance_start(DcsGuidanceState *state,
                        const DcsGuidanceConfig *config,
                        const DcsGuidanceInput *input)
{
    if (state == NULL || input == NULL || !valid_config(config)) {
        return false;
    }
    *state = (DcsGuidanceState){0};
    DcsPlannerOutput plan;
    const DcsPlannerResult result = generate_plan(NULL, config, input, false, &plan);
    if (!usable_plan(result)) {
        return false;
    }
    activate_plan(state, &plan, false);
    state->assessed_wind_north_mps = input->wind_north_mps;
    state->assessed_wind_east_mps = input->wind_east_mps;
    state->assessed_has_wind = input->has_wind_estimate;
    state->last_reassessment_ms = input->now_ms;
    state->has_reassessment_time = true;
    state->initialized = true;
    return true;
}

static DcsPathPoint point_on_segment(const DcsPathSegment *segment, float distance)
{
    if (segment->type == DCS_PATH_STRAIGHT) {
        const float dn = segment->geometry.straight.end.north_m -
                         segment->geometry.straight.start.north_m;
        const float de = segment->geometry.straight.end.east_m -
                         segment->geometry.straight.start.east_m;
        const float length = hypotf(dn, de);
        if (length <= 0.0f) {
            return segment->geometry.straight.end;
        }
        const float fraction = distance / length;
        return (DcsPathPoint){
            segment->geometry.straight.start.north_m + fraction * dn,
            segment->geometry.straight.start.east_m + fraction * de,
        };
    }
    const float sign = segment->geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
    const float angle = segment->geometry.arc.start_angle_rad +
        sign * distance / segment->geometry.arc.radius_m;
    return (DcsPathPoint){
        segment->geometry.arc.centre.north_m + segment->geometry.arc.radius_m * cosf(angle),
        segment->geometry.arc.centre.east_m + segment->geometry.arc.radius_m * sinf(angle),
    };
}

static float project_segment(const DcsPathSegment *segment,
                             DcsPathPoint position,
                             DcsPathPoint *projection,
                             float *cross_track)
{
    if (segment->type == DCS_PATH_STRAIGHT) {
        const float dn = segment->geometry.straight.end.north_m -
                         segment->geometry.straight.start.north_m;
        const float de = segment->geometry.straight.end.east_m -
                         segment->geometry.straight.start.east_m;
        const float length = hypotf(dn, de);
        if (length <= 0.0f) {
            *projection = segment->geometry.straight.end;
            *cross_track = hypotf(position.north_m - projection->north_m,
                                  position.east_m - projection->east_m);
            return 0.0f;
        }
        const float tn = dn / length, te = de / length;
        const float rn = position.north_m - segment->geometry.straight.start.north_m;
        const float re = position.east_m - segment->geometry.straight.start.east_m;
        const float along = rn * tn + re * te;
        *projection = (DcsPathPoint){
            segment->geometry.straight.start.north_m + clampf(along, 0.0f, length) * tn,
            segment->geometry.straight.start.east_m + clampf(along, 0.0f, length) * te,
        };
        *cross_track = fabsf(tn * re - te * rn);
        return along;
    }
    const float dn = position.north_m - segment->geometry.arc.centre.north_m;
    const float de = position.east_m - segment->geometry.arc.centre.east_m;
    const float distance = hypotf(dn, de);
    const float angle = distance > 0.0f ? atan2f(de, dn)
                                       : segment->geometry.arc.start_angle_rad;
    *projection = (DcsPathPoint){
        segment->geometry.arc.centre.north_m + segment->geometry.arc.radius_m * cosf(angle),
        segment->geometry.arc.centre.east_m + segment->geometry.arc.radius_m * sinf(angle),
    };
    *cross_track = fabsf(distance - segment->geometry.arc.radius_m);
    const float sign = segment->geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
    return sign * wrap_signed(angle - segment->geometry.arc.start_angle_rad) *
           segment->geometry.arc.radius_m;
}

static void advance_plan_progress(DcsGuidanceState *state,
                                  const DcsGuidanceConfig *config,
                                  DcsPathPoint position,
                                  float *cross_track)
{
    if (state->active_segment >= state->active_plan.segment_count) {
        DcsPathPoint projection;
        (void)project_segment(&state->active_plan.segments[
            state->active_plan.segment_count - 1u], position,
            &projection, cross_track);
        state->target_passed = true;
        return;
    }
    while (state->active_segment < state->active_plan.segment_count) {
        const DcsPathSegment *segment = &state->active_plan.segments[state->active_segment];
        const float length = segment_length(segment);
        DcsPathPoint projection;
        float projected_progress = project_segment(segment, position, &projection, cross_track);
        if (segment->type == DCS_PATH_ARC) {
            const float angle = atan2f(position.east_m - segment->geometry.arc.centre.east_m,
                                       position.north_m - segment->geometry.arc.centre.north_m);
            if (state->has_previous_arc_angle) {
                const float sign = segment->geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
                state->segment_progress_m += sign * wrap_signed(angle - state->previous_arc_angle_rad) *
                                             segment->geometry.arc.radius_m;
            } else {
                state->segment_progress_m = fmaxf(0.0f, projected_progress);
                state->has_previous_arc_angle = true;
            }
            state->previous_arc_angle_rad = angle;
        } else {
            state->segment_progress_m = projected_progress;
        }
        if (state->segment_progress_m + config->segment_end_tolerance_m < length) {
            return;
        }
        state->completed_distance_m += length;
        state->active_segment++;
        state->segment_progress_m = 0.0f;
        state->has_previous_arc_angle = false;
        if (state->active_segment >= state->active_plan.segment_count) {
            state->target_passed = true;
            return;
        }
    }
}

static DcsPathPoint los_point(const DcsGuidanceState *state,
                              const DcsGuidanceConfig *config,
                              DcsPathPoint position)
{
    if (state->active_segment >= state->active_plan.segment_count) {
        const DcsPathSegment *last = &state->active_plan.segments[
            state->active_plan.segment_count - 1u];
        DcsPathPoint projection;
        float cross_track;
        const float along = project_segment(last, position, &projection, &cross_track);
        return point_on_segment(last, fmaxf(segment_length(last), along) +
                                      config->los_lookahead_m);
    }
    uint8_t index = state->active_segment;
    DcsPathPoint projection = position;
    float cross_track = 0.0f;
    float progress;
    if (state->active_plan.segments[index].type == DCS_PATH_ARC) {
        progress = state->segment_progress_m;
    } else {
        progress = project_segment(&state->active_plan.segments[index], position,
                                   &projection, &cross_track);
    }
    progress = clampf(progress, 0.0f,
                      segment_length(&state->active_plan.segments[index]));
    float remaining = config->los_lookahead_m;
    while (index < state->active_plan.segment_count) {
        const DcsPathSegment *segment = &state->active_plan.segments[index];
        const float length = segment_length(segment);
        const float available = length - progress;
        if (remaining <= available || index + 1u == state->active_plan.segment_count) {
            return point_on_segment(segment, progress + remaining);
        }
        remaining -= available;
        index++;
        progress = 0.0f;
    }
    return projection;
}

static bool heading_for_course(const DcsGuidanceConfig *config,
                               const DcsGuidanceInput *input,
                               float course,
                               float *heading)
{
    const float wind_n = input->has_wind_estimate ? input->wind_north_mps : 0.0f;
    const float wind_e = input->has_wind_estimate ? input->wind_east_mps : 0.0f;
    const float crosswind = -wind_n * sinf(course) + wind_e * cosf(course);
    if (fabsf(crosswind) >= config->planner.horizontal_airspeed_mps) {
        return false;
    }
    *heading = wrap_positive(course - asinf(crosswind /
                            config->planner.horizontal_airspeed_mps));
    return isfinite(*heading);
}

static DcsPathSegment make_loiter(DcsPathPoint start, float course,
                                  float radius, float sweep)
{
    const float sign = sweep >= 0.0f ? 1.0f : -1.0f;
    DcsPathSegment segment = {.type = DCS_PATH_ARC};
    segment.geometry.arc.radius_m = radius;
    segment.geometry.arc.centre.north_m = start.north_m - sign * radius * sinf(course);
    segment.geometry.arc.centre.east_m = start.east_m + sign * radius * cosf(course);
    segment.geometry.arc.start_angle_rad = atan2f(
        start.east_m - segment.geometry.arc.centre.east_m,
        start.north_m - segment.geometry.arc.centre.north_m);
    segment.geometry.arc.sweep_rad = sweep;
    return segment;
}

static void loiter_exit(const DcsPathSegment *arc, DcsPathPoint *position,
                        float *course)
{
    const float end_angle = arc->geometry.arc.start_angle_rad + arc->geometry.arc.sweep_rad;
    const float sign = arc->geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
    *position = (DcsPathPoint){
        arc->geometry.arc.centre.north_m + arc->geometry.arc.radius_m * cosf(end_angle),
        arc->geometry.arc.centre.east_m + arc->geometry.arc.radius_m * sinf(end_angle),
    };
    *course = wrap_positive(end_angle + sign * 0.5f * DCS_GUIDANCE_PI_F);
}

static bool evaluate_loiter_arc(const DcsGuidanceConfig *config,
                                const DcsGuidanceInput *input,
                                const DcsPathSegment *arc,
                                float *height_loss)
{
    DcsPathEvaluation evaluation;
    const float wn = input->has_wind_estimate ? input->wind_north_mps : 0.0f;
    const float we = input->has_wind_estimate ? input->wind_east_mps : 0.0f;
    if (dcs_planner_evaluate_segments(arc, 1u, wn, we,
                                      &config->planner, &evaluation) !=
        DCS_PATH_EVALUATION_OK) {
        return false;
    }
    *height_loss = evaluation.predicted_height_loss_m;
    return true;
}

static DcsPlannerResult plan_from_loiter_exit(const DcsGuidanceState *state,
                                              const DcsGuidanceConfig *config,
                                              const DcsGuidanceInput *input,
                                              const DcsPathSegment *arc,
                                              float loiter_loss,
                                              DcsPlannerOutput *plan)
{
    DcsPathPoint exit_position;
    float exit_course;
    loiter_exit(arc, &exit_position, &exit_course);
    DcsGuidanceInput exit_input = *input;
    exit_input.position = exit_position;
    exit_input.ground_v_north_mps = cosf(exit_course);
    exit_input.ground_v_east_mps = sinf(exit_course);
    exit_input.height_above_target_m -= loiter_loss;
    return generate_plan(state, config, &exit_input, true, plan);
}

static bool select_loiter(DcsGuidanceState *state,
                          const DcsGuidanceConfig *config,
                          const DcsGuidanceInput *input)
{
    float course;
    if (!ground_course(input, &course)) {
        return false;
    }
    bool found_partial = false;
    float best_margin = INFINITY;
    DcsPathSegment best_arc = {0};
    const float preferred_sign = config->prefer_right_loiter ? 1.0f : -1.0f;

    for (unsigned int side_index = 0u; side_index < 2u; ++side_index) {
        const float sign = side_index == 0u ? preferred_sign : -preferred_sign;
        for (uint8_t step = 1u; step < config->loiter_search_steps; ++step) {
            const float sweep = sign * DCS_GUIDANCE_TWO_PI_F *
                (float)step / (float)config->loiter_search_steps;
            const DcsPathSegment arc = make_loiter(input->position, course,
                                                   config->loiter_radius_m, sweep);
            float loiter_loss;
            DcsPlannerOutput exit_plan;
            if (!evaluate_loiter_arc(config, input, &arc, &loiter_loss)) {
                continue;
            }
            const DcsPlannerResult result = plan_from_loiter_exit(state, config, input,
                                                                  &arc, loiter_loss,
                                                                  &exit_plan);
            if (result != DCS_PLAN_NORMAL_APPROACH) {
                continue;
            }
            const float margin = input->height_above_target_m - loiter_loss -
                exit_plan.predicted_height_loss_m - config->planner.height_reserve_m;
            if (margin >= 0.0f && margin < best_margin) {
                best_margin = margin;
                best_arc = arc;
                found_partial = true;
            }
        }
    }
    if (found_partial) {
        state->loiter_segment = best_arc;
    } else {
        const DcsPathSegment full = make_loiter(input->position, course,
            config->loiter_radius_m, preferred_sign * DCS_GUIDANCE_TWO_PI_F);
        float loiter_loss;
        DcsPlannerOutput exit_plan;
        if (!evaluate_loiter_arc(config, input, &full, &loiter_loss) ||
            !usable_plan(plan_from_loiter_exit(state, config, input, &full,
                                               loiter_loss, &exit_plan))) {
            return false;
        }
        state->loiter_segment = full;
    }
    state->loiter_progress_rad = 0.0f;
    state->has_previous_loiter_angle = false;
    state->mode = DCS_GUIDANCE_LOITER;
    return true;
}

static void decide_at_energy(DcsGuidanceState *state,
                             const DcsGuidanceConfig *config,
                             const DcsGuidanceInput *input)
{
    DcsPlannerOutput direct_plan;
    const DcsPlannerResult result = generate_plan(state, config, input, true, &direct_plan);
    state->energy_handled = true;
    if (result == DCS_PLAN_NORMAL_APPROACH) {
        activate_plan(state, &direct_plan, true);
    } else if (result == DCS_PLAN_EXCESS_HEIGHT && select_loiter(state, config, input)) {
        return;
    } else if (usable_plan(result)) {
        activate_plan(state, &direct_plan, true);
    } else {
        state->mode = DCS_GUIDANCE_DIRECT_TARGET;
    }
}

static bool update_loiter(DcsGuidanceState *state,
                          const DcsGuidanceConfig *config,
                          const DcsGuidanceInput *input,
                          float *cross_track)
{
    DcsPathPoint projection;
    (void)project_segment(&state->loiter_segment, input->position,
                          &projection, cross_track);
    const float angle = atan2f(input->position.east_m -
                               state->loiter_segment.geometry.arc.centre.east_m,
                               input->position.north_m -
                               state->loiter_segment.geometry.arc.centre.north_m);
    const float sign = state->loiter_segment.geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
    if (state->has_previous_loiter_angle) {
        state->loiter_progress_rad += sign * wrap_signed(angle - state->previous_loiter_angle_rad);
    } else {
        state->has_previous_loiter_angle = true;
    }
    state->previous_loiter_angle_rad = angle;
    if (state->loiter_progress_rad +
        config->segment_end_tolerance_m / config->loiter_radius_m <
        fabsf(state->loiter_segment.geometry.arc.sweep_rad)) {
        return false;
    }

    DcsPlannerOutput plan;
    const DcsPlannerResult result = generate_plan(state, config, input, true, &plan);
    if (result == DCS_PLAN_EXCESS_HEIGHT &&
        fabsf(state->loiter_segment.geometry.arc.sweep_rad) >=
            DCS_GUIDANCE_TWO_PI_F - 0.01f &&
        select_loiter(state, config, input)) {
        return true;
    }
    if (usable_plan(result)) {
        activate_plan(state, &plan, true);
    } else {
        state->mode = DCS_GUIDANCE_DIRECT_TARGET;
    }
    return true;
}

static DcsPathPoint loiter_los(const DcsGuidanceState *state,
                               const DcsGuidanceConfig *config)
{
    const float progress = fmaxf(0.0f, state->loiter_progress_rad) *
                           state->loiter_segment.geometry.arc.radius_m;
    return point_on_segment(&state->loiter_segment, progress + config->los_lookahead_m);
}

static bool timer_elapsed(bool condition, bool *active, uint32_t *since,
                          uint32_t now, uint32_t duration)
{
    if (!condition) {
        *active = false;
        return false;
    }
    if (!*active) {
        *active = true;
        *since = now;
        return false;
    }
    return now - *since >= duration;
}

static bool wind_changed(const DcsGuidanceState *state,
                         const DcsGuidanceConfig *config,
                         const DcsGuidanceInput *input)
{
    if (!input->has_wind_estimate) {
        return false;
    }
    if (!state->assessed_has_wind) {
        return true;
    }
    const float reference = hypotf(state->assessed_wind_north_mps,
                                   state->assessed_wind_east_mps);
    const float latest = hypotf(input->wind_north_mps, input->wind_east_mps);
    const bool reference_weak = reference < config->planner.weak_wind_threshold_mps;
    const bool latest_weak = latest < config->planner.weak_wind_threshold_mps;
    if (reference_weak && latest_weak) {
        return false;
    }
    if (reference_weak != latest_weak) {
        return true;
    }
    return hypotf(input->wind_north_mps - state->assessed_wind_north_mps,
                  input->wind_east_mps - state->assessed_wind_east_mps) /
           reference > config->wind_change_fraction;
}

static uint8_t build_remaining(const DcsGuidanceState *state,
                               DcsPathPoint position,
                               DcsPathSegment remaining[DCS_PLANNER_MAX_SEGMENTS])
{
    if (state->active_segment >= state->active_plan.segment_count) {
        return 0u;
    }
    uint8_t count = 0u;
    const DcsPathSegment *current = &state->active_plan.segments[state->active_segment];
    remaining[count] = *current;
    if (current->type == DCS_PATH_STRAIGHT) {
        DcsPathPoint projection;
        float cross;
        (void)project_segment(current, position, &projection, &cross);
        remaining[count].geometry.straight.start = projection;
    } else {
        const float angle = atan2f(position.east_m - current->geometry.arc.centre.east_m,
                                   position.north_m - current->geometry.arc.centre.north_m);
        const float sign = current->geometry.arc.sweep_rad >= 0.0f ? 1.0f : -1.0f;
        const float total = fabsf(current->geometry.arc.sweep_rad);
        const float progressed = clampf(state->segment_progress_m /
            current->geometry.arc.radius_m, 0.0f, total);
        remaining[count].geometry.arc.start_angle_rad = angle;
        remaining[count].geometry.arc.sweep_rad = sign * (total - progressed);
    }
    count++;
    for (uint8_t i = state->active_segment + 1u;
         i < state->active_plan.segment_count && count < DCS_PLANNER_MAX_SEGMENTS;
         ++i) {
        remaining[count++] = state->active_plan.segments[i];
    }
    return count;
}

static bool remaining_height_loss(const DcsGuidanceState *state,
                                  const DcsGuidanceConfig *config,
                                  const DcsGuidanceInput *input,
                                  float *height_loss)
{
    DcsPathSegment segments[DCS_PLANNER_MAX_SEGMENTS];
    const uint8_t count = build_remaining(state, input->position, segments);
    if (count == 0u) {
        return false;
    }
    DcsPathEvaluation evaluation;
    const float wn = input->has_wind_estimate ? input->wind_north_mps : 0.0f;
    const float we = input->has_wind_estimate ? input->wind_east_mps : 0.0f;
    if (dcs_planner_evaluate_segments(segments, count, wn, we,
                                      &config->planner, &evaluation) !=
        DCS_PATH_EVALUATION_OK) {
        return false;
    }
    *height_loss = evaluation.predicted_height_loss_m;
    return true;
}

static bool remaining_feasible(const DcsGuidanceState *state,
                               const DcsGuidanceConfig *config,
                               const DcsGuidanceInput *input,
                               float *height_loss)
{
    if (!remaining_height_loss(state, config, input, height_loss)) {
        return false;
    }
    const float wind_speed = input->has_wind_estimate
        ? hypotf(input->wind_north_mps, input->wind_east_mps) : 0.0f;
    if (wind_speed >= config->planner.weak_wind_threshold_mps) {
        const float final_course = state->active_plan.selected_final_course_rad;
        const float final_tailwind = input->wind_north_mps * cosf(final_course) +
                                     input->wind_east_mps * sinf(final_course);
        if (final_tailwind > 0.0f) {
            return false;
        }
    }
    return !input->vertical_valid || input->height_above_target_m +
            config->height_shortage_threshold_m >=
            *height_loss + config->planner.height_reserve_m;
}

static void reassess(DcsGuidanceState *state,
                     const DcsGuidanceConfig *config,
                     const DcsGuidanceInput *input,
                     bool horizontal_unacceptable,
                     DcsGuidanceOutput *output)
{
    output->replan_attempted = true;
    float remaining_loss;
    if (!horizontal_unacceptable &&
        remaining_feasible(state, config, input, &remaining_loss)) {
        state->assessed_wind_north_mps = input->wind_north_mps;
        state->assessed_wind_east_mps = input->wind_east_mps;
        state->assessed_has_wind = input->has_wind_estimate;
        state->last_reassessment_ms = input->now_ms;
        state->has_reassessment_time = true;
        return;
    }
    DcsPlannerOutput plan;
    const bool after_energy = state->energy_handled;
    const DcsPlannerResult result = generate_plan(state, config, input, after_energy, &plan);
    if (usable_plan(result)) {
        activate_plan(state, &plan, after_energy);
        output->replan_succeeded = true;
    }
    state->assessed_wind_north_mps = input->wind_north_mps;
    state->assessed_wind_east_mps = input->wind_east_mps;
    state->assessed_has_wind = input->has_wind_estimate;
    state->last_reassessment_ms = input->now_ms;
    state->has_reassessment_time = true;
}

void dcs_guidance_update(DcsGuidanceState *state,
                         const DcsGuidanceConfig *config,
                         const DcsGuidanceInput *input,
                         DcsGuidanceOutput *output)
{
    if (output == NULL) {
        return;
    }
    *output = (DcsGuidanceOutput){0};
    if (state == NULL || input == NULL || !state->initialized || !valid_config(config)) {
        output->mode = DCS_GUIDANCE_INACTIVE;
        return;
    }
    if (state->mode != DCS_GUIDANCE_FLARE && input->vertical_valid &&
        input->height_above_target_m <= config->flare_height_m) {
        state->mode = DCS_GUIDANCE_FLARE;
    }
    output->mode = state->mode;
    output->active_segment = state->active_segment;
    output->using_propagated_position = input->horizontal_propagated;
    if (state->mode == DCS_GUIDANCE_FLARE) {
        output->flare_requested = true;
        return;
    }
    if (!input->horizontal_valid || !finite_point(input->position)) {
        if (state->has_last_valid_heading) {
            output->desired_heading_rad = state->last_valid_heading_rad;
            output->heading_command_valid = true;
        }
        return;
    }

    float cross_track = 0.0f;
    DcsPathPoint lookahead = input->position;
    if (state->mode == DCS_GUIDANCE_LOITER) {
        (void)update_loiter(state, config, input, &cross_track);
        if (state->mode == DCS_GUIDANCE_LOITER) {
            lookahead = loiter_los(state, config);
        }
    }
    if (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
        state->mode == DCS_GUIDANCE_TRACK_FINAL) {
        advance_plan_progress(state, config, input->position, &cross_track);
        const float total_progress = state->completed_distance_m +
                                     fmaxf(0.0f, state->segment_progress_m);
        bool handled_energy_now = false;
        if (!state->energy_handled && state->active_plan.has_energy_point &&
            total_progress + config->segment_end_tolerance_m >=
                state->active_plan.energy_point_distance_m) {
            decide_at_energy(state, config, input);
            handled_energy_now = true;
        }
        if (!handled_energy_now &&
            (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
             state->mode == DCS_GUIDANCE_TRACK_FINAL) &&
            !state->final_entry_handled &&
            total_progress + config->segment_end_tolerance_m >=
                state->active_plan.final_approach_entry_distance_m) {
            state->final_entry_handled = true;
            state->mode = DCS_GUIDANCE_TRACK_FINAL;
        }
        if (state->target_passed) {
            output->target_passed = true;
        }
        if (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
            state->mode == DCS_GUIDANCE_TRACK_FINAL) {
            lookahead = los_point(state, config, input->position);
        } else if (state->mode == DCS_GUIDANCE_LOITER) {
            lookahead = loiter_los(state, config);
        }
    }

    if (state->mode == DCS_GUIDANCE_DIRECT_TARGET) {
        lookahead = (DcsPathPoint){0.0f, 0.0f};
    }
    output->cross_track_error_m = cross_track;

    const bool horizontal_trigger = timer_elapsed(
        cross_track > config->horizontal_error_threshold_m,
        &state->horizontal_timer_active, &state->horizontal_since_ms,
        input->now_ms, config->horizontal_persistence_ms);
    float remaining_loss = 0.0f;
    const bool has_remaining_loss =
        (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
         state->mode == DCS_GUIDANCE_TRACK_FINAL) &&
        remaining_height_loss(state, config, input, &remaining_loss);
    const bool height_shortage = input->vertical_valid && has_remaining_loss &&
        remaining_loss + config->planner.height_reserve_m -
            input->height_above_target_m > config->height_shortage_threshold_m;
    const bool height_trigger = timer_elapsed(height_shortage,
        &state->height_timer_active, &state->height_since_ms,
        input->now_ms, config->height_persistence_ms);
    const bool wind_trigger = timer_elapsed(wind_changed(state, config, input),
        &state->wind_timer_active, &state->wind_since_ms,
        input->now_ms, config->wind_persistence_ms);
    const bool interval_ready = !state->has_reassessment_time ||
        input->now_ms - state->last_reassessment_ms >=
            config->reassessment_min_interval_ms;
    const bool target_request = state->target_passed &&
                                !state->target_reassessment_done;
    if ((target_request || (interval_ready &&
         (horizontal_trigger || height_trigger || wind_trigger))) &&
        state->mode != DCS_GUIDANCE_LOITER &&
        state->mode != DCS_GUIDANCE_DIRECT_TARGET) {
        reassess(state, config, input, horizontal_trigger, output);
        if (target_request && !output->replan_succeeded) {
            state->target_reassessment_done = true;
        }
        state->horizontal_timer_active = false;
        state->height_timer_active = false;
        state->wind_timer_active = false;
        if (output->replan_succeeded &&
            (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
             state->mode == DCS_GUIDANCE_TRACK_FINAL)) {
            lookahead = los_point(state, config, input->position);
        }
    }

    const float dn = lookahead.north_m - input->position.north_m;
    const float de = lookahead.east_m - input->position.east_m;
    if (hypotf(dn, de) > 1.0e-4f) {
        output->desired_ground_course_rad = wrap_positive(atan2f(de, dn));
        float heading;
        if (heading_for_course(config, input, output->desired_ground_course_rad, &heading)) {
            output->desired_heading_rad = heading;
            output->heading_command_valid = true;
            output->wind_correction_achievable = true;
            state->last_valid_heading_rad = heading;
            state->has_last_valid_heading = true;
        } else if (state->has_last_valid_heading) {
            output->desired_heading_rad = state->last_valid_heading_rad;
            output->heading_command_valid = true;
        }
        if (!output->wind_correction_achievable && interval_ready &&
            !output->replan_attempted &&
            (state->mode == DCS_GUIDANCE_TRACK_TO_ENERGY ||
             state->mode == DCS_GUIDANCE_TRACK_FINAL)) {
            reassess(state, config, input, false, output);
        }
    }
    output->mode = state->mode;
    output->active_segment = state->active_segment;
    output->target_passed = state->target_passed;
}
