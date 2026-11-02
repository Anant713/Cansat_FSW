#include "dcs_planner.h"

#include "dcs_config.h"
#include "dcs_dubins.h"

#include <math.h>
#include <stddef.h>

#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

#if DCS_FLIGHT_ARC_TIME_INTERVALS == 0 || (DCS_FLIGHT_ARC_TIME_INTERVALS % 2) != 0
#error "Arc integration requires a positive even interval count"
#endif

typedef enum {
    EVALUATION_OK,
    EVALUATION_WIND_INFEASIBLE,
    EVALUATION_NUMERIC_FAILURE
} EvaluationResult;

static bool valid_config(const DcsPlannerConfig *config)
{
    return config != NULL &&
        isfinite(config->turn_radius_m) && config->turn_radius_m > 0.0f &&
        isfinite(config->horizontal_airspeed_mps) && config->horizontal_airspeed_mps > 0.0f &&
        isfinite(config->straight_sink_rate_mps) && config->straight_sink_rate_mps > 0.0f &&
        isfinite(config->turn_loss_fraction) && config->turn_loss_fraction >= 0.0f &&
        isfinite(config->final_approach_length_m) && config->final_approach_length_m > 0.0f &&
        isfinite(config->energy_point_upstream_m) && config->energy_point_upstream_m >= 0.0f &&
        isfinite(config->height_reserve_m) && config->height_reserve_m >= 0.0f &&
        isfinite(config->height_tolerance_fraction) &&
        config->height_tolerance_fraction >= 0.0f && config->height_tolerance_fraction < 1.0f &&
        isfinite(config->weak_wind_threshold_mps) && config->weak_wind_threshold_mps > 0.0f &&
        (!config->use_final_course_override || isfinite(config->final_course_override_rad));
}

static float wrap_angle(float angle)
{
    float result = fmodf(angle, TWO_PI_F);
    return result < 0.0f ? result + TWO_PI_F : result;
}

static float select_final_course(const DcsPlannerInput *input,
                                 const DcsPlannerConfig *config,
                                 float wind_n, float wind_e, float wind_speed)
{
    if (config->use_final_course_override) {
        return wrap_angle(config->final_course_override_rad);
    }
    if (wind_speed >= config->weak_wind_threshold_mps) {
        return wrap_angle(atan2f(-wind_e, -wind_n));
    }
    if (input->position.north_m != 0.0f || input->position.east_m != 0.0f) {
        return wrap_angle(atan2f(-input->position.east_m, -input->position.north_m));
    }
    if (wind_speed > 0.0f) {
        return wrap_angle(atan2f(-wind_e, -wind_n));
    }
    return wrap_angle(input->ground_course_rad);
}

static bool ground_speed(float course, float wind_n, float wind_e,
                         float wind_speed, float airspeed, float *speed)
{
    const float c = cosf(course), s = sinf(course);
    const float parallel = wind_n * c + wind_e * s;
    const float cross_ratio = (-wind_n * s + wind_e * c) / airspeed;
    if (fabsf(cross_ratio) > 1.0f) {
        return false;
    }
    const float along_air = airspeed * sqrtf(fmaxf(0.0f, 1.0f - cross_ratio * cross_ratio));
    if (parallel < 0.0f) {
        if (wind_speed >= airspeed) {
            return false;
        }
        /* Conjugate form avoids subtracting nearly equal values upwind. */
        *speed = (airspeed - wind_speed) *
                 ((airspeed + wind_speed) / (along_air - parallel));
    } else {
        *speed = parallel + along_air;
    }
    return isfinite(*speed) && *speed > 0.0f;
}

static EvaluationResult segment_time(const DcsPathSegment *segment,
                                     float wind_n, float wind_e, float wind_speed,
                                     float airspeed, float *time)
{
    *time = 0.0f;
    if (segment->type == DCS_PATH_STRAIGHT) {
        const float dn = segment->geometry.straight.end.north_m -
                         segment->geometry.straight.start.north_m;
        const float de = segment->geometry.straight.end.east_m -
                         segment->geometry.straight.start.east_m;
        const float length = hypotf(dn, de);
        if (length == 0.0f) {
            return EVALUATION_OK;
        }
        float speed;
        if (!ground_speed(atan2f(de, dn), wind_n, wind_e, wind_speed, airspeed, &speed)) {
            return EVALUATION_WIND_INFEASIBLE;
        }
        *time = length / speed;
    } else {
        const float sweep = segment->geometry.arc.sweep_rad;
        if (sweep == 0.0f) {
            return EVALUATION_OK;
        }
        const float sign = sweep > 0.0f ? 1.0f : -1.0f;
        const float course = segment->geometry.arc.start_angle_rad + sign * 0.5f * PI_F;
        /* If wind >= airspeed, traversable courses form one downwind cone of
         * width <= pi. Endpoints must be feasible, and an arc > pi cannot fit
         * inside it. This checks continuous feasibility, not just sample nodes.
         */
        if (wind_speed >= airspeed && fabsf(sweep) > PI_F) {
            return EVALUATION_WIND_INFEASIBLE;
        }
        float inverse_speed_sum = 0.0f;
        for (unsigned int i = 0; i <= DCS_FLIGHT_ARC_TIME_INTERVALS; ++i) {
            const float fraction = (float) i / (float) DCS_FLIGHT_ARC_TIME_INTERVALS;
            float speed;
            if (!ground_speed(course + fraction * sweep, wind_n, wind_e, wind_speed, airspeed, &speed)) {
                return EVALUATION_WIND_INFEASIBLE;
            }
            const float weight = (i == 0u || i == DCS_FLIGHT_ARC_TIME_INTERVALS)
                ? 1.0f : (i % 2u == 0u ? 2.0f : 4.0f);
            inverse_speed_sum += weight / speed;
        }
        const float length = segment->geometry.arc.radius_m * fabsf(sweep);
        *time = length * inverse_speed_sum /
                (3.0f * (float) DCS_FLIGHT_ARC_TIME_INTERVALS);
    }
    return isfinite(*time) ? EVALUATION_OK : EVALUATION_NUMERIC_FAILURE;
}

DcsPathEvaluationResult dcs_planner_evaluate_segments(
    const DcsPathSegment *segments, uint8_t segment_count,
    float wind_north_mps, float wind_east_mps,
    const DcsPlannerConfig *config, DcsPathEvaluation *evaluation)
{
    if (segments == NULL || evaluation == NULL || !valid_config(config) ||
        segment_count == 0u || segment_count > DCS_PLANNER_MAX_SEGMENTS ||
        !isfinite(wind_north_mps) || !isfinite(wind_east_mps)) {
        return DCS_PATH_EVALUATION_INVALID;
    }
    const float wind_speed = hypotf(wind_north_mps, wind_east_mps);
    float straight_time = 0.0f, turn_time = 0.0f;
    for (uint8_t i = 0u; i < segment_count; ++i) {
        float time;
        const EvaluationResult result = segment_time(&segments[i],
            wind_north_mps, wind_east_mps, wind_speed,
            config->horizontal_airspeed_mps, &time);
        if (result == EVALUATION_WIND_INFEASIBLE) {
            return DCS_PATH_EVALUATION_WIND_INFEASIBLE;
        }
        if (result != EVALUATION_OK) {
            return DCS_PATH_EVALUATION_INVALID;
        }
        if (segments[i].type == DCS_PATH_ARC) {
            turn_time += time;
        } else {
            straight_time += time;
        }
    }
    evaluation->travel_time_s = straight_time + turn_time;
    evaluation->predicted_height_loss_m = config->straight_sink_rate_mps *
        (straight_time + (1.0f + config->turn_loss_fraction) * turn_time);
    if (!isfinite(evaluation->travel_time_s) ||
        !isfinite(evaluation->predicted_height_loss_m)) {
        return DCS_PATH_EVALUATION_INVALID;
    }
    return DCS_PATH_EVALUATION_OK;
}

static bool better_candidate(DcsPlannerResult result, float turns, float length,
                              bool has_best, DcsPlannerResult best_result,
                              float best_turns, float best_length)
{
    if (!has_best) {
        return true;
    }
    if (result != best_result) {
        return result == DCS_PLAN_NORMAL_APPROACH;
    }
    if (turns < best_turns - DCS_FLIGHT_TURN_EQUALITY_RAD) {
        return true;
    }
    if (fabsf(turns - best_turns) <= DCS_FLIGHT_TURN_EQUALITY_RAD) {
        return length < best_length - DCS_FLIGHT_LENGTH_EQUALITY_M;
    }
    return false;
}

DcsPlannerResult dcs_planner_generate(const DcsPlannerInput *input,
                                     const DcsPlannerConfig *config,
                                     DcsPlannerOutput *output)
{
    if (input == NULL || output == NULL || !valid_config(config) ||
        !isfinite(input->position.north_m) || !isfinite(input->position.east_m) ||
        !isfinite(input->ground_course_rad) || !isfinite(input->height_above_target_m) ||
        (input->has_wind_estimate &&
         (!isfinite(input->wind_north_mps) || !isfinite(input->wind_east_mps)))) {
        return DCS_PLAN_INVALID_INPUT;
    }
    const float wind_n = input->has_wind_estimate ? input->wind_north_mps : 0.0f;
    const float wind_e = input->has_wind_estimate ? input->wind_east_mps : 0.0f;
    const float wind_speed = hypotf(wind_n, wind_e);
    const float connection_offset = config->final_approach_length_m +
        (config->connect_directly_to_final_entry ? 0.0f : config->energy_point_upstream_m);
    if (!isfinite(connection_offset) || !isfinite(wind_speed) ||
        !isfinite(config->horizontal_airspeed_mps + wind_speed)) {
        return DCS_PLAN_INVALID_INPUT;
    }
    const float final_course = select_final_course(input, config, wind_n, wind_e, wind_speed);
    const DcsDubinsPose start = {input->position, input->ground_course_rad};
    const DcsDubinsPose connection_pose = {
        {-connection_offset * cosf(final_course), -connection_offset * sinf(final_course)},
        final_course
    };
    DcsPlannerOutput best = {0};
    DcsPlannerResult best_result = DCS_PLAN_NO_FEASIBLE_APPROACH;
    float best_turns = 0.0f, best_length = 0.0f;
    bool has_best = false, has_wind_candidate = false, has_geometry = false;

    for (int f = 0; f < DCS_DUBINS_FAMILY_COUNT; ++f) {
        DcsDubinsPath connection;
        if (!dcs_dubins_generate(&start, &connection_pose, config->turn_radius_m,
                                 (DcsDubinsFamily) f, &connection)) {
            continue;
        }
        has_geometry = true;
        DcsPlannerOutput candidate = {0};
        for (unsigned int i = 0; i < 3u; ++i) {
            candidate.segments[i] = connection.segments[i];
        }
        candidate.segment_count = DCS_PLANNER_MAX_SEGMENTS;
        candidate.segments[3].type = DCS_PATH_STRAIGHT;
        candidate.segments[3].geometry.straight.start = connection_pose.position;
        /* Target-centred frame: zero-initialized straight.end is the target. */
        candidate.has_energy_point = !config->connect_directly_to_final_entry;
        candidate.selected_final_course_rad = final_course;
        candidate.energy_point_distance_m = connection.length_m;
        candidate.final_approach_entry_distance_m = connection.length_m +
            (config->connect_directly_to_final_entry ? 0.0f : config->energy_point_upstream_m);
        float straight_time = 0.0f, turn_time = 0.0f;
        EvaluationResult evaluation = EVALUATION_OK;
        for (unsigned int i = 0; i < DCS_PLANNER_MAX_SEGMENTS; ++i) {
            float time;
            evaluation = segment_time(&candidate.segments[i], wind_n, wind_e,
                                       wind_speed, config->horizontal_airspeed_mps, &time);
            if (evaluation != EVALUATION_OK) {
                break;
            }
            if (candidate.segments[i].type == DCS_PATH_ARC) {
                turn_time += time;
            } else {
                straight_time += time;
            }
        }
        if (evaluation == EVALUATION_NUMERIC_FAILURE) {
            return DCS_PLAN_INVALID_INPUT;
        }
        if (evaluation == EVALUATION_WIND_INFEASIBLE) {
            continue;
        }
        has_wind_candidate = true;
        candidate.predicted_travel_time_s = straight_time + turn_time;
        candidate.predicted_height_loss_m = config->straight_sink_rate_mps *
            (straight_time + (1.0f + config->turn_loss_fraction) * turn_time);
        const float tolerance = config->height_tolerance_fraction * candidate.predicted_height_loss_m;
        const float required = candidate.predicted_height_loss_m + config->height_reserve_m;
        const float error = input->height_above_target_m - required;
        const float length = connection.length_m + connection_offset;
        if (!isfinite(candidate.predicted_travel_time_s) ||
            !isfinite(candidate.predicted_height_loss_m) || !isfinite(required) ||
            !isfinite(error) || !isfinite(length)) {
            return DCS_PLAN_INVALID_INPUT;
        }
        if (error < -tolerance) {
            continue;
        }
        const DcsPlannerResult result = error > tolerance ? DCS_PLAN_EXCESS_HEIGHT : DCS_PLAN_NORMAL_APPROACH;
        if (better_candidate(result, connection.absolute_turn_rad, length, has_best,
                              best_result, best_turns, best_length)) {
            best = candidate;
            best_result = result;
            best_turns = connection.absolute_turn_rad;
            best_length = length;
            has_best = true;
        }
    }
    if (!has_geometry) {
        return DCS_PLAN_INVALID_INPUT;
    }
    if (has_best) {
        *output = best;
        return best_result;
    }
    DcsPlannerOutput failure = {0};
    failure.failure_reason = has_wind_candidate ? DCS_PLAN_FAILURE_INSUFFICIENT_HEIGHT
                                              : DCS_PLAN_FAILURE_WIND_INFEASIBLE;
    *output = failure;
    return DCS_PLAN_NO_FEASIBLE_APPROACH;
}
