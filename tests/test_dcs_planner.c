#include "../dcs_planner.h"
#include "../dcs_config.h"
#include "../dcs_dubins.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846

static DcsPlannerConfig simulation_config(void)
{
    const DcsPlannerConfig config = {
        .turn_radius_m = DCS_FLIGHT_PLAN_TURN_RADIUS_M,
        .horizontal_airspeed_mps = DCS_FLIGHT_HORIZONTAL_AIRSPEED_MPS,
        .straight_sink_rate_mps = DCS_FLIGHT_STRAIGHT_SINK_RATE_MPS,
        .turn_loss_fraction = DCS_FLIGHT_TURN_LOSS_FRACTION,
        .final_approach_length_m = DCS_FLIGHT_FINAL_APPROACH_LENGTH_M,
        .energy_point_upstream_m = DCS_FLIGHT_ENERGY_POINT_UPSTREAM_M,
        .height_reserve_m = DCS_FLIGHT_HEIGHT_RESERVE_M,
        .height_tolerance_fraction = DCS_FLIGHT_HEIGHT_CLASS_TOLERANCE_FRACTION,
        .weak_wind_threshold_mps = DCS_FLIGHT_WEAK_WIND_THRESHOLD_MPS,
        .use_final_course_override = true,
        .final_course_override_rad = 0.0f
    };
    return config;
}

static double final_course(const DcsPlannerOutput *output)
{
    const DcsPathSegment *line = &output->segments[3];
    return atan2(line->geometry.straight.end.east_m - line->geometry.straight.start.east_m,
                 line->geometry.straight.end.north_m - line->geometry.straight.start.north_m);
}

static void check_markers(const DcsPlannerOutput *output, const DcsPlannerConfig *config)
{
    assert(output->segment_count == 4u);
    assert(output->failure_reason == DCS_PLAN_FAILURE_NONE);
    assert(output->energy_point_distance_m >= 0.0f);
    assert(fabs(output->final_approach_entry_distance_m - output->energy_point_distance_m -
                config->energy_point_upstream_m) < 0.001);
    assert(output->segments[3].type == DCS_PATH_STRAIGHT);
    assert(output->segments[3].geometry.straight.end.north_m == 0.0f);
    assert(output->segments[3].geometry.straight.end.east_m == 0.0f);
    assert(fabs(hypot(output->segments[3].geometry.straight.start.north_m,
                      output->segments[3].geometry.straight.start.east_m) -
                config->energy_point_upstream_m - config->final_approach_length_m) < 0.001);
}

static double reference_speed(double course, double wn, double we, double va)
{
    const double perpendicular = -wn * sin(course) + we * cos(course);
    const double radicand = va * va - perpendicular * perpendicular;
    if (radicand < 0.0) {
        return -1.0;
    }
    return wn * cos(course) + we * sin(course) + sqrt(radicand);
}

/* Much finer double-precision midpoint quadrature, independent of production
 * Simpson sampling. Used to check time, turn allowance, and ranking outcomes.
 */
static bool reference_times(const DcsPathSegment *segments, unsigned int count,
                            double wn, double we, double va,
                            double *straight_time, double *turn_time)
{
    *straight_time = *turn_time = 0.0;
    for (unsigned int i = 0; i < count; ++i) {
        const DcsPathSegment *segment = &segments[i];
        if (segment->type == DCS_PATH_STRAIGHT) {
            const double dn = segment->geometry.straight.end.north_m -
                              segment->geometry.straight.start.north_m;
            const double de = segment->geometry.straight.end.east_m -
                              segment->geometry.straight.start.east_m;
            const double length = hypot(dn, de);
            if (length > 0.000001) {
                const double speed = reference_speed(atan2(de, dn), wn, we, va);
                if (speed <= 0.0) {
                    return false;
                }
                *straight_time += length / speed;
            }
        } else {
            const double sweep = segment->geometry.arc.sweep_rad;
            const double course = segment->geometry.arc.start_angle_rad +
                                  (sweep >= 0.0 ? 1.0 : -1.0) * PI / 2.0;
            double sum = 0.0;
            if (sweep == 0.0) {
                continue;
            }
            for (unsigned int j = 0; j < 8192u; ++j) {
                const double speed = reference_speed(course + sweep * (j + 0.5) / 8192.0,
                                                      wn, we, va);
                if (speed <= 0.0) {
                    return false;
                }
                sum += 1.0 / speed;
            }
            *turn_time += segment->geometry.arc.radius_m * fabs(sweep) * sum / 8192.0;
        }
    }
    return true;
}

static void straight_and_height_cases(void)
{
    DcsPlannerConfig config = simulation_config();
    DcsPlannerInput input = {
        .position = {-200.0f, 0.0f}, .ground_course_rad = 0.0f,
        .height_above_target_m = 90.0f,
        /* Missing wind deliberately carries unusable values: ignored. */
        .wind_north_mps = NAN, .wind_east_mps = INFINITY, .has_wind_estimate = false
    };
    DcsPlannerOutput output;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NORMAL_APPROACH);
    check_markers(&output, &config);
    assert(fabsf(output.predicted_travel_time_s - 25.0f) < 0.001f);
    assert(fabsf(output.predicted_height_loss_m - 75.0f) < 0.001f);
    assert(fabsf(output.energy_point_distance_m - 150.0f) < 0.001f);
    assert(fabsf(output.final_approach_entry_distance_m - 170.0f) < 0.001f);
    /* Exact metric ties retain the first family, LSL. */
    assert(output.segments[0].geometry.arc.sweep_rad == 0.0f);
    assert(signbit(output.segments[0].geometry.arc.sweep_rad));
    input.height_above_target_m = 88.5f; /* -2% of predicted loss, not reserve. */
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NORMAL_APPROACH);
    input.height_above_target_m = 88.4f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NO_FEASIBLE_APPROACH);
    assert(output.failure_reason == DCS_PLAN_FAILURE_INSUFFICIENT_HEIGHT);
    assert(output.segment_count == 0u);
    input.height_above_target_m = 600.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabsf(output.predicted_height_loss_m - 75.0f) < 0.001f);

    /* Known tailwind/headwind/crosswind predictions on a straight route. */
    input.has_wind_estimate = true;
    input.wind_north_mps = 2.0f;
    input.wind_east_mps = 0.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabsf(output.predicted_travel_time_s - 20.0f) < 0.001f);
    input.wind_north_mps = -2.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(output.predicted_travel_time_s - 200.0 / 6.0) < 0.001);
    input.wind_north_mps = 0.0f;
    input.wind_east_mps = 2.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(output.predicted_travel_time_s - 200.0 / sqrt(60.0)) < 0.001);
}

static void course_selection_and_near_target(void)
{
    DcsPlannerConfig config = simulation_config();
    config.use_final_course_override = false;
    DcsPlannerInput input = {
        .position = {-10.0f, 0.0f}, .ground_course_rad = 0.0f,
        .height_above_target_m = 600.0f
    };
    DcsPlannerOutput output;
    /* Nearby/high start still routes to qE, 50 m behind target, no shortening. */
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    check_markers(&output, &config);
    assert(fabs(output.segments[3].geometry.straight.start.north_m + 50.0) < 0.001);
    assert(output.energy_point_distance_m > 40.0f); /* Actual turning connection. */
    assert(fabs(final_course(&output)) < 0.001);
    /* Initial exact-target start with no wind uses current GNSS course. */
    input.position = (DcsPathPoint){0.0f, 0.0f};
    input.ground_course_rad = (float) (PI / 2.0);
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output) - PI / 2.0) < 0.001);
    input.has_wind_estimate = true;
    input.wind_north_mps = 0.0f;
    input.wind_east_mps = 0.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output) - PI / 2.0) < 0.001);
    /* At target, even weak nonzero wind points the approach into wind. */
    input.wind_east_mps = 0.5f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output) + PI / 2.0) < 0.001);
    /* Away from target, weak wind retains position-to-target bearing. */
    input.position.north_m = -10.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output)) < 0.001);
    /* At the threshold wind is strong, regardless of position. */
    input.wind_east_mps = config.weak_wind_threshold_mps;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output) + PI / 2.0) < 0.001);
    /* Override wins over wind and position. */
    config.use_final_course_override = true;
    config.final_course_override_rad = 0.4f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(final_course(&output) - 0.4) < 0.001);
}

static void wind_failures_and_numeric_integration(void)
{
    DcsPlannerConfig config = simulation_config();
    DcsPlannerInput input = {
        .position = {-200.0f, 0.0f}, .ground_course_rad = 0.0f,
        .height_above_target_m = 1000.0f, .has_wind_estimate = true,
        .wind_north_mps = -8.0f
    };
    DcsPlannerOutput output;
    /* Equal headwind gives zero progress; more headwind cannot be flown. */
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NO_FEASIBLE_APPROACH);
    assert(output.failure_reason == DCS_PLAN_FAILURE_WIND_INFEASIBLE);
    input.wind_north_mps = -9.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NO_FEASIBLE_APPROACH);
    assert(output.failure_reason == DCS_PLAN_FAILURE_WIND_INFEASIBLE);
    input.wind_north_mps = 0.0f;
    input.wind_east_mps = 9.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NO_FEASIBLE_APPROACH);
    assert(output.failure_reason == DCS_PLAN_FAILURE_WIND_INFEASIBLE);
    /* Wind >= airspeed is not globally rejected: downwind straight is possible. */
    input.wind_north_mps = 9.0f;
    input.wind_east_mps = 0.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(fabs(output.predicted_travel_time_s - 200.0 / 17.0) < 0.001);
    /* A near-target connection must turn outside the admissible downwind cone. */
    input.position.north_m = -10.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NO_FEASIBLE_APPROACH);
    assert(output.failure_reason == DCS_PLAN_FAILURE_WIND_INFEASIBLE);

    input.position = (DcsPathPoint){-10.0f, 15.0f};
    input.ground_course_rad = -0.8f;
    const float wind_fractions[] = {0.0f, 0.4f, 0.8f, 0.95f, 0.99f, 0.999f};
    for (unsigned int i = 0; i < sizeof(wind_fractions) / sizeof(wind_fractions[0]); ++i) {
        input.wind_north_mps = config.horizontal_airspeed_mps * wind_fractions[i];
        input.wind_east_mps = 0.0f;
        input.height_above_target_m = 10000000.0f;
        assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
        double straight, turning;
        assert(reference_times(output.segments, 4u, input.wind_north_mps, 0.0,
                                config.horizontal_airspeed_mps, &straight, &turning));
        const double height = config.straight_sink_rate_mps *
            (straight + (1.0 + config.turn_loss_fraction) * turning);
        assert(fabs(output.predicted_travel_time_s - straight - turning) /
               (straight + turning) < 0.002);
        assert(fabs(output.predicted_height_loss_m - height) / height < 0.002);
    }
}

static void candidate_ranking(void)
{
    DcsPlannerConfig config = simulation_config();
    DcsPlannerInput input = {
        .position = {15.0f, 45.0f}, .ground_course_rad = -0.9f,
        .has_wind_estimate = true, .wind_north_mps = 1.5f, .wind_east_mps = -0.8f
    };
    const DcsDubinsPose start = {input.position, input.ground_course_rad};
    const DcsDubinsPose goal = {{-50.0f, 0.0f}, 0.0f};
    DcsDubinsPath paths[6];
    double heights[6];
    bool valid[6];
    for (int f = 0; f < 6; ++f) {
        valid[f] = dcs_dubins_generate(&start, &goal, config.turn_radius_m,
                                      (DcsDubinsFamily) f, &paths[f]);
        if (valid[f]) {
            DcsPathSegment segments[4];
            memcpy(segments, paths[f].segments, sizeof(paths[f].segments));
            segments[3] = (DcsPathSegment){.type = DCS_PATH_STRAIGHT,
                .geometry.straight = {.start = goal.position, .end = {0.0f, 0.0f}}};
            double straight, turning;
            assert(reference_times(segments, 4u, input.wind_north_mps, input.wind_east_mps,
                                    config.horizontal_airspeed_mps, &straight, &turning));
            heights[f] = config.straight_sink_rate_mps *
                (straight + (1.0 + config.turn_loss_fraction) * turning);
        }
    }
    unsigned int nominal_winners = 0u, nominal_over_excess = 0u;
    for (int anchor = 0; anchor < 6; ++anchor) {
        if (!valid[anchor]) {
            continue;
        }
        input.height_above_target_m = (float) (heights[anchor] + config.height_reserve_m);
        int best = -1;
        bool best_nominal = false;
        for (int f = 0; f < 6; ++f) {
            if (!valid[f]) {
                continue;
            }
            const double error = input.height_above_target_m - config.height_reserve_m - heights[f];
            const double tolerance = config.height_tolerance_fraction * heights[f];
            if (error < -tolerance) {
                continue;
            }
            const bool nominal = fabs(error) <= tolerance;
            const bool same_class_better = best >= 0 && nominal == best_nominal &&
                (paths[f].absolute_turn_rad < paths[best].absolute_turn_rad - 0.00001f ||
                 (fabsf(paths[f].absolute_turn_rad - paths[best].absolute_turn_rad) <= 0.00001f &&
                  paths[f].length_m < paths[best].length_m - 0.001f));
            if (best < 0 || (nominal && !best_nominal) || same_class_better) {
                best = f;
                best_nominal = nominal;
            }
        }
        assert(best >= 0 && best_nominal);
        DcsPlannerOutput output;
        assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NORMAL_APPROACH);
        ++nominal_winners;
        assert(fabs(output.energy_point_distance_m - paths[best].length_m) < 0.002);
        assert(fabs(output.predicted_height_loss_m - heights[best]) < 0.01);
        for (int f = 0; f < 6; ++f) {
            if (valid[f] && paths[f].absolute_turn_rad < paths[best].absolute_turn_rad - 0.001f &&
                input.height_above_target_m > heights[f] * (1.0 + config.height_tolerance_fraction) +
                                                config.height_reserve_m) {
                ++nominal_over_excess;
            }
        }
    }
    assert(nominal_winners >= 4u);
    assert(nominal_over_excess > 0u); /* Exercises height-class priority over turns. */
}

static void equal_turns_choose_shorter_and_coincident_markers(void)
{
    DcsPlannerConfig config = simulation_config();
    config.final_course_override_rad = (float) PI;
    DcsPlannerInput input = {
        .position = {-100.0f, -10.0f}, .ground_course_rad = 0.0f
    };
    const DcsDubinsPose start = {input.position, input.ground_course_rad};
    const DcsDubinsPose goal = {{50.0f, 0.0f}, (float) PI};
    DcsDubinsPath left, right;
    assert(dcs_dubins_generate(&start, &goal, config.turn_radius_m, DCS_DUBINS_LSL, &left));
    assert(dcs_dubins_generate(&start, &goal, config.turn_radius_m, DCS_DUBINS_RSR, &right));
    assert(fabsf(left.absolute_turn_rad - right.absolute_turn_rad) < 0.00001f);
    assert(right.length_m < left.length_m - 1.0f);
    const double left_loss = config.straight_sink_rate_mps / config.horizontal_airspeed_mps *
        (left.length_m + 50.0 + config.turn_loss_fraction * config.turn_radius_m * left.absolute_turn_rad);
    const double right_loss = config.straight_sink_rate_mps / config.horizontal_airspeed_mps *
        (right.length_m + 50.0 + config.turn_loss_fraction * config.turn_radius_m * right.absolute_turn_rad);
    input.height_above_target_m = (float) ((left_loss + right_loss) / 2.0 + config.height_reserve_m);
    DcsPlannerOutput output;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_NORMAL_APPROACH);
    assert(fabsf(output.energy_point_distance_m - right.length_m) < 0.002f);
    assert(output.segments[0].geometry.arc.sweep_rad > 0.0f);

    /* Already at the genuine energy pose: zero marker is valid, not clamped. */
    config.final_course_override_rad = 0.0f;
    input.position = (DcsPathPoint){-50.0f, 0.0f};
    input.height_above_target_m = 600.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(output.energy_point_distance_m == 0.0f);
    assert(output.final_approach_entry_distance_m == 20.0f);
    assert(fabsf(output.predicted_height_loss_m - 18.75f) < 0.001f);
    /* Zero upstream leg is allowed: qE and qA are genuinely coincident. */
    config.energy_point_upstream_m = 0.0f;
    input.position.north_m = -100.0f;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_EXCESS_HEIGHT);
    assert(output.energy_point_distance_m == output.final_approach_entry_distance_m);
    check_markers(&output, &config);
}

static void invalid_input_preserves_output(void)
{
    DcsPlannerConfig config = simulation_config();
    DcsPlannerInput input = {.position = {-200.0f, 0.0f}, .height_above_target_m = 600.0f};
    DcsPlannerOutput output = {0}, saved;
    output.predicted_travel_time_s = 123.0f;
    memcpy(&saved, &output, sizeof(output));
    assert(dcs_planner_generate(NULL, &config, &output) == DCS_PLAN_INVALID_INPUT);
    assert(dcs_planner_generate(&input, NULL, &output) == DCS_PLAN_INVALID_INPUT);
    assert(dcs_planner_generate(&input, &config, NULL) == DCS_PLAN_INVALID_INPUT);
    const float invalid_values[] = {0.0f, -1.0f, NAN, INFINITY};
    for (unsigned int i = 0; i < sizeof(invalid_values) / sizeof(invalid_values[0]); ++i) {
        DcsPlannerConfig bad = config;
        bad.turn_radius_m = invalid_values[i];
        assert(dcs_planner_generate(&input, &bad, &output) == DCS_PLAN_INVALID_INPUT);
        bad = config;
        bad.horizontal_airspeed_mps = invalid_values[i];
        assert(dcs_planner_generate(&input, &bad, &output) == DCS_PLAN_INVALID_INPUT);
    }
    input.has_wind_estimate = true;
    input.wind_north_mps = NAN;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_INVALID_INPUT);
    input.has_wind_estimate = false;
    input.ground_course_rad = INFINITY;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_INVALID_INPUT);
    input.ground_course_rad = 0.0f;
    config.final_approach_length_m = FLT_MAX;
    config.energy_point_upstream_m = FLT_MAX;
    assert(dcs_planner_generate(&input, &config, &output) == DCS_PLAN_INVALID_INPUT);
    assert(memcmp(&saved, &output, sizeof(output)) == 0);
}

static void post_energy_route_and_public_evaluator(void)
{
    DcsPlannerConfig config = simulation_config();
    config.connect_directly_to_final_entry = true;
    config.use_final_course_override = true;
    config.final_course_override_rad = 0.0f;
    DcsPlannerInput input = {
        .position = {-100.0f, 0.0f},
        .ground_course_rad = 0.0f,
        .height_above_target_m = 100.0f,
    };
    DcsPlannerOutput output;
    const DcsPlannerResult result = dcs_planner_generate(&input, &config, &output);
    assert(result == DCS_PLAN_NORMAL_APPROACH || result == DCS_PLAN_EXCESS_HEIGHT);
    assert(!output.has_energy_point);
    assert(fabsf(output.final_approach_entry_distance_m -
                 output.energy_point_distance_m) < 0.001f);

    DcsPathEvaluation evaluation;
    assert(dcs_planner_evaluate_segments(output.segments, output.segment_count,
        0.0f, 0.0f, &config, &evaluation) == DCS_PATH_EVALUATION_OK);
    assert(fabsf(evaluation.predicted_height_loss_m -
                 output.predicted_height_loss_m) < 0.001f);
}

int main(void)
{
    straight_and_height_cases();
    course_selection_and_near_target();
    wind_failures_and_numeric_integration();
    candidate_ranking();
    equal_turns_choose_shorter_and_coincident_markers();
    post_energy_route_and_public_evaluator();
    invalid_input_preserves_output();
    puts("Planner tests passed.");
    return 0;
}
