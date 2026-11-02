#ifndef DCS_PLANNER_H
#define DCS_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

/* Positions use the target-centred North/East frame in metres.
 * Angles are radians, clockwise from North. Height is positive above target.
 */
#define DCS_PLANNER_MAX_SEGMENTS 4u

typedef struct {
    float north_m;
    float east_m;
} DcsPathPoint;

typedef enum {
    DCS_PATH_STRAIGHT,
    DCS_PATH_ARC
} DcsPathSegmentType;

typedef struct {
    DcsPathSegmentType type;
    union {
        struct {
            DcsPathPoint start;
            DcsPathPoint end;
        } straight;
        struct {
            DcsPathPoint centre;
            float radius_m;
            float start_angle_rad;
            /* Positive is clockwise/right; negative is left. */
            float sweep_rad;
        } arc;
    } geometry;
} DcsPathSegment;

typedef struct {
    float turn_radius_m;
    float horizontal_airspeed_mps;
    float straight_sink_rate_mps;
    float turn_loss_fraction;
    float final_approach_length_m;
    float energy_point_upstream_m;
    float height_reserve_m;
    /* Band = fraction * each candidate's predicted loss, excluding reserve. */
    float height_tolerance_fraction;
    float weak_wind_threshold_mps;
    bool use_final_course_override;
    float final_course_override_rad;
    /* False for initial route via qE; true after the energy decision/loiter. */
    bool connect_directly_to_final_entry;
} DcsPlannerConfig;

typedef struct {
    DcsPathPoint position;
    float ground_course_rad; /* GNSS velocity direction, not payload heading. */
    float height_above_target_m;
    float wind_north_mps;
    float wind_east_mps;
    /* False only before an estimate exists; accepted wind does not expire. */
    bool has_wind_estimate;
} DcsPlannerInput;

typedef enum {
    DCS_PLAN_NORMAL_APPROACH,
    DCS_PLAN_EXCESS_HEIGHT,
    DCS_PLAN_NO_FEASIBLE_APPROACH,
    DCS_PLAN_INVALID_INPUT
} DcsPlannerResult;

typedef enum {
    DCS_PLAN_FAILURE_NONE,
    DCS_PLAN_FAILURE_INSUFFICIENT_HEIGHT,
    DCS_PLAN_FAILURE_WIND_INFEASIBLE
} DcsPlannerFailureReason;

typedef struct {
    DcsPathSegment segments[DCS_PLANNER_MAX_SEGMENTS];
    uint8_t segment_count;
    /* Path distance from its beginning, not another segment. */
    float energy_point_distance_m;
    /* qA lies within the combined qE-to-target straight, not another segment. */
    float final_approach_entry_distance_m;
    float predicted_travel_time_s;
    /* Includes turn allowance; reserve is accounted for separately. */
    float predicted_height_loss_m;
    bool has_energy_point;
    float selected_final_course_rad;
    /* Relevant when the result is NO_FEASIBLE_APPROACH. */
    DcsPlannerFailureReason failure_reason;
} DcsPlannerOutput;

typedef enum {
    DCS_PATH_EVALUATION_OK,
    DCS_PATH_EVALUATION_WIND_INFEASIBLE,
    DCS_PATH_EVALUATION_INVALID
} DcsPathEvaluationResult;

typedef struct {
    float travel_time_s;
    float predicted_height_loss_m;
} DcsPathEvaluation;

/* Evaluate already-constructed geometry with the same wind/turn-loss model. */
DcsPathEvaluationResult dcs_planner_evaluate_segments(
    const DcsPathSegment *segments, uint8_t segment_count,
    float wind_north_mps, float wind_east_mps,
    const DcsPlannerConfig *config, DcsPathEvaluation *evaluation);

/*
 * Generate an initial or replacement route from the supplied current state.
 * Bounded work and caller-owned storage; no allocation, drivers or scheduler.
 * Caller supplies usable navigation and a coherent input snapshot. No GNSS or
 * attitude validity policy is implemented here. Missing wind uses the explicit
 * zero-wind planning assumption, not a measurement of calm conditions. The
 * planner selects final course internally: manual override, otherwise into
 * strong wind, otherwise position-to-target bearing. At zero displacement use
 * into-wind course for any nonzero estimate, otherwise usable GNSS ground
 * course. Connect to the energy point, arriving along the final course, then
 * continue straight through final entry to target.
 *
 * Only NORMAL_APPROACH / EXCESS_HEIGHT outputs are usable normal routes.
 * Other results do not authorize replacing the active route. The manager owns
 * reassessment, fallback, overrun and flare decisions. Use separate candidate
 * storage so an unsuccessful planning attempt cannot overwrite the active path.
 * INVALID_INPUT leaves output unchanged. NO_FEASIBLE_APPROACH clears the
 * candidate output and sets failure_reason: INSUFFICIENT_HEIGHT if any wind-
 * traversable candidate exists, otherwise WIND_INFEASIBLE. Geometry/height
 * arithmetic outside finite float representation returns INVALID_INPUT.
 * Arc integration and equality settings are frozen compile-time flight values;
 * physical planner values are supplied through the caller-owned configuration.
 */
DcsPlannerResult dcs_planner_generate(const DcsPlannerInput *input,
                                     const DcsPlannerConfig *config,
                                     DcsPlannerOutput *output);

#endif
