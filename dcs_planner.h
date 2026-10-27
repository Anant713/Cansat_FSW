#ifndef DCS_PLANNER_H
#define DCS_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

/* Interface only: dcs_planner_generate is not implemented yet.
 * Positions use the target-centred North/East frame in metres.
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
    /* Absolute classification band; conversion from percentage is pending. */
    float height_tolerance_m;
    float weak_wind_threshold_mps;
    bool use_final_course_override;
    float final_course_override_rad;
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
    float predicted_travel_time_s;
    /* Includes turn allowance; reserve is accounted for separately. */
    float predicted_height_loss_m;
    /* Relevant when the result is NO_FEASIBLE_APPROACH. */
    DcsPlannerFailureReason failure_reason;
} DcsPlannerOutput;

/*
 * Generate an initial or replacement route from the supplied current state.
 * Bounded work and caller-owned storage; no allocation, drivers or scheduler.
 * Caller supplies usable navigation and a coherent input snapshot. No GNSS or
 * attitude validity policy is implemented here. Missing wind uses the explicit
 * zero-wind planning assumption, not a measurement of calm conditions. The
 * planner selects final course internally: manual override, otherwise into
 * strong wind, otherwise position-to-target bearing. Exact-target fallback
 * without an override or strong wind remains an unresolved terminal case.
 *
 * Only NORMAL_APPROACH / EXCESS_HEIGHT outputs are usable normal routes.
 * Other results do not authorize replacing the active route. The manager owns
 * reassessment, fallback, overrun and flare decisions. Use separate candidate
 * storage so an unsuccessful planning attempt cannot overwrite the active path.
 */
DcsPlannerResult dcs_planner_generate(const DcsPlannerInput *input,
                                     const DcsPlannerConfig *config,
                                     DcsPlannerOutput *output);

#endif
