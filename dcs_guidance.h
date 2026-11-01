#ifndef DCS_GUIDANCE_H
#define DCS_GUIDANCE_H

#include "dcs_planner.h"

#include <stdbool.h>
#include <stdint.h>

#define DCS_GUIDANCE_MAX_LOITER_SEARCH_STEPS 32u

typedef enum {
    DCS_GUIDANCE_INACTIVE,
    DCS_GUIDANCE_TRACK_TO_ENERGY,
    DCS_GUIDANCE_LOITER,
    DCS_GUIDANCE_TRACK_FINAL,
    DCS_GUIDANCE_DIRECT_TARGET,
    DCS_GUIDANCE_FLARE
} DcsGuidanceMode;

typedef struct {
    DcsPlannerConfig planner;
    float los_lookahead_m;
    float horizontal_error_threshold_m;
    float height_shortage_threshold_m;
    float wind_change_fraction;
    uint32_t horizontal_persistence_ms;
    uint32_t height_persistence_ms;
    uint32_t wind_persistence_ms;
    uint32_t reassessment_min_interval_ms;
    float segment_end_tolerance_m;
    float loiter_radius_m;
    uint8_t loiter_search_steps;
    bool prefer_right_loiter;
    float flare_height_m;
} DcsGuidanceConfig;

typedef struct {
    uint32_t now_ms;
    DcsPathPoint position;
    float ground_v_north_mps;
    float ground_v_east_mps;
    float height_above_target_m;
    float wind_north_mps;
    float wind_east_mps;
    bool horizontal_valid;
    bool horizontal_propagated;
    bool vertical_valid;
    bool has_wind_estimate;
} DcsGuidanceInput;

typedef struct {
    DcsGuidanceMode mode;
    float desired_ground_course_rad;
    float desired_heading_rad;
    bool heading_command_valid;
    bool wind_correction_achievable;
    bool flare_requested;
    bool using_propagated_position;
    float cross_track_error_m;
    uint8_t active_segment;
    bool replan_attempted;
    bool replan_succeeded;
    bool target_passed;
} DcsGuidanceOutput;

typedef struct {
    DcsPlannerOutput active_plan;
    DcsGuidanceMode mode;
    uint8_t active_segment;
    float completed_distance_m;
    float segment_progress_m;
    float previous_arc_angle_rad;
    bool has_previous_arc_angle;
    bool energy_handled;
    bool final_entry_handled;
    bool target_passed;
    bool target_reassessment_done;
    DcsPathSegment loiter_segment;
    float loiter_progress_rad;
    float previous_loiter_angle_rad;
    bool has_previous_loiter_angle;
    float last_valid_heading_rad;
    bool has_last_valid_heading;
    float assessed_wind_north_mps;
    float assessed_wind_east_mps;
    bool assessed_has_wind;
    uint32_t horizontal_since_ms;
    uint32_t height_since_ms;
    uint32_t wind_since_ms;
    bool horizontal_timer_active;
    bool height_timer_active;
    bool wind_timer_active;
    uint32_t last_reassessment_ms;
    bool has_reassessment_time;
    bool initialized;
} DcsGuidanceState;

bool dcs_guidance_start(DcsGuidanceState *state,
                        const DcsGuidanceConfig *config,
                        const DcsGuidanceInput *input);

void dcs_guidance_update(DcsGuidanceState *state,
                         const DcsGuidanceConfig *config,
                         const DcsGuidanceInput *input,
                         DcsGuidanceOutput *output);

#endif
