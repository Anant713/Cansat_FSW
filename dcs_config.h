#ifndef DCS_CONFIG_H
#define DCS_CONFIG_H

/*
 * Common configuration for future planning/guidance simulation tests.
 * SIM_DUMMY values below are arbitrary placeholders, NOT measured vehicle
 * properties, tuned settings, or flight-approved defaults. Replace them using
 * test evidence before flight use. No planner/guidance implementation currently
 * consumes these constants; existing modules still take caller-owned configs.
 */
#define DCS_CONFIG_SIMULATION_ONLY 1

/* DUMMY: simulation placeholders only. Units are explicit in each name. */
#define DCS_SIM_DUMMY_PLAN_TURN_RADIUS_M             20.0f
#define DCS_SIM_DUMMY_HORIZONTAL_AIRSPEED_MPS         8.0f
#define DCS_SIM_DUMMY_STRAIGHT_SINK_RATE_MPS          3.0f
#define DCS_SIM_DUMMY_FINAL_APPROACH_LENGTH_M       30.0f
#define DCS_SIM_DUMMY_ENERGY_POINT_UPSTREAM_M       20.0f
#define DCS_SIM_DUMMY_LOS_LOOKAHEAD_M               10.0f
#define DCS_SIM_DUMMY_HORIZONTAL_ERROR_X_M          10.0f
#define DCS_SIM_DUMMY_HEIGHT_SHORTAGE_Y_M            5.0f
#define DCS_SIM_DUMMY_WEAK_WIND_THRESHOLD_MPS        1.0f

/* DUMMY: exact values/percentage basis remain unapproved within discussed ranges. */
#define DCS_SIM_DUMMY_HEIGHT_RESERVE_M              15.0f
#define DCS_SIM_DUMMY_HEIGHT_CLASS_TOLERANCE_FRACTION 0.02f

/* Agreed initial test settings; still require physical/replay validation. */
#define DCS_TEST_TURN_LOSS_FRACTION                   0.20f
#define DCS_TEST_WIND_CHANGE_FRACTION                 0.30f
#define DCS_TEST_HORIZONTAL_PERSISTENCE_MS            1000u
#define DCS_TEST_HEIGHT_PERSISTENCE_MS                1000u
#define DCS_TEST_WIND_PERSISTENCE_MS                  1000u
#define DCS_TEST_REASSESSMENT_MIN_INTERVAL_MS         2000u

#endif
