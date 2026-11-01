#ifndef DCS_CONFIG_H
#define DCS_CONFIG_H

/*
 * Frozen configuration for the November 2026 flight. These values were
 * previously simulation placeholders and are being accepted as flight values
 * by programme decision. They are assumptions, not experimentally identified
 * vehicle parameters. Keep the complete set with the flight log.
 */

/* Target and navigation freshness. */
#define DCS_FLIGHT_TARGET_LAT_E7                         267175350
#define DCS_FLIGHT_TARGET_LON_E7                         843015430
#define DCS_FLIGHT_MAX_GNSS_AGE_MS                            1500u
#define DCS_FLIGHT_MAX_BARO_AGE_MS                             500u

/* Attitude: launch with body +X true North, +Y East and +Z down. */
#define DCS_FLIGHT_INITIAL_ROLL_RAD                          0.0f
#define DCS_FLIGHT_INITIAL_PITCH_RAD                         0.0f
#define DCS_FLIGHT_INITIAL_HEADING_RAD                       0.0f
#define DCS_FLIGHT_IMU_SAMPLE_RATE_HZ                      100.0f
#define DCS_FLIGHT_GYRO_RANGE_DEG_S                       2000.0f
#define DCS_FLIGHT_AHRS_GAIN                                 0.5f
#define DCS_FLIGHT_ACCEL_REJECTION_DEG                      10.0f
#define DCS_FLIGHT_MAG_REJECTION_DEG                        10.0f
#define DCS_FLIGHT_AHRS_RECOVERY_TIMEOUT_S                   5.0f
#define DCS_FLIGHT_MIN_ACCEL_G                               0.8f
#define DCS_FLIGHT_MAX_ACCEL_G                               1.2f
#define DCS_FLIGHT_CALIBRATED_MAG_STRENGTH                  25.25f
#define DCS_FLIGHT_MAX_MAG_RELATIVE_ERROR                    0.25f
/* WMMHR-2025, 2026-11-02, 26.717535 N, 84.301543 E, 60 m GPS:
 * +0.4720 degrees east. True heading = magnetic heading + declination.
 */
#define DCS_FLIGHT_MAGNETIC_DECLINATION_RAD           0.0082379541f
#define DCS_FLIGHT_ATTITUDE_MAX_STEP_S                       0.05f
#define DCS_FLIGHT_MAX_HEADING_REFERENCE_AGE_MS              5000u

/* Installed sensor geometry. Coordinates are measured from the physical-Body
 * origin, interpreted as the cylinder axis, in the drawing's +X/+Y directions.
 * Z is not known and is not needed for the implemented rotation about Body Z.
 */
#define DCS_FLIGHT_BMI_BODY_X_M                           0.007157f
#define DCS_FLIGHT_BMI_BODY_Y_M                           0.040885f

/* Planner and guidance. */
#define DCS_FLIGHT_PLAN_TURN_RADIUS_M                       20.0f
#define DCS_FLIGHT_HORIZONTAL_AIRSPEED_MPS                   8.0f
#define DCS_FLIGHT_STRAIGHT_SINK_RATE_MPS                    3.0f
#define DCS_FLIGHT_FINAL_APPROACH_LENGTH_M                  30.0f
#define DCS_FLIGHT_ENERGY_POINT_UPSTREAM_M                  20.0f
#define DCS_FLIGHT_LOS_LOOKAHEAD_M                          10.0f
#define DCS_FLIGHT_HORIZONTAL_ERROR_M                       10.0f
#define DCS_FLIGHT_HEIGHT_SHORTAGE_M                         5.0f
#define DCS_FLIGHT_WEAK_WIND_THRESHOLD_MPS                   1.0f
#define DCS_FLIGHT_HEIGHT_RESERVE_M                         15.0f
#define DCS_FLIGHT_HEIGHT_CLASS_TOLERANCE_FRACTION           0.02f
#define DCS_FLIGHT_TURN_LOSS_FRACTION                        0.20f
#define DCS_FLIGHT_WIND_CHANGE_FRACTION                      0.30f
#define DCS_FLIGHT_HORIZONTAL_PERSISTENCE_MS                 1000u
#define DCS_FLIGHT_HEIGHT_PERSISTENCE_MS                     1000u
#define DCS_FLIGHT_WIND_PERSISTENCE_MS                       1000u
#define DCS_FLIGHT_REASSESSMENT_MIN_INTERVAL_MS              2000u
#define DCS_FLIGHT_SEGMENT_END_TOLERANCE_M                   0.5f
#define DCS_FLIGHT_LOITER_RADIUS_M                          25.0f
#define DCS_FLIGHT_LOITER_SEARCH_STEPS                         16u
#define DCS_FLIGHT_PREFER_RIGHT_LOITER                          1

/* Deterministic numerical settings used by the flight planner. */
#define DCS_FLIGHT_ARC_TIME_INTERVALS                          32u
#define DCS_FLIGHT_TURN_EQUALITY_RAD                      0.00001f
#define DCS_FLIGHT_LENGTH_EQUALITY_M                       0.001f

/* In-flight wind/trim identification. */
#define DCS_FLIGHT_CANOPY_SETTLING_MS                         5000u
#define DCS_FLIGHT_IDENTIFICATION_COMMAND                     0.35f
#define DCS_FLIGHT_IDENTIFICATION_TIMEOUT_MS                 30000u
#define DCS_FLIGHT_IDENTIFICATION_MIN_HEIGHT_M               150.0f
#define DCS_FLIGHT_TRIM_SETTLING_MS                            3000u
#define DCS_FLIGHT_TRIM_SAMPLE_COUNT                            20u
#define DCS_FLIGHT_TRIM_TIMEOUT_MS                            10000u
#define DCS_FLIGHT_CIRCLE_MIN_SAMPLES                            12u
#define DCS_FLIGHT_CIRCLE_MAX_SAMPLES                            64u
#define DCS_FLIGHT_CIRCLE_MIN_COVERAGE_RAD                     4.5f
#define DCS_FLIGHT_CIRCLE_MIN_SPREAD_MPS                       1.0f
#define DCS_FLIGHT_CIRCLE_MAX_RMS_FRACTION                     0.10f
#define DCS_FLIGHT_CIRCLE_MIN_AIRSPEED_MPS                     1.0f
#define DCS_FLIGHT_CIRCLE_MAX_AIRSPEED_MPS                    15.0f

/* Straight-flight wind estimator. */
#define DCS_FLIGHT_WIND_EMA_ALPHA                              0.20f
#define DCS_FLIGHT_WIND_MAX_HEADING_RATE_RAD_S                 0.20f
#define DCS_FLIGHT_WIND_MAX_BRAKE_COMMAND                      0.10f
#define DCS_FLIGHT_WIND_MAX_COURSE_RATE_RAD_S                  0.20f
#define DCS_FLIGHT_WIND_MIN_GROUND_SPEED_MPS                   0.20f
#define DCS_FLIGHT_WIND_MAX_HEADING_AGE_MS                      500u
#define DCS_FLIGHT_WIND_MAX_FIX_INTERVAL_MS                    1000u

/* Heading controller, flare and installed MG996R pulse assumptions. */
#define DCS_FLIGHT_CONTROLLER_KP                                1.0f
#define DCS_FLIGHT_CONTROLLER_KD                                0.2f
#define DCS_FLIGHT_CONTROLLER_MAX_COMMAND                       0.7f
#define DCS_FLIGHT_COMMAND_RATE_PER_S                           1.0f
#define DCS_FLIGHT_CONTROLLER_MAX_STEP_S                        0.2f
#define DCS_FLIGHT_FLARE_HEIGHT_M                               5.0f
#define DCS_FLIGHT_FLARE_BRAKE_DEMAND                           0.8f
#define DCS_FLIGHT_MAX_STEERING_ANGLE_DEG                      45.0f
#define DCS_FLIGHT_LEFT_SERVO_NEUTRAL_US                         500u
#define DCS_FLIGHT_LEFT_SERVO_MAX_PULL_US                       1000u
#define DCS_FLIGHT_LEFT_SERVO_ABSOLUTE_MIN_US                    500u
#define DCS_FLIGHT_LEFT_SERVO_ABSOLUTE_MAX_US                   2500u
#define DCS_FLIGHT_RIGHT_SERVO_NEUTRAL_US                        500u
#define DCS_FLIGHT_RIGHT_SERVO_MAX_PULL_US                      1000u
#define DCS_FLIGHT_RIGHT_SERVO_ABSOLUTE_MIN_US                   500u
#define DCS_FLIGHT_RIGHT_SERVO_ABSOLUTE_MAX_US                  2500u

#endif
