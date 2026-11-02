#include "dcs_flight_config.h"

#include "dcs_config.h"

#include <stddef.h>

void dcs_flight_config_load(DcsFlightConfig *config)
{
    if (config == NULL) {
        return;
    }

    *config = (DcsFlightConfig){
        .navigation = {
            .target_lat_e7 = DCS_FLIGHT_TARGET_LAT_E7,
            .target_lon_e7 = DCS_FLIGHT_TARGET_LON_E7,
            .max_gnss_age_ms = DCS_FLIGHT_MAX_GNSS_AGE_MS,
            .max_baro_age_ms = DCS_FLIGHT_MAX_BARO_AGE_MS,
        },
        .attitude = {
            .initial_roll_rad = DCS_FLIGHT_INITIAL_ROLL_RAD,
            .initial_pitch_rad = DCS_FLIGHT_INITIAL_PITCH_RAD,
            .initial_heading_rad = DCS_FLIGHT_INITIAL_HEADING_RAD,
            .sample_rate_hz = DCS_FLIGHT_IMU_SAMPLE_RATE_HZ,
            .gyro_range_deg_s = DCS_FLIGHT_GYRO_RANGE_DEG_S,
            .gain = DCS_FLIGHT_AHRS_GAIN,
            .accel_rejection_deg = DCS_FLIGHT_ACCEL_REJECTION_DEG,
            .mag_rejection_deg = DCS_FLIGHT_MAG_REJECTION_DEG,
            .recovery_timeout_s = DCS_FLIGHT_AHRS_RECOVERY_TIMEOUT_S,
            .min_accel_g = DCS_FLIGHT_MIN_ACCEL_G,
            .max_accel_g = DCS_FLIGHT_MAX_ACCEL_G,
            .calibrated_mag_strength = DCS_FLIGHT_CALIBRATED_MAG_STRENGTH,
            .max_mag_relative_error = DCS_FLIGHT_MAX_MAG_RELATIVE_ERROR,
            .magnetic_declination_rad = DCS_FLIGHT_MAGNETIC_DECLINATION_RAD,
            .max_step_s = DCS_FLIGHT_ATTITUDE_MAX_STEP_S,
            .max_heading_reference_age_ms =
                DCS_FLIGHT_MAX_HEADING_REFERENCE_AGE_MS,
        },
        .runtime = {
            .canopy_settling_ms = DCS_FLIGHT_CANOPY_SETTLING_MS,
            .identification_steering_command =
                DCS_FLIGHT_IDENTIFICATION_COMMAND,
            .identification_timeout_ms =
                DCS_FLIGHT_IDENTIFICATION_TIMEOUT_MS,
            .minimum_identification_height_m =
                DCS_FLIGHT_IDENTIFICATION_MIN_HEIGHT_M,
            .trim_settling_ms = DCS_FLIGHT_TRIM_SETTLING_MS,
            .trim_sample_count = DCS_FLIGHT_TRIM_SAMPLE_COUNT,
            .trim_timeout_ms = DCS_FLIGHT_TRIM_TIMEOUT_MS,
            .fallback_horizontal_airspeed_mps =
                DCS_FLIGHT_HORIZONTAL_AIRSPEED_MPS,
            .fallback_has_wind = false,
            .fallback_wind_north_mps = 0.0f,
            .fallback_wind_east_mps = 0.0f,
            .flare_brake_demand = DCS_FLIGHT_FLARE_BRAKE_DEMAND,
            .circle_fit = {
                .minimum_samples = DCS_FLIGHT_CIRCLE_MIN_SAMPLES,
                .maximum_samples = DCS_FLIGHT_CIRCLE_MAX_SAMPLES,
                .minimum_direction_coverage_rad =
                    DCS_FLIGHT_CIRCLE_MIN_COVERAGE_RAD,
                .minimum_velocity_spread_mps =
                    DCS_FLIGHT_CIRCLE_MIN_SPREAD_MPS,
                .maximum_rms_fraction =
                    DCS_FLIGHT_CIRCLE_MAX_RMS_FRACTION,
                .minimum_airspeed_mps =
                    DCS_FLIGHT_CIRCLE_MIN_AIRSPEED_MPS,
                .maximum_airspeed_mps =
                    DCS_FLIGHT_CIRCLE_MAX_AIRSPEED_MPS,
                .minimum_turn_radius_m =
                    DCS_FLIGHT_CIRCLE_MIN_TURN_RADIUS_M,
                .maximum_turn_radius_m =
                    DCS_FLIGHT_CIRCLE_MAX_TURN_RADIUS_M,
                .minimum_turn_direction_consistency =
                    DCS_FLIGHT_CIRCLE_MIN_DIRECTION_CONSISTENCY,
            },
            .wind = {
                .horizontal_airspeed_mps =
                    DCS_FLIGHT_HORIZONTAL_AIRSPEED_MPS,
                .ema_alpha = DCS_FLIGHT_WIND_EMA_ALPHA,
                .max_heading_rate_rad_s =
                    DCS_FLIGHT_WIND_MAX_HEADING_RATE_RAD_S,
                .max_brake_command = DCS_FLIGHT_WIND_MAX_BRAKE_COMMAND,
                .max_course_rate_rad_s =
                    DCS_FLIGHT_WIND_MAX_COURSE_RATE_RAD_S,
                .min_ground_speed_mps =
                    DCS_FLIGHT_WIND_MIN_GROUND_SPEED_MPS,
                .max_heading_age_ms = DCS_FLIGHT_WIND_MAX_HEADING_AGE_MS,
                .max_fix_interval_ms =
                    DCS_FLIGHT_WIND_MAX_FIX_INTERVAL_MS,
            },
            .guidance = {
                .planner = {
                    .turn_radius_m = DCS_FLIGHT_PLAN_TURN_RADIUS_M,
                    .horizontal_airspeed_mps =
                        DCS_FLIGHT_HORIZONTAL_AIRSPEED_MPS,
                    .straight_sink_rate_mps =
                        DCS_FLIGHT_STRAIGHT_SINK_RATE_MPS,
                    .turn_loss_fraction = DCS_FLIGHT_TURN_LOSS_FRACTION,
                    .final_approach_length_m =
                        DCS_FLIGHT_FINAL_APPROACH_LENGTH_M,
                    .energy_point_upstream_m =
                        DCS_FLIGHT_ENERGY_POINT_UPSTREAM_M,
                    .height_reserve_m = DCS_FLIGHT_HEIGHT_RESERVE_M,
                    .height_tolerance_fraction =
                        DCS_FLIGHT_HEIGHT_CLASS_TOLERANCE_FRACTION,
                    .weak_wind_threshold_mps =
                        DCS_FLIGHT_WEAK_WIND_THRESHOLD_MPS,
                },
                .los_lookahead_m = DCS_FLIGHT_LOS_LOOKAHEAD_M,
                .horizontal_error_threshold_m =
                    DCS_FLIGHT_HORIZONTAL_ERROR_M,
                .height_shortage_threshold_m =
                    DCS_FLIGHT_HEIGHT_SHORTAGE_M,
                .wind_change_fraction = DCS_FLIGHT_WIND_CHANGE_FRACTION,
                .horizontal_persistence_ms =
                    DCS_FLIGHT_HORIZONTAL_PERSISTENCE_MS,
                .height_persistence_ms = DCS_FLIGHT_HEIGHT_PERSISTENCE_MS,
                .wind_persistence_ms = DCS_FLIGHT_WIND_PERSISTENCE_MS,
                .reassessment_min_interval_ms =
                    DCS_FLIGHT_REASSESSMENT_MIN_INTERVAL_MS,
                .segment_end_tolerance_m =
                    DCS_FLIGHT_SEGMENT_END_TOLERANCE_M,
                .loiter_radius_m = DCS_FLIGHT_LOITER_RADIUS_M,
                .loiter_search_steps = DCS_FLIGHT_LOITER_SEARCH_STEPS,
                .prefer_right_loiter = DCS_FLIGHT_PREFER_RIGHT_LOITER != 0,
                .flare_height_m = DCS_FLIGHT_FLARE_HEIGHT_M,
            },
            .controller = {
                .kp = DCS_FLIGHT_CONTROLLER_KP,
                .kd = DCS_FLIGHT_CONTROLLER_KD,
                .max_command = DCS_FLIGHT_CONTROLLER_MAX_COMMAND,
                .command_rate_per_s = DCS_FLIGHT_COMMAND_RATE_PER_S,
                .max_step_s = DCS_FLIGHT_CONTROLLER_MAX_STEP_S,
            },
            .left_servo = {
                DCS_FLIGHT_LEFT_SERVO_NEUTRAL_US,
                DCS_FLIGHT_LEFT_SERVO_MAX_PULL_US,
                DCS_FLIGHT_LEFT_SERVO_ABSOLUTE_MIN_US,
                DCS_FLIGHT_LEFT_SERVO_ABSOLUTE_MAX_US,
            },
            .right_servo = {
                DCS_FLIGHT_RIGHT_SERVO_NEUTRAL_US,
                DCS_FLIGHT_RIGHT_SERVO_MAX_PULL_US,
                DCS_FLIGHT_RIGHT_SERVO_ABSOLUTE_MIN_US,
                DCS_FLIGHT_RIGHT_SERVO_ABSOLUTE_MAX_US,
            },
        },
    };
}
