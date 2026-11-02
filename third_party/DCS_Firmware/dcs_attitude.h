#ifndef DCS_ATTITUDE_H
#define DCS_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>
#include "third_party/Fusion/Fusion/FusionAhrs.h"

typedef struct {
    float x;
    float y;
    float z;
} DcsVec3;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} DcsQuaternion;

typedef struct {
    /* Known initial body-to-NED pose: roll about X, pitch about Y, heading. */
    float initial_roll_rad;
    float initial_pitch_rad;
    float initial_heading_rad;
    float sample_rate_hz;
    float gyro_range_deg_s;
    float gain;
    float accel_rejection_deg;
    float mag_rejection_deg;
    float recovery_timeout_s;
    float min_accel_g;
    float max_accel_g;
    float calibrated_mag_strength;
    float max_mag_relative_error;
    float magnetic_declination_rad;
    float max_step_s;
    /* Age of the last heading reference: initial known pose or accepted mag. */
    uint32_t max_heading_reference_age_ms;
} DcsAttitudeConfig;

typedef struct {
    uint32_t system_time_ms;
    DcsVec3 gyro_rad_s;
    DcsVec3 accel_mps2;
    DcsVec3 mag_calibrated;
    bool gyro_valid;
    bool accel_valid;
    bool mag_valid;
    bool accel_correction_allowed;
    bool mag_correction_allowed;
} DcsAttitudeSample;

/*
 * Calibrated values in the installed sensors' native axes. Magnetometer
 * offset/matrix calibration must already have been applied in IIS2MDC axes.
 * The BMI acceleration is always transferred to the physical-Body origin,
 * which is defined as the cylinder axis. body_alpha_z uses the payload/Body
 * +Z sign. Supply zero to retain centripetal correction while deliberately
 * omitting tangential correction.
 */
typedef struct {
    uint32_t system_time_ms;
    DcsVec3 gyro_bmi_rad_s;
    DcsVec3 accel_bmi_mps2;
    DcsVec3 mag_iis2mdc_calibrated;
    bool gyro_valid;
    bool accel_valid;
    bool mag_valid;
    bool accel_correction_allowed;
    bool mag_correction_allowed;
    float body_alpha_z_rad_s2;
} DcsInstalledAttitudeSample;

typedef struct {
    /* Persistent private engine state. Uses magnetic NED internally. */
    FusionAhrs fusion;
    DcsQuaternion body_to_ned;
    float payload_heading_rad;
    float payload_heading_rate_rad_s;
    bool orientation_valid;
    bool heading_valid;
    bool accel_correction_used;
    bool mag_correction_used;
    bool accel_reference_available;
    bool mag_reference_available;
    float accel_error_deg;
    float mag_error_deg;
    bool accel_recovery;
    bool mag_recovery;
    bool gyro_overrange_recovery;
    bool fusion_startup;
    uint32_t last_sample_ms;
    uint32_t last_heading_reference_ms;
    bool initialized;
} DcsAttitudeState;

/*
 * Flight software owns sensor calibration and supplies calibrated vectors
 * and their validity. This interface requires the right-handed payload body
 * frame: +X parafoil front, +Y right, +Z down. The installed BMI270 and
 * IIS2MDC mappings are implemented by dcs_sensor_transform; this module itself
 * performs no sensor-to-payload conversion. Units are rad/s for
 * gyro, m/s^2 for accelerometer specific force (including gravity at rest),
 * and the same units as calibrated_mag_strength for magnetometer.
 * The caller may disable accelerometer correction during deployment, turns,
 * or strong payload motion even when the acceleration magnitude is near 1 g.
 * Magnetometer correction may likewise be disabled during known interference.
 * The quaternion rotates body vectors into North/East/Down coordinates.
 * This estimates PAYLOAD orientation, not canopy orientation.
 * The gyro timestamp identifies a NEW IMU sample, not a super-loop iteration.
 * Accel accompanies that IMU sample. The caller supplies the latest sufficiently
 * fresh mag vector and clears mag_valid when it becomes stale; there is no
 * independent mag-age check in this interface. Keep config unchanged after init.
 * Compile wrapper and Fusion with FUSION_USE_NORMAL_SQRT (see build instructions).
 */
/*
 * Initialize once from the known pose at system_time_ms. Run continuously
 * through all flight phases; secondary DCS activation does not reinitialize it.
 * A repeated update timestamp returns false and leaves the state unchanged.
 * An excessive time gap invalidates the estimate and requires explicit init
 * with a known CURRENT pose; never restore the launch-pad pose during flight.
 * Validity means the estimate is available under the existing timestamp and
 * heading-reference policy; it is not an accuracy bound. Inspect recovery and
 * overrange flags too. Heading retains its last value when geometrically invalid.
 */
bool dcs_attitude_init(DcsAttitudeState *state,
                       const DcsAttitudeConfig *config,
                       uint32_t system_time_ms);
bool dcs_attitude_update(DcsAttitudeState *state,
                         const DcsAttitudeConfig *config,
                         const DcsAttitudeSample *sample);

/* Installed-hardware entry point; maps native axes before running Fusion. */
bool dcs_attitude_update_installed(
    DcsAttitudeState *state,
    const DcsAttitudeConfig *config,
    const DcsInstalledAttitudeSample *sample);

#endif
