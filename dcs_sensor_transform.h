#ifndef DCS_SENSOR_TRANSFORM_H
#define DCS_SENSOR_TRANSFORM_H

#include "dcs_attitude.h"

/*
 * Fixed installed-axis mappings into the right-handed payload frame:
 * +X parafoil front, +Y right, +Z into the top-view page/down.
 *
 * Apply sensor calibration in each sensor's native coordinate system before
 * calling these functions. In particular, the supplied IIS2MDC offset and
 * mapping matrix are defined in its left-handed native coordinate system.
 */
DcsVec3 dcs_bmi_vector_to_payload(DcsVec3 calibrated_bmi);
DcsVec3 dcs_mag_vector_to_payload(DcsVec3 calibrated_mag);

/* BMI location relative to the physical-Body origin, expressed in payload axes. */
DcsVec3 dcs_bmi_position_in_payload_m(void);

/*
 * Transfer calibrated accelerometer specific force from the BMI location to
 * the physical-Body origin, assuming rigid rotation only about physical Body Z.
 * body_omega_z_rad_s and body_alpha_z_rad_s2 are positive about Body +Z, which
 * is also payload +Z. Gravity cancels in the point-to-point translation.
 *
 * This corrects both tangential alpha x r and centripetal omega x (omega x r)
 * acceleration. Pass alpha = 0 only when angular-rate change is negligible.
 * It does not model pendulum motion or an unknown canopy-system rotation point.
 */
DcsVec3 dcs_bmi_accel_to_body_origin_payload(
    DcsVec3 calibrated_bmi_accel_mps2,
    float body_omega_z_rad_s,
    float body_alpha_z_rad_s2);

#endif
