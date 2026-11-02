#include "dcs_sensor_transform.h"

#include "dcs_config.h"

#define COS_30_DEG 0.8660254037844386f
#define COS_60_DEG 0.5f

DcsVec3 dcs_bmi_vector_to_payload(DcsVec3 calibrated_bmi)
{
    return (DcsVec3){
        COS_60_DEG * calibrated_bmi.x - COS_30_DEG * calibrated_bmi.y,
        -COS_30_DEG * calibrated_bmi.x - COS_60_DEG * calibrated_bmi.y,
        -calibrated_bmi.z,
    };
}

DcsVec3 dcs_mag_vector_to_payload(DcsVec3 calibrated_mag)
{
    return (DcsVec3){
        COS_30_DEG * calibrated_mag.x - COS_60_DEG * calibrated_mag.y,
        COS_60_DEG * calibrated_mag.x + COS_30_DEG * calibrated_mag.y,
        -calibrated_mag.z,
    };
}

DcsVec3 dcs_bmi_position_in_payload_m(void)
{
    /* Payload +X is +15 degrees clockwise from physical Body +X. */
    const float cos_15 = 0.9659258262890683f;
    const float sin_15 = 0.2588190451025207f;
    return (DcsVec3){
        cos_15 * DCS_FLIGHT_BMI_BODY_X_M +
            sin_15 * DCS_FLIGHT_BMI_BODY_Y_M,
        -sin_15 * DCS_FLIGHT_BMI_BODY_X_M +
            cos_15 * DCS_FLIGHT_BMI_BODY_Y_M,
        0.0f,
    };
}

DcsVec3 dcs_bmi_accel_to_body_origin_payload(
    DcsVec3 calibrated_bmi_accel_mps2,
    float body_omega_z_rad_s,
    float body_alpha_z_rad_s2)
{
    const DcsVec3 sensor =
        dcs_bmi_vector_to_payload(calibrated_bmi_accel_mps2);
    const DcsVec3 r = dcs_bmi_position_in_payload_m();
    const float omega_squared = body_omega_z_rad_s * body_omega_z_rad_s;

    /* a_sensor = a_origin + alpha x r + omega x (omega x r). */
    const DcsVec3 rotational = {
        -body_alpha_z_rad_s2 * r.y - omega_squared * r.x,
        body_alpha_z_rad_s2 * r.x - omega_squared * r.y,
        0.0f,
    };
    return (DcsVec3){
        sensor.x - rotational.x,
        sensor.y - rotational.y,
        sensor.z,
    };
}
