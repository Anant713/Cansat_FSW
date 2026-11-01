#include "../dcs_sensor_transform.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool near(float actual, float expected)
{
    return fabsf(actual - expected) < 1.0e-5f;
}

int main(void)
{
    DcsVec3 value;

    value = dcs_bmi_vector_to_payload((DcsVec3){1.0f, 0.0f, 0.0f});
    assert(near(value.x, 0.5f));
    assert(near(value.y, -0.8660254f));
    assert(near(value.z, 0.0f));
    value = dcs_bmi_vector_to_payload((DcsVec3){0.0f, 0.0f, 1.0f});
    assert(near(value.z, -1.0f));

    value = dcs_mag_vector_to_payload((DcsVec3){1.0f, 0.0f, 0.0f});
    assert(near(value.x, 0.8660254f));
    assert(near(value.y, 0.5f));
    value = dcs_mag_vector_to_payload((DcsVec3){0.0f, 0.0f, 1.0f});
    assert(near(value.z, -1.0f));

    const DcsVec3 r = dcs_bmi_position_in_payload_m();
    assert(near(r.x, 0.0174949f));
    assert(near(r.y, 0.0376395f));

    /* Build the BMI measurement caused only by rotation about the origin. */
    const float omega = 4.0f;
    const float alpha = 2.0f;
    const DcsVec3 rotational_payload = {
        -alpha * r.y - omega * omega * r.x,
        alpha * r.x - omega * omega * r.y,
        0.0f,
    };
    /* BMI mapping is orthogonal and symmetric, so its inverse is itself. */
    const DcsVec3 rotational_bmi =
        dcs_bmi_vector_to_payload(rotational_payload);
    value = dcs_bmi_accel_to_body_origin_payload(
        rotational_bmi, omega, alpha);
    assert(near(value.x, 0.0f));
    assert(near(value.y, 0.0f));
    assert(near(value.z, 0.0f));

    puts("dcs_sensor_transform tests passed");
    return 0;
}
