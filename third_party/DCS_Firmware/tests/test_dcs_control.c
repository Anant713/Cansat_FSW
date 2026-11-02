#include "../dcs_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    const DcsHeadingControllerConfig config = {
        .kp = 1.0f,
        .kd = 0.5f,
        .max_command = 0.8f,
        .command_rate_per_s = 2.0f,
        .max_step_s = 0.2f,
    };
    DcsHeadingControllerState state;
    assert(dcs_heading_controller_init(&state, &config));
    assert(near(dcs_heading_controller_update(&state, &config, 1000u,
        0.5f, 0.0f, 0.2f, true), 0.4f, 1.0e-5f));
    assert(near(dcs_heading_controller_update(&state, &config, 1100u,
        2.0f, 0.0f, 0.0f, true), 0.6f, 1.0e-5f));
    assert(near(dcs_heading_controller_update(&state, &config, 1200u,
        2.0f, 0.0f, 0.0f, false), 0.0f, 1.0e-5f));

    DcsBrakeDemand brakes = dcs_allocate_single_sided_brakes(-0.3f);
    assert(near(brakes.left, 0.3f, 1.0e-5f) && brakes.right == 0.0f);
    brakes = dcs_allocate_single_sided_brakes(0.4f);
    assert(brakes.left == 0.0f && near(brakes.right, 0.4f, 1.0e-5f));
    brakes = dcs_allocate_flare(0.7f);
    assert(near(brakes.left, 0.7f, 1.0e-5f) && near(brakes.right, 0.7f, 1.0e-5f));

    const DcsServoMapConfig left = {1000u, 1500u, 900u, 2100u};
    const DcsServoMapConfig right = {2000u, 1400u, 900u, 2100u};
    DcsServoPulseOutput pulse;
    assert(dcs_servo_map_pair(&left, &right, (DcsBrakeDemand){0.5f, 0.25f}, &pulse));
    assert(pulse.left_pulse_us == 1250u);
    assert(pulse.right_pulse_us == 1850u);

    puts("dcs_control tests passed");
    return 0;
}
