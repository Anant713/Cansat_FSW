#include "apps/Control.h"
#include "apps/EstimationDcs.h"
#include "drivers/Servo.h"
#include "main.h"

#include <math.h>

#define RAD_TO_DEG 57.295779513082320876f

static DcsRuntimeState runtime;
static bool initialized, started;

static float pulse_to_angle(uint16_t pulse)
{
    return ((float)pulse - 500.0f) * 180.0f / 2000.0f;
}

bool Control_Init(void)
{
    initialized = Estimation_GetFlightConfig() != NULL;
    started = false;
    return initialized;
}

void Control_Update(Cansat *c)
{
    if (!initialized || c == NULL) return;
    const DcsFlightConfig *config = Estimation_GetFlightConfig();
    if (!started && c->missionState == CANSAT_STATE_DESCENT_2 &&
        c->actuators.parafoilDeployed) {
        started = dcs_runtime_start(&runtime, &config->runtime, HAL_GetTick());
    }
    if (!started || (c->missionState != CANSAT_STATE_DESCENT_2 &&
                     c->missionState != CANSAT_STATE_TOUCHDOWN)) return;
    DcsRuntimeInput input;
    DcsRuntimeOutput output;
    if (!Estimation_MakeRuntimeInput(c, &input)) return;
    dcs_runtime_update(&runtime, &config->runtime, &input, &output);
    if (output.servo_output_valid) {
        Servo_SetAngles(pulse_to_angle(output.servo.left_pulse_us),
                        pulse_to_angle(output.servo.right_pulse_us));
        c->actuators.servoValid = Servo_GetData(&c->actuators.servo);
    }
    c->navigation.headingErrorValid = output.guidance.heading_command_valid &&
        input.heading_valid;
    if (c->navigation.headingErrorValid) {
        float error = output.guidance.desired_heading_rad - input.heading_rad;
        error = atan2f(sinf(error), cosf(error));
        c->navigation.headingError = error * RAD_TO_DEG;
    }
}

bool DCSGuidance_Init(void) { return Control_Init(); }
void DCSGuidance_Update(Cansat *c) { Control_Update(c); }
bool DCSController_Init(void) { return Control_Init(); }
void DCSController_Update(Cansat *c) { Control_Update(c); }
