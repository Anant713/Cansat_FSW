#include "App.h"
#include "main.h"
#include "drivers/Switches.h"
#include "drivers/Servo.h"
#include "apps/Cansat.h"
#include "apps/SensorAcquisition.h"
#include "apps/Communication.h"
#include "apps/Logging.h"
#include "apps/StateMachine.h"
#include "apps/BarometricAltitude.h"
#include "apps/Estimation.h"
#include "apps/Control.h"
#include "apps/CameraControl.h"

/* Available in the debugger; later operations will consume this shared data. */
Cansat flight_cansat;
bool flight_sensor_init_ok;
bool flight_communication_init_ok;
bool flight_logging_init_ok;
bool flight_state_init_ok;
bool flight_calibration_init_ok;
static uint32_t lastAcquisition, lastTelemetry, lastLog;

/* Acquisition, persistent mission gates, communication and logging. */
void App_Init(void)
{
    /* Generated main has already configured GPIO and TIM4. */
    if (!Switches_Init() || !Servo_Init()) Error_Handler();
    Switches_LED(true);
    Servo_SetAngles(SERVO_MIN_ANGLE_DEG, SERVO_MIN_ANGLE_DEG);
    Servo_Enable(); /* Hold startup position throughout calibration and flight. */
    Cansat_Init(&flight_cansat);
    flight_cansat.actuators.servoValid = Servo_GetData(&flight_cansat.actuators.servo);
    Switches_Update(&flight_cansat.switches);
    if (!flight_cansat.actuators.servoValid || !flight_cansat.actuators.servo.enabled)
        Error_Handler();
    CameraControl_Init(&flight_cansat.camera);
    Switches_Update(&flight_cansat.switches);
    /* Mount before UART reception starts: SD startup can block for some time. */
    flight_logging_init_ok = Logging_Init();
    flight_sensor_init_ok = SensorAcquisition_Init();
    flight_calibration_init_ok = SensorAcquisition_CalibrateAtStartup(&flight_cansat);
    flight_communication_init_ok = Communication_Init();
    StateMachine_SetCameraReady(flight_calibration_init_ok ||
        SensorAcquisition_GetGroundPressure() > 0.0f);
    flight_state_init_ok = StateMachine_Init(&flight_cansat);
    if (!Estimation_Init() || !Control_Init()) Error_Handler();
    lastTelemetry = lastLog = HAL_GetTick();
    lastAcquisition = lastTelemetry - ACQUISITION_PERIOD_MS;
    /* A failed sensor does not prevent acquisition from the others. */
}

void App_RunOnce(void)
{
    uint32_t now = HAL_GetTick();
    flight_cansat.telemetry.uptimeMs = now;
    if ((uint32_t)(now - lastAcquisition) >= ACQUISITION_PERIOD_MS) {
        lastAcquisition = now;
        SensorAcquisition_Update(&flight_cansat);
    }
    Communication_Update(&flight_cansat);
    /* Attitude estimation runs after calibrated acquisition in every mission
     * state. Auxiliary navigation altitude is updated separately;
     * state-machine height gates read the BMP driver's relative height. */
    BarometricAltitude_Update(&flight_cansat);
    Estimation_Update(&flight_cansat);
    StateMachine_Update(&flight_cansat);
    Control_Update(&flight_cansat);
    /* Refresh after acquisition; both due operations use the same Cansat data.
     * Send first so a successful packet count is reflected in the log row. */
    now = HAL_GetTick();
    flight_cansat.telemetry.uptimeMs = now;
    if ((uint32_t)(now - lastTelemetry) >= COMMUNICATION_TELEMETRY_PERIOD_MS) {
        lastTelemetry = now;
        if (flight_cansat.telemetry.enabled)
            (void)Communication_SendTelemetry(&flight_cansat);
    }
    if ((uint32_t)(now - lastLog) >= LOGGING_PERIOD_MS) {
        lastLog = now;
        if (Logging_IsReady()) (void)Logging_Write(&flight_cansat);
    }
    /* Cooperative processing sequence: acquisition -> command processing ->
     * barometric navigation -> estimation -> state machine -> control ->
     * telemetry -> logging.
     * Acquisition and SD writes are synchronous and can delay these nominal
     * schedules. No busy wait or catch-up burst is used.
     */
}
