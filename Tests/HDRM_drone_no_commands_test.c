/* All times in milliseconds; countdown begins after startup calibration succeeds. */
#ifndef HDRM_FIRE
#define HDRM_FIRE 300000U
#endif
#ifndef HDRM_ON
#define HDRM_ON 10000U
#endif
#define TEST_WITH_TELEMETRY 1
#define TEST_PROCESS_TELECOMMANDS 0
#define App_Init Drone_BaseInit
#define App_RunOnce Drone_BaseRunOnce
#include "test_acquisition_logging.c"
#undef App_Init
#undef App_RunOnce
#include "drivers/Switches.h"
#include "drivers/MissionStore.h"
#include "drivers/Servo.h"

_Static_assert(HDRM_FIRE <= UINT32_MAX && HDRM_ON > 0U && HDRM_ON <= UINT32_MAX,
               "Invalid HDRM timing in milliseconds");
typedef enum { DRONE_WAITING, DRONE_ACTIVE, DRONE_COMPLETE, DRONE_DISABLED } DronePhase;
DronePhase drone_phase;
bool drone_servos_ok;
uint32_t drone_countdown_ms, drone_fired_ms, drone_off_ms;

static void droneService(void)
{
    uint32_t now = HAL_GetTick();
    if (stopped && drone_phase != DRONE_DISABLED) {
        Switches_HDRM(false);
        drone_phase = DRONE_DISABLED;
        Console_Write("HDRM cancelled by console stop.\r\n");
    }
    if (drone_phase == DRONE_WAITING && (uint32_t)(now - drone_countdown_ms) >= HDRM_FIRE) {
        Switches_HDRM(true);
        drone_fired_ms = now;
        drone_phase = DRONE_ACTIVE;
        Console_Printf("HDRM ON at %lu ms.\r\n", (unsigned long)now);
    } else if (drone_phase == DRONE_ACTIVE && (uint32_t)(now - drone_fired_ms) >= HDRM_ON) {
        Switches_HDRM(false);
        drone_off_ms = now;
        drone_phase = DRONE_COMPLETE;
        test_acquisition_logging_data.actuators.parafoilDeployed = true;
        Console_Printf("HDRM OFF at %lu ms; actual ON=%lu ms.\r\n", (unsigned long)now,
                       (unsigned long)(now - drone_fired_ms));
    }
    Switches_Update(&test_acquisition_logging_data.switches);
}

void App_Init(void)
{
    bool switchesOK = Switches_Init();
    /* Shared startup enables the LED and holds both servos at zero. */
    Drone_BaseInit();
    /* Base initialization clears Cansat, so copy the command snapshot afterward. */
    test_acquisition_logging_data.actuators.servoValid =
        Servo_GetData(&test_acquisition_logging_data.actuators.servo);
    drone_servos_ok = test_acquisition_logging_data.actuators.servoValid &&
        test_acquisition_logging_data.actuators.servo.enabled;
    Console_Write(drone_servos_ok ? "Servos enabled: both holding 5 degrees.\r\n" :
                                  "Servo initialization/PWM enable FAILED.\r\n");
    drone_phase = DRONE_DISABLED;
    Console_Write("Drone test: telemetry always enabled; incoming XBee commands discarded.\r\n");
    /* BaseInit has already completed calibration/recovery and flash persistence. */
    if (switchesOK && test_calibration_init_ok) {
        drone_phase = DRONE_WAITING;
        Console_Printf("Calibration ready. HDRM countdown=%lu ms, pulse=%lu ms.\r\n",
                       (unsigned long)HDRM_FIRE, (unsigned long)HDRM_ON);
        drone_countdown_ms = HAL_GetTick();
    } else Console_Write("Startup calibration/setup FAILED; HDRM disabled.\r\n");
    /* Exclude blocking startup calibration from recurring-task schedules. */
    lastLog = lastTelemetry = lastReport = HAL_GetTick();
    lastAcquisition = lastReport - ACQUISITION_PERIOD_MS;
}

void App_RunOnce(void)
{
    droneService();
    Drone_BaseRunOnce();
    droneService();
}
