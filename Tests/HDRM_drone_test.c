/* Timings are milliseconds from completion of App_Init (not power-on). */
#ifndef HDRM_FIRE
#define HDRM_FIRE 10000U
#endif
#ifndef HDRM_ON
#define HDRM_ON 5000U
#endif
 
/* Reuse the complete acquisition/logging/telecommand harness. CMake compiles
 * this entry point alone, not test_telecommands.c as a second source. */
#define App_Init HDRM_BaseInit
#define App_RunOnce HDRM_BaseRunOnce
#include "test_telecommands.c"
#undef App_Init
#undef App_RunOnce
#include "drivers/Switches.h"

_Static_assert(HDRM_FIRE <= UINT32_MAX && HDRM_ON > 0U && HDRM_ON <= UINT32_MAX,
               "Invalid HDRM timing (milliseconds)");

typedef enum { HDRM_WAITING, HDRM_ACTIVE, HDRM_COMPLETE, HDRM_DISABLED } HDRMTestPhase;
HDRMTestPhase hdrm_drone_phase;
uint32_t hdrm_drone_started_ms, hdrm_drone_fired_ms, hdrm_drone_off_ms;

static void serviceHDRM(void)
{
    uint32_t now = HAL_GetTick();
    if (hdrm_drone_phase == HDRM_WAITING &&
        (uint32_t)(now - hdrm_drone_started_ms) >= HDRM_FIRE) {
        Switches_HDRM(true);
        hdrm_drone_fired_ms = now;
        hdrm_drone_phase = HDRM_ACTIVE;
        Switches_Update(&test_telecommands_data.switches);
        Console_Printf("HDRM ON at %lu ms; pulse=%lu ms.\r\n",
                       (unsigned long)now, (unsigned long)HDRM_ON);
    } else if (hdrm_drone_phase == HDRM_ACTIVE &&
               (uint32_t)(now - hdrm_drone_fired_ms) >= HDRM_ON) {
        Switches_HDRM(false);
        hdrm_drone_off_ms = now;
        hdrm_drone_phase = HDRM_COMPLETE;
        test_telecommands_data.actuators.parafoilDeployed = true;
        Switches_Update(&test_telecommands_data.switches);
        Console_Printf("HDRM OFF at %lu ms; elapsed ON=%lu ms. One-shot complete.\r\n",
            (unsigned long)now, (unsigned long)(now - hdrm_drone_fired_ms));
    }
}

void App_Init(void)
{
    bool switchesOK = Switches_Init(); /* Drive HDRM low before SD/sensor startup. */
    HDRM_BaseInit();
    hdrm_drone_phase = switchesOK ? HDRM_WAITING : HDRM_DISABLED;
    hdrm_drone_fired_ms = hdrm_drone_off_ms = 0;
    test_telecommands_data.actuators.parafoilDeployed = false;
    Switches_Update(&test_telecommands_data.switches);
    if (switchesOK)
        Console_Printf("HDRM timer armed: fire after %lu ms, ON for %lu ms. Reboot restarts timer.\r\n",
                       (unsigned long)HDRM_FIRE, (unsigned long)HDRM_ON);
    else Console_Write("HDRM disabled: switch initialization failed.\r\n");
    hdrm_drone_started_ms = HAL_GetTick();
}

void App_RunOnce(void)
{
    serviceHDRM();
    HDRM_BaseRunOnce();
    serviceHDRM(); /* Check again after synchronous sensor/SD/command work. */
}
