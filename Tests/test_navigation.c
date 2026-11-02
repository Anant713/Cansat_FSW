#include "App.h"
#include "Console.h"
#include "main.h"
#include "apps/BarometricAltitude.h"
#include "apps/Cansat.h"
#include "apps/EstimationDcs.h"
#include "apps/SensorAcquisition.h"

#include <string.h>

#define NAVIGATION_REPORT_PERIOD_MS 1000U

Cansat navigation_test_cansat;
volatile DcsRuntimeInput navigation_test_output;
volatile bool navigation_test_sensor_init_ok;
volatile bool navigation_test_calibration_ok;
volatile bool navigation_test_init_ok;
volatile bool navigation_test_output_ok;
volatile uint32_t navigation_test_updates;
volatile uint32_t navigation_test_reports;

static uint32_t last_acquisition;
static uint32_t last_report;
static bool stream = true;
static char command[CONSOLE_LINE_MAX + 1U];

static void report(void)
{
    DcsRuntimeInput output;
    navigation_test_output_ok =
        Estimation_MakeRuntimeInput(&navigation_test_cansat, &output);
    if (navigation_test_output_ok) navigation_test_output = output;

    Console_Printf("Navigation init sensors/calibration/filter=%u/%u/%u updates=%lu reports=%lu\r\n",
        (unsigned)navigation_test_sensor_init_ok,
        (unsigned)navigation_test_calibration_ok,
        (unsigned)navigation_test_init_ok,
        (unsigned long)navigation_test_updates,
        (unsigned long)navigation_test_reports);
    Console_Printf("GNSS valid/fix/speed/course=%u/%u/%u/%u lat=%.7f lon=%.7f\r\n",
        (unsigned)navigation_test_cansat.gnss.valid,
        (unsigned)navigation_test_cansat.gnss.fixValid,
        (unsigned)navigation_test_cansat.gnss.speedValid,
        (unsigned)navigation_test_cansat.gnss.courseValid,
        navigation_test_cansat.gnss.latitude,
        navigation_test_cansat.gnss.longitude);
    Console_Printf("Barometer altitude valid=%u height=%.3f m sample_ms=%lu\r\n",
        (unsigned)navigation_test_cansat.navigation.altitudeValid,
        (double)navigation_test_cansat.navigation.altitude,
        (unsigned long)navigation_test_cansat.navigation.altitudeTimestampMs);
    if (!navigation_test_output_ok) {
        Console_Write("DCS navigation output unavailable.\r\n");
        return;
    }
    Console_Printf("Position N/E=%.3f/%.3f m target-distance=%.3f m valid=%u propagated=%u\r\n",
        (double)output.navigation.position.north_m,
        (double)output.navigation.position.east_m,
        (double)navigation_test_cansat.navigation.targetDistance,
        (unsigned)output.navigation.horizontal_valid,
        (unsigned)output.navigation.horizontal_propagated);
    Console_Printf("Velocity N/E=%.3f/%.3f m/s height=%.3f m vertical-valid=%u fresh-velocity=%u\r\n",
        (double)output.navigation.ground_v_north_mps,
        (double)output.navigation.ground_v_east_mps,
        (double)output.navigation.height_above_target_m,
        (unsigned)output.navigation.vertical_valid,
        (unsigned)output.has_fresh_gnss_velocity);
}

void App_Init(void)
{
    (void)Console_Init();
    Cansat_Init(&navigation_test_cansat);
    navigation_test_sensor_init_ok = SensorAcquisition_Init();
    navigation_test_calibration_ok =
        SensorAcquisition_CalibrateAtStartup(&navigation_test_cansat);
    navigation_test_init_ok = Estimation_Init();
    last_report = HAL_GetTick();
    last_acquisition = last_report - ACQUISITION_PERIOD_MS;
    Console_Write("DCS navigation test: provide GNSS sky view and a valid saved ground calibration.\r\n");
    Console_Write("Commands: status, stream on, stream off, help.\r\n");
    report();
}

void App_RunOnce(void)
{
    Console_Process();
    bool requested = false;
    const ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) requested = true;
        else if (!strcmp(command, "stream on")) {
            stream = true;
            Console_Write("Streaming on.\r\n");
        } else if (!strcmp(command, "stream off")) {
            stream = false;
            Console_Write("Streaming off.\r\n");
        } else {
            Console_Write("Commands: status, stream on, stream off, help.\r\n");
        }
    } else if (input < 0) {
        Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_acquisition) >= ACQUISITION_PERIOD_MS) {
        last_acquisition = now;
        SensorAcquisition_Update(&navigation_test_cansat);
        BarometricAltitude_Update(&navigation_test_cansat);
        Estimation_Update(&navigation_test_cansat);
        ++navigation_test_updates;
    }
    now = HAL_GetTick();
    if (requested || (stream &&
        (uint32_t)(now - last_report) >= NAVIGATION_REPORT_PERIOD_MS)) {
        last_report = now;
        ++navigation_test_reports;
        report();
    }
}
