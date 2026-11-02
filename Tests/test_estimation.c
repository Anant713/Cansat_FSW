#include "App.h"
#include "Console.h"
#include "main.h"
#include "apps/Cansat.h"
#include "apps/Estimation.h"
#include "apps/SensorAcquisition.h"

#include <math.h>
#include <string.h>

#define ESTIMATION_REPORT_PERIOD_MS 1000U
#define ESTIMATION_SAMPLE_MAX_AGE_MS 250U

Cansat estimation_test_cansat;
volatile bool estimation_test_sensor_init_ok;
volatile bool estimation_test_calibration_ok;
volatile bool estimation_test_init_ok;
volatile bool estimation_test_bmi_fresh;
volatile bool estimation_test_mag_fresh;
volatile bool estimation_test_quaternion_ok;
volatile uint32_t estimation_test_updates;
volatile uint32_t estimation_test_reports;

static uint32_t last_acquisition;
static uint32_t last_report;
static bool stream = true;
static char command[CONSOLE_LINE_MAX + 1U];

static bool sample_fresh(bool valid, uint32_t timestamp_ms, uint32_t now)
{
    return valid && (uint32_t)(now - timestamp_ms) <= ESTIMATION_SAMPLE_MAX_AGE_MS;
}

static void report(void)
{
    const uint32_t now = HAL_GetTick();
    const float *q = estimation_test_cansat.attitude.quaternion;
    const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] +
                             q[2] * q[2] + q[3] * q[3]);
    estimation_test_bmi_fresh = sample_fresh(estimation_test_cansat.bmi.valid,
                                             estimation_test_cansat.bmi.timestamp_ms, now);
    estimation_test_mag_fresh = sample_fresh(estimation_test_cansat.mag.valid,
                                             estimation_test_cansat.mag.timestamp_ms, now);
    estimation_test_quaternion_ok = estimation_test_cansat.attitude.quaternionValid &&
        isfinite(norm) && fabsf(norm - 1.0f) <= 0.02f;

    Console_Printf("Estimation init sensors/calibration/filter=%u/%u/%u updates=%lu reports=%lu\r\n",
        (unsigned)estimation_test_sensor_init_ok,
        (unsigned)estimation_test_calibration_ok,
        (unsigned)estimation_test_init_ok,
        (unsigned long)estimation_test_updates,
        (unsigned long)estimation_test_reports);
    Console_Printf("Inputs BMI/MAG fresh=%u/%u ages=%lu/%lu ms\r\n",
        (unsigned)estimation_test_bmi_fresh, (unsigned)estimation_test_mag_fresh,
        (unsigned long)(now - estimation_test_cansat.bmi.timestamp_ms),
        (unsigned long)(now - estimation_test_cansat.mag.timestamp_ms));
    Console_Printf("Quaternion valid/unit=%u/%u q=%.5f,%.5f,%.5f,%.5f norm=%.5f\r\n",
        (unsigned)estimation_test_cansat.attitude.quaternionValid,
        (unsigned)estimation_test_quaternion_ok,
        (double)q[0], (double)q[1], (double)q[2], (double)q[3], (double)norm);
    Console_Printf("Heading yaw=%.3f deg yaw-rate=%.3f deg/s angular-valid=%u\r\n",
        (double)estimation_test_cansat.attitude.yaw,
        (double)estimation_test_cansat.attitude.yawRate,
        (unsigned)estimation_test_cansat.attitude.angularVelocityValid);
}

void App_Init(void)
{
    (void)Console_Init();
    Cansat_Init(&estimation_test_cansat);
    estimation_test_sensor_init_ok = SensorAcquisition_Init();
    estimation_test_calibration_ok =
        SensorAcquisition_CalibrateAtStartup(&estimation_test_cansat);
    estimation_test_init_ok = Estimation_Init();
    last_report = HAL_GetTick();
    last_acquisition = last_report - ACQUISITION_PERIOD_MS;
    Console_Write("DCS attitude estimation test: keep the board stationary initially.\r\n");
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
        SensorAcquisition_Update(&estimation_test_cansat);
        Estimation_Update(&estimation_test_cansat);
        ++estimation_test_updates;
    }
    now = HAL_GetTick();
    if (requested || (stream &&
        (uint32_t)(now - last_report) >= ESTIMATION_REPORT_PERIOD_MS)) {
        last_report = now;
        ++estimation_test_reports;
        report();
    }
}
