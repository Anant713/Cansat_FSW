#include "App.h"
#include "Console.h"
#include "main.h"
#include "drivers/BMP.h"
#include "drivers/AHT20.h"
#include "drivers/bmi270.h"
#include "drivers/IIS2MDC.h"
#include <string.h>

/* Flash this separate test application to check actual wiring and data.
 * Change the two selectable addresses to match your board's address pins. */
#define TEST_BMP_ADDRESS BMP280_ADDRESS_LOW
#define TEST_BMI_ADDRESS BMI270_ADDRESS_LOW
extern I2C_HandleTypeDef hi2c2;

BMP280 sensor_bmp;
AHT20 sensor_aht;
BMI270 sensor_bmi;
IIS2MDC sensor_mag;
volatile BMP280Data sensor_bmp_data;
volatile AHT20Data sensor_aht_data;
volatile BMI270Data sensor_bmi_data;
volatile IIS2MDCData sensor_mag_data;
volatile bool sensor_init_ok[4]; /* BMP, AHT, BMI, IIS */
volatile SensorStatus sensor_status[4];
volatile uint32_t sensor_sample_count[4];
/* Set true in debugger only after placing the board at rest. */
volatile bool sensor_calibrate_gyro;
volatile BMI270CalibrationState sensor_gyro_calibration_state;
BMI270Calibration sensor_saved_imu;

/* Replace these identity values with measured calibration parameters.
 * These are examples of storing parameters in code, not measured calibration. */
static const BMI270Calibration saved_imu = {
    .accel = SENSOR_CALIBRATION_3D_IDENTITY,
    .gyro_bias_dps = {0, 0, 0}
};
static const SensorCalibration3D saved_mag = SENSOR_CALIBRATION_3D_IDENTITY;
static uint32_t last_poll, last_humidity_start;
static uint32_t last_report;
static bool stream = true;
static char command[CONSOLE_LINE_MAX + 1U];

static void report(void)
{
    Console_Printf("Init BMP/AHT/BMI/MAG=%u/%u/%u/%u status=%u/%u/%u/%u gyro-cal=%u\r\n",
        (unsigned)sensor_init_ok[0], (unsigned)sensor_init_ok[1],
        (unsigned)sensor_init_ok[2], (unsigned)sensor_init_ok[3],
        (unsigned)sensor_status[0], (unsigned)sensor_status[1],
        (unsigned)sensor_status[2], (unsigned)sensor_status[3], (unsigned)sensor_gyro_calibration_state);
    if (sensor_bmp_data.valid)
        Console_Printf("BMP280 T=%.2f C P=%.2f Pa sample_ms=%lu count=%lu\r\n",
            (double)sensor_bmp_data.temperature, (double)sensor_bmp_data.pressure,
            (unsigned long)sensor_bmp_data.timestamp_ms, (unsigned long)sensor_sample_count[0]);
    else Console_Write("BMP280: no valid sample\r\n");
    if (sensor_aht_data.valid)
        Console_Printf("AHT20 humidity=%.2f %%RH sample_ms=%lu count=%lu\r\n",
            (double)sensor_aht_data.humidity, (unsigned long)sensor_aht_data.timestamp_ms,
            (unsigned long)sensor_sample_count[1]);
    else Console_Write("AHT20: no valid sample\r\n");
    if (sensor_bmi_data.valid)
        Console_Printf("BMI270 accel XYZ=%.3f,%.3f,%.3f m/s^2 gyro XYZ=%.3f,%.3f,%.3f deg/s sample_ms=%lu count=%lu\r\n",
            (double)sensor_bmi_data.accelX, (double)sensor_bmi_data.accelY, (double)sensor_bmi_data.accelZ,
            (double)sensor_bmi_data.gyroX, (double)sensor_bmi_data.gyroY, (double)sensor_bmi_data.gyroZ,
            (unsigned long)sensor_bmi_data.timestamp_ms, (unsigned long)sensor_sample_count[2]);
    else Console_Write("BMI270: no valid sample\r\n");
    if (sensor_mag_data.valid)
        Console_Printf("IIS2MDC mag XYZ=%.3f,%.3f,%.3f uT overrun=%u sample_ms=%lu count=%lu\r\n",
            (double)sensor_mag_data.magX, (double)sensor_mag_data.magY, (double)sensor_mag_data.magZ,
            (unsigned)sensor_mag_data.overrun, (unsigned long)sensor_mag_data.timestamp_ms,
            (unsigned long)sensor_sample_count[3]);
    else Console_Write("IIS2MDC: no valid sample\r\n");
}

void App_Init(void)
{
    (void)Console_Init();
    Console_Write("Sensor test initializing...\r\n");
    /* Independent attempts: a missing device does not hide results from others. */
    sensor_init_ok[0] = BMP280_Init(&sensor_bmp, &hi2c2, TEST_BMP_ADDRESS);
    sensor_init_ok[1] = AHT20_Init(&sensor_aht, &hi2c2);
    sensor_init_ok[2] = BMI270_Init(&sensor_bmi, &hi2c2, TEST_BMI_ADDRESS);
    sensor_init_ok[3] = IIS2MDC_Init(&sensor_mag, &hi2c2);
    if (sensor_init_ok[2]) sensor_init_ok[2] = BMI270_SetCalibration(&sensor_bmi, &saved_imu);
    if (sensor_init_ok[3]) sensor_init_ok[3] = IIS2MDC_SetCalibration(&sensor_mag, &saved_mag);
    sensor_status[0] = sensor_bmp.io.status;
    sensor_status[1] = sensor_aht.io.status;
    sensor_status[2] = sensor_bmi.io.status;
    sensor_status[3] = sensor_mag.io.status;
    last_poll = HAL_GetTick();
    last_humidity_start = last_poll-2000U; /* first conversion on the first pass */
    Console_Write("Commands: status, stream on, stream off, cal gyro (keep board stationary), help.\r\n");
    report();
}

void App_RunOnce(void)
{
    Console_Process();
    bool requested = false;
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (!strcmp(command, "status")) requested = true;
        else if (!strcmp(command, "stream on")) { stream = true; Console_Write("Streaming on.\r\n"); }
        else if (!strcmp(command, "stream off")) { stream = false; Console_Write("Streaming off.\r\n"); }
        else if (!strcmp(command, "cal gyro")) sensor_calibrate_gyro = true;
        else Console_Write("Commands: status, stream on, stream off, cal gyro (stationary), help.\r\n");
    } else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    uint32_t now = HAL_GetTick();
    if (sensor_calibrate_gyro) {
        sensor_calibrate_gyro = false;
        bool ok = sensor_init_ok[2] && BMI270_StartGyroCalibration(&sensor_bmi, 64, 120000);
        Console_Write(ok ? "Gyro calibration started; keep stationary.\r\n" : "Gyro calibration refused.\r\n");
    }
    if (sensor_init_ok[1]) {
        /* Consume a pending result before triggering the next conversion. */
        if (sensor_aht.state != AHT20_IDLE) {
            if (AHT20_IsReady(&sensor_aht)) {
                AHT20Data sample;
                bool ok = AHT20_Read(&sensor_aht, &sample);
                sensor_aht_data = sample;
                if (ok) ++sensor_sample_count[1];
            } else if (sensor_aht.io.status != SENSOR_NOT_READY) {
                sensor_aht_data.valid = false;
            }
            sensor_status[1] = sensor_aht.io.status;
        }
        /* >1 second recommended by AHT20 datasheet to limit self-heating. */
        if (sensor_aht.state == AHT20_IDLE && (uint32_t)(now-last_humidity_start) >= 2000U) {
            last_humidity_start = now;
            if (!AHT20_StartMeasurement(&sensor_aht)) sensor_aht_data.valid = false;
            sensor_status[1] = sensor_aht.io.status;
        }
    }
    if ((uint32_t)(now-last_poll) < 200U) goto reporting;
    last_poll = now;
    if (sensor_init_ok[0]) {
        BMP280Data sample;
        bool ok = BMP280_Read(&sensor_bmp, &sample);
        sensor_bmp_data = sample;
        sensor_status[0] = sensor_bmp.io.status;
        if (ok) ++sensor_sample_count[0];
    }
    if (sensor_init_ok[2]) {
        BMI270Data sample;
        bool ok = BMI270_Read(&sensor_bmi, &sample);
        sensor_bmi_data = sample;
        sensor_status[2] = sensor_bmi.io.status;
        sensor_gyro_calibration_state = sensor_bmi.calibration_state;
        if (ok) ++sensor_sample_count[2];
        if (sensor_gyro_calibration_state == BMI270_CAL_DONE)
            (void)BMI270_GetCalibration(&sensor_bmi, &sensor_saved_imu);
    }
    if (sensor_init_ok[3]) {
        IIS2MDCData sample;
        bool ok = IIS2MDC_Read(&sensor_mag, &sample);
        sensor_mag_data = sample;
        sensor_status[3] = sensor_mag.io.status;
        if (ok) ++sensor_sample_count[3];
    }
reporting:
    if (requested || (stream && (uint32_t)(now-last_report) >= 1000U)) {
        last_report = now;
        report();
    }
}
