#include "apps/SensorAcquisition.h"
#include "apps/Cansat.h"
#include "drivers/CalibrationStore.h"
#include <string.h>

/* Match the address pins and the receiver's current baud rate on the board. */
#define ACQUISITION_BMP_ADDRESS BMP280_ADDRESS_HIGH // 
#define ACQUISITION_BMI_ADDRESS BMI270_ADDRESS_LOW
#define ACQUISITION_GNSS_BAUD GNSS_EXPECTED_BAUD
#define HUMIDITY_PERIOD_MS 2000U

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart6;

static BMP280 bmp;
static AHT20 aht;
static BMI270 bmi;
static IIS2MDC mag;
static bool gnssInitialized;
static bool batteryInitialized;
static uint32_t lastHumidityStart;
static SensorCalibrationStatus calibrationStatus;
static uint32_t calibrationStarted;
static SensorCalibrationRecord savedCalibration;
static bool calibrationValid;

static bool restoreCalibration(void)
{
    bool bmpOK = false, bmiOK = false;
    if (bmp.initialized) {
        if (calibrationValid) bmpOK = BMP280_SetGroundPressure(&bmp, savedCalibration.groundPressurePa);
        else { bmp.groundPressureValid = false; bmp.groundPressurePa = 0.0f; }
    }
    if (bmi.initialized) {
        BMI270Calibration correction;
        if (BMI270_GetCalibration(&bmi, &correction)) {
            if (calibrationValid)
                memcpy(correction.gyro_bias_dps, savedCalibration.gyroBiasDps, sizeof(correction.gyro_bias_dps));
            else memset(correction.gyro_bias_dps, 0, sizeof(correction.gyro_bias_dps));
            bmiOK = BMI270_SetCalibration(&bmi, &correction);
        }
    }
    return bmpOK && bmiOK;
}

bool SensorAcquisition_Init(void)
{
    calibrationStatus = SENSOR_CAL_IDLE;
    CalibrationStoreResult stored = CalibrationStore_Load(&savedCalibration);
    calibrationValid = stored == CAL_STORE_VALID;
    /* Do not short-circuit startup when one device is absent. */
    bool bmpOK = BMP280_Init(&bmp, &hi2c2, ACQUISITION_BMP_ADDRESS);
    bool ahtOK = AHT20_Init(&aht, &hi2c2);
    bool bmiOK = BMI270_Init(&bmi, &hi2c2, ACQUISITION_BMI_ADDRESS);
    bool magOK = IIS2MDC_Init(&mag, &hi2c2);
    batteryInitialized = Battery_Init();
    gnssInitialized = GNSS_Init(&huart6, ACQUISITION_GNSS_BAUD);
    lastHumidityStart = HAL_GetTick() - HUMIDITY_PERIOD_MS;
    bool restored = !calibrationValid || restoreCalibration();
    return bmpOK && ahtOK && bmiOK && magOK && batteryInitialized && gnssInitialized &&
           restored && stored != CAL_STORE_ERROR;
}

bool SensorAcquisition_StartCalibration(Cansat *c)
{
    if (c == NULL || c->missionState != CANSAT_STATE_LAUNCH_PAD) return false;
    if (calibrationStatus == SENSOR_CAL_RUNNING) return true;
    BMI270Calibration imuCalibration;
    SensorCalibration3D magneticCalibration;
    bool imuSaved = BMI270_GetCalibration(&bmi, &imuCalibration);
    bool magSaved = IIS2MDC_GetCalibration(&mag, &magneticCalibration);
    float temperatureOffset = bmp.initialized ? bmp.temperature_offset_c : 0.0f;
    float pressureOffset = bmp.initialized ? bmp.pressure_offset_pa : 0.0f;
    float humidityOffset = aht.initialized ? aht.humidity_offset_rh : 0.0f;
    calibrationStatus = SENSOR_CAL_FAILED;
    /* Old snapshots must not survive a failed reinitialization as valid data. */
    c->bmp.valid = c->bmp.heightValid = false;
    c->env.valid = c->bmi.valid = c->mag.valid = false;
    c->navigation.altitudeValid = false;
    bool bmpOK = BMP280_Init(&bmp, &hi2c2, ACQUISITION_BMP_ADDRESS);
    if (bmpOK) bmpOK = BMP280_SetCalibration(&bmp, temperatureOffset, pressureOffset);
    if (bmpOK) bmpOK = BMP280_CalibrateGroundPressure(&bmp);
    bool ahtOK = AHT20_Init(&aht, &hi2c2);
    if (ahtOK) ahtOK = AHT20_SetCalibration(&aht, humidityOffset);
    bool bmiOK = BMI270_Init(&bmi, &hi2c2, ACQUISITION_BMI_ADDRESS);
    if (bmiOK && imuSaved) bmiOK = BMI270_SetCalibration(&bmi, &imuCalibration);
    bool magOK = IIS2MDC_Init(&mag, &hi2c2);
    if (magOK && magSaved) magOK = IIS2MDC_SetCalibration(&mag, &magneticCalibration);
    lastHumidityStart = HAL_GetTick() - HUMIDITY_PERIOD_MS;
    if (!(bmpOK && ahtOK && bmiOK && magOK)) {
        (void)restoreCalibration();
        return false;
    }
    if (!BMI270_StartGyroCalibration(&bmi, CALIBRATION_GYRO_SAMPLES,
                                    CALIBRATION_GYRO_TIMEOUT_MS)) {
        (void)restoreCalibration();
        return false;
    }
    calibrationStarted = HAL_GetTick();
    calibrationStatus = SENSOR_CAL_RUNNING;
    return true;
}

bool SensorAcquisition_CalibrateGroundPressure(void)
{
    if (calibrationStatus == SENSOR_CAL_RUNNING) return false;
    /* A failed startup zero must not silently use a previous flight's zero. */
    bmp.groundPressureValid = false;
    bmp.groundPressurePa = 0.0f;
    return bmp.initialized && BMP280_CalibrateGroundPressure(&bmp);
}

SensorCalibrationStatus SensorAcquisition_UpdateCalibration(void)
{
    if (calibrationStatus == SENSOR_CAL_RUNNING) {
        if (bmi.calibration_state == BMI270_CAL_DONE) {
            BMI270Calibration correction;
            SensorCalibrationRecord candidate = {.groundPressurePa = bmp.groundPressurePa};
            bool ready = bmp.groundPressureValid && BMI270_GetCalibration(&bmi, &correction);
            if (ready) memcpy(candidate.gyroBiasDps, correction.gyro_bias_dps, sizeof(candidate.gyroBiasDps));
            if (ready && CalibrationStore_Save(&candidate)) {
                savedCalibration = candidate;
                calibrationValid = true;
                calibrationStatus = SENSOR_CAL_SUCCESS;
            } else {
                (void)restoreCalibration();
                calibrationStatus = SENSOR_CAL_FAILED;
            }
        }
        else if (bmi.calibration_state != BMI270_CAL_COLLECTING ||
                 (uint32_t)(HAL_GetTick() - calibrationStarted) >= CALIBRATION_GYRO_TIMEOUT_MS) {
            BMI270_CancelGyroCalibration(&bmi);
            (void)restoreCalibration();
            calibrationStatus = SENSOR_CAL_FAILED;
        }
    }
    return calibrationStatus;
}

bool SensorAcquisition_HasCalibration(void) { return calibrationValid; }
SensorCalibrationStatus SensorAcquisition_GetCalibrationStatus(void) { return calibrationStatus; }

float SensorAcquisition_GetGroundPressure(void)
{
    return bmp.initialized && bmp.groundPressureValid ? bmp.groundPressurePa : 0.0f;
}

bool SensorAcquisition_UseFallbackGroundPressure(Cansat *c)
{
    if (c == NULL || !bmp.initialized ||
        !BMP280_SetGroundPressure(&bmp, STARTUP_FALLBACK_GROUND_PRESSURE_PA))
        return false;
    /* A conversion may not be ready yet; normal acquisition will refresh it. */
    (void)SensorAcquisition_UpdateBMP(c);
    return true;
}

bool SensorAcquisition_UpdateBMP(Cansat *cansat)
{
    if (cansat == NULL || !bmp.initialized) return false;
    BMP280Data sample;
    if (!BMP280_Read(&bmp, &sample)) return false;
    cansat->bmp = sample;
    /* A candidate zero must not drive transitions before its atomic commit. */
    if (calibrationStatus == SENSOR_CAL_RUNNING) cansat->bmp.heightValid = false;
    return true;
}

bool SensorAcquisition_UpdateAHT(Cansat *cansat)
{
    if (cansat == NULL || !aht.initialized) return false;

    if (aht.state != AHT20_IDLE) {
        AHT20Data sample;
        /* Read checks readiness itself and returns without waiting. */
        if (!AHT20_Read(&aht, &sample)) return false;
        cansat->env = sample;
        return true;
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastHumidityStart) >= HUMIDITY_PERIOD_MS) {
        /* Limit failed trigger retries as well as successful conversions. */
        lastHumidityStart = now;
        (void)AHT20_StartMeasurement(&aht);
    }
    return false;
}

bool SensorAcquisition_UpdateBMI(Cansat *cansat)
{
    if (cansat == NULL || !bmi.initialized) return false;
    BMI270Data sample;
    if (!BMI270_Read(&bmi, &sample)) return false;
    cansat->bmi = sample;
    return true;
}

bool SensorAcquisition_UpdateMag(Cansat *cansat)
{
    if (cansat == NULL || !mag.initialized) return false;
    IIS2MDCData sample;
    if (!IIS2MDC_Read(&mag, &sample)) return false;
    cansat->mag = sample;
    return true;
}

bool SensorAcquisition_UpdateGNSS(Cansat *cansat)
{
    if (cansat == NULL || !gnssInitialized) return false;
    GNSS_Process();
    /* Copy even when false: the driver expires validity independently of the
     * stored coordinates, and fresh GGA with no fix is also a useful snapshot. */
    return GNSS_GetData(&cansat->gnss);
}

bool SensorAcquisition_UpdateBattery(Cansat *cansat)
{
    if (cansat == NULL || !batteryInitialized) return false;
    /* No timestamp is available for battery data. Preserve driver invalidation
     * on conversion failure instead of presenting an old voltage as current. */
    return Battery_Read(&cansat->battery);
}

void SensorAcquisition_Update(Cansat *cansat)
{
    if (cansat == NULL) return;
    (void)SensorAcquisition_UpdateGNSS(cansat);
    (void)SensorAcquisition_UpdateBMP(cansat);
    (void)SensorAcquisition_UpdateAHT(cansat);
    (void)SensorAcquisition_UpdateBMI(cansat);
    (void)SensorAcquisition_UpdateMag(cansat);
    (void)SensorAcquisition_UpdateBattery(cansat);
    (void)SensorAcquisition_UpdateCalibration();
}
