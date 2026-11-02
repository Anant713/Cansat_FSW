#include "apps/SensorAcquisition.h"
#include "apps/Cansat.h"
#include "apps/StateMachine.h" /* Shared tagged Cansat must match forward declarations. */
#include "apps/Communication.h"
#include "apps/Logging.h"
#include "App.h"
#include "drivers/CalibrationStore.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart6;
extern Cansat flight_cansat;
extern bool flight_sensor_init_ok;
/* Startup policy has its own tests; this suite isolates acquisition/scheduling. */
bool SensorAcquisition_CalibrateAtStartup(Cansat *c) { return c != NULL; }

enum { BMP, AHT, BMI, MAG, BATTERY, GNSS, DEVICE_COUNT };
static uint32_t tick;
static bool initOK[DEVICE_COUNT], readOK[DEVICE_COUNT];
static unsigned initCalls[DEVICE_COUNT], readCalls[DEVICE_COUNT];
static unsigned starts, processes;
static unsigned gnssRestarts;
static bool gnssReceiving;
static bool triggerOK = true;
static bool humidityFails;
static GNSSData gnssSnapshot;
static unsigned commUpdates, telemetryAttempts, logAttempts;
static bool logReady, sendOK = true;
static uint16_t loggedPacket;
static uint32_t logDelay;
static BMI270 *testBmi;
static unsigned calibrationStarts;
static unsigned correctionRestores;
static CalibrationStoreResult storedResult;
static SensorCalibrationRecord storedCalibration;
static bool storeOK = true;
static unsigned storeWrites, groundSamples;
CalibrationStoreResult CalibrationStore_Load(SensorCalibrationRecord *out)
{ if (storedResult == CAL_STORE_VALID) *out = storedCalibration; return storedResult; }
bool CalibrationStore_Save(const SensorCalibrationRecord *record)
{
    ++storeWrites;
    if (!storeOK) return false;
    storedCalibration = *record; storedResult = CAL_STORE_VALID; return true;
}
bool BMP280_SetGroundPressure(BMP280 *dev, float pressure)
{ dev->groundPressurePa = pressure; dev->groundPressureValid = true; return true; }
bool BMP280_CalibrateGroundPressure(BMP280 *dev)
{ ++groundSamples; return BMP280_SetGroundPressure(dev, 101234); }
bool BMI270_GetCalibration(const BMI270 *dev, BMI270Calibration *out)
{ if (!dev->initialized) return false; *out = dev->calibration; return true; }
bool BMI270_SetCalibration(BMI270 *dev, const BMI270Calibration *value)
{ dev->calibration = *value; ++correctionRestores; return true; }
bool IIS2MDC_GetCalibration(const IIS2MDC *dev, SensorCalibration3D *out)
{ if (!dev->initialized) return false; *out = dev->calibration; return true; }
bool IIS2MDC_SetCalibration(IIS2MDC *dev, const SensorCalibration3D *value)
{ dev->calibration = *value; ++correctionRestores; return true; }
bool BMP280_SetCalibration(BMP280 *dev, float temperature, float pressure)
{ dev->temperature_offset_c = temperature; dev->pressure_offset_pa = pressure; return true; }
bool AHT20_SetCalibration(AHT20 *dev, float humidity)
{ dev->humidity_offset_rh = humidity; return true; }
bool BMI270_StartGyroCalibration(BMI270 *dev, uint16_t count, uint32_t timeout)
{
    assert(count == CALIBRATION_GYRO_SAMPLES && timeout == CALIBRATION_GYRO_TIMEOUT_MS);
    ++calibrationStarts; testBmi = dev; dev->calibration_state = BMI270_CAL_COLLECTING;
    return true;
}
void BMI270_CancelGyroCalibration(BMI270 *dev) { dev->calibration_state = BMI270_CAL_IDLE; }

uint32_t HAL_GetTick(void) { return tick; }
/* State-machine behavior is covered by its dedicated suite. */
bool StateMachine_Init(Cansat *cansat) { (void)cansat; return true; }
void StateMachine_SetCameraReady(bool ready) { (void)ready; }
void StateMachine_Update(Cansat *cansat) { (void)cansat; }
void BarometricAltitude_Update(Cansat *cansat) { (void)cansat; }
bool Estimation_Init(void) { return true; }
void Estimation_Update(Cansat *cansat) { (void)cansat; }
bool Control_Init(void) { return true; }
void Control_Update(Cansat *cansat) { (void)cansat; }
void Error_Handler(void) { assert(!"Unexpected Error_Handler"); }
void CameraControl_Init(CameraState *camera) { assert(camera && !camera->isOn); }
static bool flightLed;
static ServoData flightServo;
bool Switches_Init(void) { flightLed = false; return true; }
void Switches_LED(bool on) { flightLed = on; }
void Switches_Update(SwitchState *s) { s->led = flightLed; }
bool Servo_Init(void) { flightServo = (ServoData){.servo1Angle = 5, .servo2Angle = 5}; return true; }
void Servo_SetAngles(float a, float b) { flightServo.servo1Angle = a; flightServo.servo2Angle = b; }
void Servo_Enable(void) { flightServo.enabled = true; }
bool Servo_GetData(ServoData *data)
{
    *data = flightServo;
    return true;
}
/* Communication behavior is exercised in its own host suite. */
bool Communication_Init(void) { commUpdates = telemetryAttempts = 0; return true; }
void Communication_Update(Cansat *cansat) { (void)cansat; ++commUpdates; }
bool Communication_SendTelemetry(Cansat *cansat)
{
    ++telemetryAttempts;
    if (sendOK) ++cansat->telemetry.packetCount;
    return sendOK;
}
bool Logging_Init(void) { logReady = true; logAttempts = 0; return true; }
bool Logging_IsReady(void) { return logReady; }
bool Logging_Write(const Cansat *cansat)
{
    ++logAttempts;
    loggedPacket = cansat->telemetry.packetCount;
    assert(cansat->telemetry.uptimeMs == tick);
    tick += logDelay;
    return true;
}

bool BMP280_Init(BMP280 *dev, I2C_HandleTypeDef *bus, uint8_t address)
{
    assert(bus == &hi2c2 && address == BMP280_ADDRESS_HIGH);
    ++initCalls[BMP];
    *dev = (BMP280){.initialized = initOK[BMP]};
    return dev->initialized;
}
bool AHT20_Init(AHT20 *dev, I2C_HandleTypeDef *bus)
{
    assert(bus == &hi2c2);
    ++initCalls[AHT];
    *dev = (AHT20){.initialized = initOK[AHT]};
    return dev->initialized;
}
bool BMI270_Init(BMI270 *dev, I2C_HandleTypeDef *bus, uint8_t address)
{
    assert(bus == &hi2c2 && address == BMI270_ADDRESS_LOW);
    ++initCalls[BMI];
    *dev = (BMI270){.initialized = initOK[BMI]};
    testBmi = dev;
    return dev->initialized;
}
bool IIS2MDC_Init(IIS2MDC *dev, I2C_HandleTypeDef *bus)
{
    assert(bus == &hi2c2);
    ++initCalls[MAG];
    *dev = (IIS2MDC){.initialized = initOK[MAG]};
    return dev->initialized;
}
bool Battery_Init(void) { ++initCalls[BATTERY]; return initOK[BATTERY]; }
bool GNSS_Init(UART_HandleTypeDef *uart, uint32_t baud)
{
    assert(uart == &huart6 && baud == GNSS_EXPECTED_BAUD);
    ++initCalls[GNSS];
    gnssReceiving = initOK[GNSS];
    return initOK[GNSS];
}
void GNSS_GetDiagnostics(GNSSDiagnostics *out)
{ *out = (GNSSDiagnostics){.receiving = gnssReceiving}; }
bool GNSS_StartReception(void) { ++gnssRestarts; gnssReceiving = true; return true; }

bool BMP280_Read(BMP280 *dev, BMP280Data *data)
{
    assert(dev->initialized);
    ++readCalls[BMP];
    *data = (BMP280Data){0};
    if (!readOK[BMP]) return false;
    *data = (BMP280Data){.temperature = 23.5f, .pressure = 98765.0f,
                         .timestamp_ms = tick, .valid = true};
    return true;
}
bool BMI270_Read(BMI270 *dev, BMI270Data *data)
{
    assert(dev->initialized);
    ++readCalls[BMI];
    *data = (BMI270Data){0};
    if (!readOK[BMI]) return false;
    *data = (BMI270Data){.accelX = 1, .accelY = -2, .accelZ = 9.8f,
        .gyroX = 4, .gyroY = -5, .gyroZ = 6, .timestamp_ms = tick, .valid = true};
    return true;
}
bool IIS2MDC_Read(IIS2MDC *dev, IIS2MDCData *data)
{
    assert(dev->initialized);
    ++readCalls[MAG];
    *data = (IIS2MDCData){0};
    if (!readOK[MAG]) return false;
    *data = (IIS2MDCData){.magX = 11, .magY = -12, .magZ = 13,
        .overrun = true, .timestamp_ms = tick, .valid = true};
    return true;
}
bool AHT20_StartMeasurement(AHT20 *dev)
{
    assert(dev->initialized && dev->state == AHT20_IDLE);
    ++starts;
    if (!triggerOK) return false;
    dev->state = AHT20_MEASURING;
    dev->started_ms = tick;
    return true;
}
bool AHT20_Read(AHT20 *dev, AHT20Data *data)
{
    assert(dev->initialized && dev->state != AHT20_IDLE);
    ++readCalls[AHT];
    *data = (AHT20Data){0};
    if ((uint32_t)(tick - dev->started_ms) < 80U) return false;
    dev->state = AHT20_IDLE;
    if (humidityFails) return false;
    *data = (AHT20Data){.humidity = 42.5f, .timestamp_ms = tick, .valid = true};
    return true;
}
bool Battery_Read(BatteryData *data)
{
    ++readCalls[BATTERY];
    *data = (BatteryData){0};
    if (!readOK[BATTERY]) return false;
    *data = (BatteryData){.adcRaw = 2048, .voltage = 6.5f, .valid = true};
    return true;
}
void GNSS_Process(void) { ++processes; }
bool GNSS_GetData(GNSSData *data)
{
    /* Every snapshot request must follow parser service. */
    assert(processes == readCalls[GNSS] + 1U);
    ++readCalls[GNSS];
    *data = gnssSnapshot;
    return data->valid;
}

static void reset(void)
{
    storedResult = CAL_STORE_EMPTY; storeOK = true; storeWrites = groundSamples = 0;
    for (unsigned i = 0; i < DEVICE_COUNT; ++i) {
        initOK[i] = readOK[i] = true;
        initCalls[i] = readCalls[i] = 0;
    }
    starts = processes = gnssRestarts = 0;
    gnssReceiving = false;
    tick = 100U;
    triggerOK = true;
    humidityFails = false;
    gnssSnapshot = (GNSSData){.latitude = 12.3, .longitude = -45.6,
        .valid = true, .fixValid = true, .speedValid = true, .speed = 3.5f};
}

static void test_acquisition(void)
{
    reset();
    assert(SensorAcquisition_Init());
    Cansat c;
    Cansat_Init(&c);
    assert(!c.bmp.valid && !c.env.valid && !c.bmi.valid && !c.mag.valid);
    assert(!c.gnss.valid && !c.battery.valid);
    c.missionState = CANSAT_STATE_DESCENT_2;
    c.attitude.roll = 17.0f;
    c.navigation.altitude = 321.0f;
    c.actuators.servo.enabled = true;
    c.actuators.servo.servo1Angle = 55.0f;
    c.switches.dvr = true;
    Cansat expected = c;

    SensorAcquisition_Update(&c);
    assert(c.bmp.temperature == 23.5f && c.bmp.pressure == 98765.0f);
    assert(c.bmi.accelY == -2 && c.bmi.gyroZ == 6);
    assert(c.mag.magY == -12 && c.mag.overrun);
    assert(c.gnss.longitude == gnssSnapshot.longitude && c.gnss.speed == gnssSnapshot.speed);
    assert(c.battery.adcRaw == 2048 && c.battery.voltage == 6.5f);
    assert(starts == 1 && !c.env.valid);
    tick += 79U;
    assert(!SensorAcquisition_UpdateAHT(&c));
    ++tick;
    assert(SensorAcquisition_UpdateAHT(&c));
    assert(c.env.humidity == 42.5f && c.env.timestamp_ms == tick);

    expected.bmp = c.bmp; expected.env = c.env;
    expected.bmi = c.bmi; expected.mag = c.mag;
    expected.gnss = c.gnss; expected.battery = c.battery;
    assert(memcmp(&c, &expected, sizeof(c)) == 0);

    for (unsigned i = 0; i < DEVICE_COUNT; ++i) readOK[i] = false;
    gnssSnapshot.valid = gnssSnapshot.fixValid = gnssSnapshot.speedValid = false;
    tick += 10U;
    SensorAcquisition_Update(&c);
    assert(memcmp(&c.bmp, &expected.bmp, sizeof(c.bmp)) == 0);
    assert(memcmp(&c.bmi, &expected.bmi, sizeof(c.bmi)) == 0);
    assert(memcmp(&c.mag, &expected.mag, sizeof(c.mag)) == 0);
    assert(!c.gnss.valid && !c.gnss.fixValid && !c.gnss.speedValid);
    assert(!c.battery.valid && c.battery.voltage == 0);
    gnssSnapshot.valid = true; /* Fresh GGA with no fix must replace old fix. */
    assert(SensorAcquisition_UpdateGNSS(&c));
    assert(c.gnss.valid && !c.gnss.fixValid);
    gnssReceiving = false;
    (void)SensorAcquisition_UpdateGNSS(&c);
    assert(gnssRestarts == 1 && gnssReceiving);

    tick = 2100U;
    humidityFails = true;
    assert(!SensorAcquisition_UpdateAHT(&c));
    tick += 80U;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(memcmp(&c.env, &expected.env, sizeof(c.env)) == 0);
    tick = 4100U;
    humidityFails = false;
    assert(!SensorAcquisition_UpdateAHT(&c));
    tick += 80U;
    assert(SensorAcquisition_UpdateAHT(&c));
    assert(c.env.timestamp_ms == tick);

    unsigned before = processes;
    SensorAcquisition_Update(NULL);
    assert(!SensorAcquisition_UpdateBMP(NULL));
    assert(!SensorAcquisition_UpdateAHT(NULL));
    assert(!SensorAcquisition_UpdateBMI(NULL));
    assert(!SensorAcquisition_UpdateMag(NULL));
    assert(!SensorAcquisition_UpdateGNSS(NULL));
    assert(!SensorAcquisition_UpdateBattery(NULL));
    Cansat_Init(NULL);
    assert(processes == before);
}

static void test_startup_failure(void)
{
    for (unsigned missing = 0; missing < DEVICE_COUNT; ++missing) {
        reset();
        initOK[missing] = false;
        assert(!SensorAcquisition_Init());
        for (unsigned i = 0; i < DEVICE_COUNT; ++i) assert(initCalls[i] == 1);
        Cansat c;
        Cansat_Init(&c);
        SensorAcquisition_Update(&c);
        tick += 80U;
        SensorAcquisition_Update(&c);
        const bool valid[] = {c.bmp.valid, c.env.valid, c.bmi.valid,
                              c.mag.valid, c.battery.valid, c.gnss.valid};
        for (unsigned i = 0; i < DEVICE_COUNT; ++i) {
            assert(valid[i] == (i != missing));
            if (i == missing) assert(readCalls[i] == 0);
        }
    }
}

static void test_humidity_rollover(void)
{
    reset();
    tick = UINT32_MAX - 100U;
    assert(SensorAcquisition_Init());
    Cansat c;
    Cansat_Init(&c);
    uint32_t first = tick;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(starts == 1);
    tick += 80U;
    assert(SensorAcquisition_UpdateAHT(&c));
    tick = first + 1999U;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(starts == 1);
    ++tick;
    triggerOK = false;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(starts == 2);
    ++tick;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(starts == 2); /* Failed triggers do not cause a tight retry loop. */
    tick = first + 4000U;
    triggerOK = true;
    assert(!SensorAcquisition_UpdateAHT(&c));
    assert(starts == 3);
    tick += 80U;
    assert(SensorAcquisition_UpdateAHT(&c));
}

static void test_flight_integration(void)
{
    reset();
    initOK[BMP] = false;
    tick = UINT32_MAX - 5U;
    uint32_t start = tick;
    App_Init();
    assert(flightLed && flight_cansat.switches.led);
    assert(flight_cansat.actuators.servo.enabled);
    assert(flight_cansat.actuators.servo.servo1Angle == SERVO_MIN_ANGLE_DEG &&
           flight_cansat.actuators.servo.servo2Angle == SERVO_MIN_ANGLE_DEG);
    assert(!flight_sensor_init_ok);
    App_RunOnce();
    assert(!flight_cansat.bmp.valid && flight_cansat.bmi.valid);
    assert(processes == 1);
    tick += 9U;
    App_RunOnce();
    assert(processes == 1);
    ++tick;
    App_RunOnce();
    assert(processes == 2);
    assert(commUpdates == 3 && telemetryAttempts == 0 && logAttempts == 0);
    flight_cansat.telemetry.enabled = true;
    tick = start + 499U; App_RunOnce();
    assert(telemetryAttempts == 0 && logAttempts == 0);
    tick = start + 500U; App_RunOnce();
    assert(telemetryAttempts == 1 && logAttempts == 0);
    tick = start + 999U; App_RunOnce();
    assert(telemetryAttempts == 1 && logAttempts == 0);
    tick = start + 1000U; App_RunOnce();
    assert(telemetryAttempts == 2 && logAttempts == 1 && loggedPacket == 2);
    /* A stalled loop performs one operation, not a burst of missed periods. */
    tick = start + 3500U; logDelay = 700U; App_RunOnce();
    assert(telemetryAttempts == 3 && logAttempts == 2 && loggedPacket == 3);
    logDelay = 0; App_RunOnce();
    assert(telemetryAttempts == 4 && logAttempts == 2);
    sendOK = false;
    tick = start + 4700U; App_RunOnce();
    assert(telemetryAttempts == 5 && logAttempts == 3 && loggedPacket == 4);
    flight_cansat.telemetry.enabled = false;
    tick = start + 5700U; App_RunOnce();
    assert(telemetryAttempts == 5 && logAttempts == 4); /* Logs even with TX disabled. */
    logReady = false; flight_cansat.telemetry.enabled = true;
    tick = start + 6700U; App_RunOnce();
    assert(telemetryAttempts == 6 && logAttempts == 4); /* SD failure doesn't stop TX. */
    assert(commUpdates == 12);
}

static void test_calibration(void)
{
    reset();
    assert(SensorAcquisition_Init());
    Cansat c; Cansat_Init(&c);
    assert(SensorAcquisition_UseFallbackGroundPressure(&c));
    assert(SensorAcquisition_GetGroundPressure() == STARTUP_FALLBACK_GROUND_PRESSURE_PA);
    assert(!SensorAcquisition_UseFallbackGroundPressure(NULL));
    c.bmp.valid = c.bmp.heightValid = c.env.valid = c.bmi.valid = c.mag.valid = true;
    calibrationStarts = correctionRestores = 0;
    assert(SensorAcquisition_StartCalibration(&c));
    assert(!c.bmp.valid && !c.bmp.heightValid && !c.env.valid && !c.bmi.valid && !c.mag.valid);
    assert(calibrationStarts == 1 && correctionRestores == 2);
    assert(initCalls[BMP] == 2 && initCalls[AHT] == 2 && initCalls[BMI] == 2 && initCalls[MAG] == 2);
    assert(initCalls[GNSS] == 1 && initCalls[BATTERY] == 1);
    assert(SensorAcquisition_GetGroundPressure() == 101234);
    assert(SensorAcquisition_StartCalibration(&c) && calibrationStarts == 1);
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_RUNNING);
    testBmi->calibration_state = BMI270_CAL_DONE;
    testBmi->calibration.gyro_bias_dps[0] = 0.25f;
    testBmi->calibration.gyro_bias_dps[1] = -0.125f;
    testBmi->calibration.gyro_bias_dps[2] = 0.5f;
    assert(SensorAcquisition_GetCalibrationStatus() == SENSOR_CAL_RUNNING && storeWrites == 0);
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_SUCCESS);
    assert(SensorAcquisition_HasCalibration() && storeWrites == 1);
    assert(storedCalibration.groundPressurePa == 101234 && storedCalibration.gyroBiasDps[1] == -0.125f);
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_SUCCESS && storeWrites == 1);
    unsigned sampled = groundSamples;
    assert(SensorAcquisition_Init()); /* MCU reboot: no ground sampling, restored biases. */
    assert(groundSamples == sampled && SensorAcquisition_HasCalibration());
    assert(testBmi->calibration.gyro_bias_dps[1] == -0.125f);
    assert(SensorAcquisition_GetGroundPressure() == 101234);
    storeOK = false;
    assert(SensorAcquisition_StartCalibration(&c));
    testBmi->calibration.gyro_bias_dps[1] = 1.0f;
    testBmi->calibration_state = BMI270_CAL_DONE;
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_FAILED);
    assert(testBmi->calibration.gyro_bias_dps[1] == -0.125f); /* rollback */
    assert(storedCalibration.gyroBiasDps[1] == -0.125f);
    storeOK = true;
    assert(SensorAcquisition_StartCalibration(&c));
    testBmi->calibration_state = BMI270_CAL_MOTION;
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_FAILED);
    tick = UINT32_MAX - 100;
    assert(SensorAcquisition_StartCalibration(&c));
    tick += CALIBRATION_GYRO_TIMEOUT_MS;
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_FAILED);
    assert(testBmi->calibration_state == BMI270_CAL_IDLE);
    initOK[MAG] = false;
    unsigned before = calibrationStarts;
    assert(SensorAcquisition_StartCalibration(&c) && calibrationStarts == before + 1);
    testBmi->calibration_state = BMI270_CAL_MOTION;
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_FAILED);
    initOK[MAG] = true; initOK[AHT] = false; before = calibrationStarts;
    assert(SensorAcquisition_StartCalibration(&c) && calibrationStarts == before + 1);
    testBmi->calibration_state = BMI270_CAL_MOTION;
    assert(SensorAcquisition_UpdateCalibration() == SENSOR_CAL_FAILED);
    initOK[AHT] = true; initOK[BMI] = false; before = calibrationStarts;
    assert(!SensorAcquisition_StartCalibration(&c) && calibrationStarts == before);
    assert(!SensorAcquisition_StartCalibration(NULL));
    initOK[BMI] = true; storedResult = CAL_STORE_ERROR;
    assert(!SensorAcquisition_Init());
    assert(!SensorAcquisition_HasCalibration() && SensorAcquisition_GetGroundPressure() == 0);
}

int main(void)
{
    test_acquisition();
    test_startup_failure();
    test_humidity_rollover();
    test_flight_integration();
    test_calibration();
    puts("Sensor acquisition tests passed");
    return 0;
}

