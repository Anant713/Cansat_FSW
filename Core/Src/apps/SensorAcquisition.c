#include "apps/SensorAcquisition.h"
#include "apps/Cansat.h"

/* Match the address pins and the receiver's current baud rate on the board. */
#define ACQUISITION_BMP_ADDRESS BMP280_ADDRESS_HIGH // 
#define ACQUISITION_BMI_ADDRESS BMI270_ADDRESS_LOW
#define ACQUISITION_GNSS_BAUD 9600U
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

bool SensorAcquisition_Init(void)
{
    /* Do not short-circuit startup when one device is absent. */
    bool bmpOK = BMP280_Init(&bmp, &hi2c2, ACQUISITION_BMP_ADDRESS);
    bool ahtOK = AHT20_Init(&aht, &hi2c2);
    bool bmiOK = BMI270_Init(&bmi, &hi2c2, ACQUISITION_BMI_ADDRESS);
    bool magOK = IIS2MDC_Init(&mag, &hi2c2);
    batteryInitialized = Battery_Init();
    gnssInitialized = GNSS_Init(&huart6, ACQUISITION_GNSS_BAUD);
    lastHumidityStart = HAL_GetTick() - HUMIDITY_PERIOD_MS;

    return bmpOK && ahtOK && bmiOK && magOK && batteryInitialized && gnssInitialized;
}

bool SensorAcquisition_UpdateBMP(Cansat *cansat)
{
    if (cansat == NULL || !bmp.initialized) return false;
    BMP280Data sample;
    if (!BMP280_Read(&bmp, &sample)) return false;
    cansat->bmp = sample;
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
}
