#include "drivers/BMP.h"
#include "drivers/AHT20.h"
#include "drivers/bmi270.h"
#include "drivers/bmi270_config.h"
#include "drivers/IIS2MDC.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Avoid the Windows CRT's interactive abort dialog on a failed check. */
#undef assert
#define assert(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static I2C_HandleTypeDef bus = {2};
static uint32_t tick, delay_calls, transfers;
static uint8_t regs[128][256], aht_status, aht_frame[7], aht_restored;
static uint8_t image[8192];
static uint16_t image_offset, uploaded;
static bool init_fail, aht_init_fail, mag_reset_stuck;
static HAL_StatusTypeDef next_error;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; ++delay_calls; }
static HAL_StatusTypeDef transfer(I2C_HandleTypeDef *handle, uint16_t addr, uint32_t timeout)
{
    assert(handle == &bus && (addr & 1U) == 0 && (addr >> 1) < 128);
    assert(timeout > 0 && timeout <= 20);
    ++transfers;
    HAL_StatusTypeDef rc = next_error;
    next_error = HAL_OK;
    return rc;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
    uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef rc = transfer(h, addr, timeout);
    if (rc != HAL_OK) return rc;
    assert(size == I2C_MEMADD_SIZE_8BIT);
    addr >>= 1;
    if (addr == 0x1e && n > 1) { assert(reg & 0x80); reg &= 0x7f; }
    memcpy(data, &regs[addr][reg], n);
    if ((addr == 0x68 || addr == 0x69) && reg == 0x0c) regs[addr][3] = 0;
    if (addr == 0x1e && reg == 0x68) regs[addr][0x67] = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
    uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef rc = transfer(h, addr, timeout);
    if (rc != HAL_OK) return rc;
    assert(size == I2C_MEMADD_SIZE_8BIT);
    addr >>= 1;
    if (addr == 0x68 || addr == 0x69) {
        if (reg == 0x5b) {
            assert(n == 2 && data[0] <= 15);
            image_offset = (uint16_t)(((uint16_t)data[1]*16U+data[0])*2U);
            assert(image_offset == uploaded);
        } else if (reg == 0x5e) {
            assert(n % 2 == 0 && image_offset+n <= sizeof(image));
            memcpy(image+image_offset, data, n);
            uploaded += n;
            return HAL_OK;
        } else if (reg == 0x59 && data[0] == 1) {
            assert(uploaded == 8192 && memcmp(image, bmi270_config_file, 8192) == 0);
            regs[addr][0x21] = init_fail ? 2 : 1;
        }
    }
    memcpy(&regs[addr][reg], data, n);
    if (addr == 0x1e && reg == 0x60 && data[0] == 0x20 && !mag_reset_stuck)
        regs[addr][reg] = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t addr,
    uint8_t *data, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef rc = transfer(h, addr, timeout);
    if (rc != HAL_OK) return rc;
    assert(addr == 0x70);
    if (data[0] == 0xac) { assert(n == 3 && data[1] == 0x33 && data[2] == 0); aht_status |= 0x80; }
    else if (data[0] == 0x1b || data[0] == 0x1c || data[0] == 0x1e) {
        assert(n == 3 && data[1] == 0 && data[2] == 0);
    } else if (data[0] == 0xbb || data[0] == 0xbc || data[0] == 0xbe) {
        assert(n == 3 && data[1] == 0x55 && data[2] == 0x42);
        ++aht_restored;
        if (aht_restored == 3 && !aht_init_fail) aht_status = 0x18;
    }
    else assert(data[0] == 0x71 && n == 1);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t addr,
    uint8_t *data, uint16_t n, uint32_t timeout)
{
    HAL_StatusTypeDef rc = transfer(h, addr, timeout);
    if (rc != HAL_OK) return rc;
    assert(addr == 0x70);
    if (n == 1) *data = aht_status;
    else if (n == 3) { data[0] = 0; data[1] = 0x55; data[2] = 0x42; }
    else { assert(n == 7); memcpy(data, aht_frame, 7); data[0] = aht_status; }
    return HAL_OK;
}
static void reset(void)
{
    memset(regs, 0, sizeof(regs)); memset(image, 0, sizeof(image));
    tick = 0; delay_calls = 0; transfers = 0; next_error = HAL_OK;
    uploaded = 0; image_offset = 0; init_fail = false; aht_init_fail = false;
    mag_reset_stuck = false; aht_status = 0x18; aht_restored = 0;
    regs[0x76][0xd0] = regs[0x77][0xd0] = 0x58;
    regs[0x68][0] = regs[0x69][0] = 0x24;
    regs[0x1e][0x4f] = 0x40;
}
static void le(uint8_t *out, int value) { out[0] = (uint8_t)value; out[1] = (uint8_t)((unsigned)value >> 8); }
static void near(float actual, float expected, float tolerance)
{
    if (fabsf(actual-expected) > tolerance) fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
    assert(fabsf(actual-expected) <= tolerance);
}
static void bmp_fixture(uint8_t address)
{
    const int coeff[] = {27504,26435,-1000,36477,-10685,3024,2855,140,-7,15500,-14600,6000};
    for (unsigned i = 0; i < 12; ++i) le(&regs[address][0x88+2*i], coeff[i]);
    const uint32_t raw[] = {415148,519888};
    for (unsigned i = 0; i < 2; ++i) {
        regs[address][0xf7+3*i] = (uint8_t)(raw[i] >> 12);
        regs[address][0xf8+3*i] = (uint8_t)(raw[i] >> 4);
        regs[address][0xf9+3*i] = (uint8_t)(raw[i] << 4);
    }
}
static void test_bmp(void)
{
    reset(); bmp_fixture(0x77);
    BMP280 dev; BMP280Data out;
    assert(BMP280_Init(&dev, &bus, 0x77));
    assert(regs[0x77][0xf4] == 0x57 && regs[0x77][0xf5] == 0x28);
    assert(dev.groundPressureValid);
    near(dev.groundPressurePa, 100653.26f, 0.1f);
    assert(BMP280_Read(&dev, &out) && out.heightValid);
    near(out.relativeHeightM, 0, 0.01f);
    float ground = dev.groundPressurePa;
    dev.groundPressurePa = out.pressure / 0.9f;
    assert(BMP280_Read(&dev, &out) && out.heightValid);
    near(out.relativeHeightM, 879.93f, 0.1f);
    dev.groundPressurePa = ground;
    tick += 44;
    assert(BMP280_Read(&dev, &out));
    near(out.temperature, 25.0825f, 0.001f); near(out.pressure, 100653.26f, 0.1f);
    assert(BMP280_Calibrate(&dev, &out, 26.0f, 101000.0f));
    regs[0x77][0xf3] = 8; /* last complete sample remains readable */
    assert(BMP280_Read(&dev, &out)); near(out.temperature, 26, 0.001f); near(out.pressure, 101000, 0.1f);
    near(out.relativeHeightM, 0, 0.01f); /* pressure offset also shifts local reference */
    assert(!BMP280_SetMode(&dev, BMP280_FORCED));
    assert(regs[0x77][0xf4] == 0x54 && dev.io.status == SENSOR_NOT_READY);
    regs[0x77][0xf3] = 0;
    assert(BMP280_SetMode(&dev, BMP280_FORCED));
    assert(!BMP280_Read(&dev, &out)); tick += 44;
    assert(BMP280_Read(&dev, &out) && dev.mode == BMP280_SLEEP);
    assert(!BMP280_Read(&dev, &out) && !out.valid);
    assert(BMP280_SetMode(&dev, BMP280_NORMAL)); tick += 44;
    next_error = HAL_TIMEOUT;
    assert(!BMP280_Read(&dev, &out) && !out.valid && dev.io.status == SENSOR_TIMEOUT);
    regs[0x77][0xd0] = 0x60;
    assert(!BMP280_Init(&dev, &bus, 0x77) && !dev.initialized && dev.io.status == SENSOR_WRONG_ID);
    reset();
    assert(!BMP280_Init(&dev, &bus, 0x76) && dev.io.status == SENSOR_BAD_DATA);
    bmp_fixture(0x76); regs[0x76][0xf3] = 1;
    assert(!BMP280_Init(&dev, &bus, 0x76) && dev.io.status == SENSOR_TIMEOUT);
    reset(); bmp_fixture(0x76); regs[0x76][0xf3] = 8;
    assert(!BMP280_Init(&dev, &bus, 0x76) && dev.io.status == SENSOR_TIMEOUT && !dev.initialized);
    reset(); bmp_fixture(0x76);
    regs[0x76][0xf7] = 0x80; regs[0x76][0xf8] = regs[0x76][0xf9] = 0;
    assert(!BMP280_Init(&dev, &bus, 0x76) && !dev.groundPressureValid && !dev.initialized);
    puts("BMP280: factory compensation, offsets, mode transitions and failures passed");
}
static void test_aht(void)
{
    reset(); AHT20 dev; AHT20Data out;
    assert(AHT20_Init(&dev, &bus));
    uint32_t delays = delay_calls;
    tick = UINT32_MAX-40U;
    assert(AHT20_StartMeasurement(&dev));
    assert(!AHT20_StartMeasurement(&dev));
    uint32_t calls = transfers;
    tick += 79;
    assert(!AHT20_IsReady(&dev) && transfers == calls); /* no premature bus polling */
    tick += 1;
    assert(!AHT20_IsReady(&dev) && dev.io.status == SENSOR_NOT_READY);
    aht_status = 0x18;
    /* Fixed CRC fixture (init ff, polynomial 31), 50% RH, 25 C. */
    const uint8_t frame[] = {0x18,0x80,0,0x06,0,0,0x23};
    memcpy(aht_frame, frame, 7);
    tick += 1000; /* completed data can be consumed by a slow main loop */
    calls = transfers;
    assert(AHT20_IsReady(&dev));
    assert(transfers == calls + 1); /* one frame read, no status command */
    assert(AHT20_IsReady(&dev) && AHT20_Read(&dev, &out) && out.valid);
    assert(transfers == calls + 1); /* Read consumes cache without more I2C */
    near(out.humidity, 50, 0.001f);
    assert(AHT20_Calibrate(&dev, &out, 52));
    assert(AHT20_StartMeasurement(&dev)); tick += 80; aht_status = 0x18;
    assert(AHT20_Read(&dev, &out)); near(out.humidity, 52, 0.001f);
    assert(!AHT20_Read(&dev, &out) && !out.valid);
    assert(AHT20_StartMeasurement(&dev)); tick += 80; aht_status = 0x18; aht_frame[6] ^= 1;
    assert(!AHT20_Read(&dev, &out) && !out.valid && dev.io.status == SENSOR_BAD_DATA);
    assert(dev.crc_errors == 1 && dev.calculated_crc == 0x23 && dev.frame[6] == 0x22);
    assert(AHT20_StartMeasurement(&dev)); tick += 500;
    assert(dev.crc_errors == 1); /* trigger must not erase failure evidence */
    assert(!AHT20_IsReady(&dev) && dev.io.status == SENSOR_TIMEOUT && dev.state == AHT20_IDLE);
    assert(AHT20_StartMeasurement(&dev)); tick += 80; next_error = HAL_ERROR;
    assert(!AHT20_IsReady(&dev) && dev.io.status == SENSOR_IO_ERROR && dev.state == AHT20_IDLE);
    assert(delay_calls == delays); /* no conversion waits in any runtime path */
    reset(); aht_status = 0;
    assert(AHT20_Init(&dev, &bus) && aht_restored == 3); /* factory register restore */
    aht_status = 0; aht_init_fail = true;
    assert(!AHT20_Init(&dev, &bus) && dev.io.status == SENSOR_CALIBRATION_ERROR);
    puts("AHT20: deferred conversion, tick wrap, late read, CRC, recovery and offsets passed");
}
static void imu_sample(uint8_t address, int gx, int gy, int gz)
{
    regs[address][3] = 0xc0;
    le(&regs[address][0x0c], 0); le(&regs[address][0x0e], 0); le(&regs[address][0x10], 2048);
    le(&regs[address][0x12], gx); le(&regs[address][0x14], gy); le(&regs[address][0x16], gz);
}
static void test_bmi(void)
{
    reset(); BMI270 dev; BMI270Data out;
    regs[0x69][0x3c] = 0x7f; /* -1 cross-axis coefficient */
    assert(BMI270_Init(&dev, &bus, 0x69) && uploaded == 8192);
    assert(dev.gyro_cross_zx == -1 && regs[0x69][0x40] == 0xa8 && regs[0x69][0x42] == 0xe8);
    uint32_t delays = delay_calls;
    assert(!BMI270_Read(&dev, &out) && !out.valid && dev.io.status == SENSOR_NOT_READY);
    imu_sample(0x69, 100, -100, 512);
    assert(BMI270_Read(&dev, &out));
    near(out.accelZ, 9.80665f, 0.0001f); near(out.gyroX, 101*2000.0f/32768, 0.0001f);
    near(out.gyroY, -100*2000.0f/32768, 0.0001f);
    assert(!BMI270_Read(&dev, &out)); /* fresh flag consumed */
    assert(BMI270_StartGyroCalibration(&dev, 32, 1000));
    BMI270Calibration saved;
    assert(BMI270_GetCalibration(&dev, &saved));
    assert(!BMI270_SetCalibration(&dev, &saved));
    for (unsigned i = 0; i < 32; ++i) {
        tick += 10; imu_sample(0x69, 16, -8, 4);
        assert(BMI270_Read(&dev, &out));
        assert(dev.calibration_count == i+1);
    }
    assert(dev.calibration_state == BMI270_CAL_DONE);
    near(out.gyroX, 0, 0.0001f); near(out.gyroY, 0, 0.0001f);
    assert(BMI270_GetCalibration(&dev, &saved)); near(saved.gyro_bias_dps[0], 0.9765625f, 0.0001f);
    assert(BMI270_StartGyroCalibration(&dev, 32, 1000));
    imu_sample(0x69, 200, 0, 0); assert(BMI270_Read(&dev, &out));
    assert(dev.calibration_state == BMI270_CAL_MOTION);
    near(dev.calibration.gyro_bias_dps[0], saved.gyro_bias_dps[0], 0.0001f);
    assert(BMI270_StartGyroCalibration(&dev, 32, 1000));
    tick += 1000; assert(!BMI270_Read(&dev, &out));
    assert(dev.calibration_state == BMI270_CAL_TIMEOUT);
    assert(BMI270_StartGyroCalibration(&dev, 32, 1000));
    next_error = HAL_ERROR; assert(!BMI270_Read(&dev, &out) && !out.valid);
    assert(dev.calibration_state == BMI270_CAL_IO_ERROR);
    saved.accel.bias[2] = 1.0f;
    saved.accel.matrix[0][2] = 0.5f;
    assert(BMI270_SetCalibration(&dev, &saved));
    imu_sample(0x69, 16, -8, 4); assert(BMI270_Read(&dev, &out));
    near(out.accelZ, 8.80665f, 0.0001f); near(out.accelX, 4.403325f, 0.0001f);
    assert(delay_calls == delays);
    reset(); init_fail = true;
    assert(!BMI270_Init(&dev, &bus, 0x68) && !dev.initialized && dev.io.status == SENSOR_BAD_DATA);
    reset(); regs[0x68][0] = 0;
    assert(!BMI270_Init(&dev, &bus, 0x68) && dev.io.status == SENSOR_WRONG_ID);
    puts("BMI270: full image, signed units/CAS, incremental gyro calibration and failures passed");
}
static void test_mag(void)
{
    reset(); IIS2MDC dev; IIS2MDCData out;
    assert(IIS2MDC_Init(&dev, &bus));
    assert(regs[0x1e][0x60] == 0x88 && regs[0x1e][0x61] == 2 && regs[0x1e][0x62] == 0x10);
    uint32_t delays = delay_calls;
    assert(!IIS2MDC_Read(&dev, &out) && !out.valid);
    le(&regs[0x1e][0x68], 100); le(&regs[0x1e][0x6a], -200); le(&regs[0x1e][0x6c], 300);
    regs[0x1e][0x67] = 0x88;
    assert(IIS2MDC_Read(&dev, &out) && out.overrun);
    near(out.magX, 15, 0.001f); near(out.magY, -30, 0.001f); near(out.magZ, 45, 0.001f);
    SensorCalibration3D cal = {{5, -10, 15}, {{1,0.5f,0},{0,2,0},{0,0,1}}, 2};
    assert(IIS2MDC_SetCalibration(&dev, &cal));
    regs[0x1e][0x67] = 8;
    assert(IIS2MDC_Read(&dev, &out));
    near(out.magX, 0, 0.001f); near(out.magY, -80, 0.001f); near(out.magZ, 60, 0.001f);
    memset(cal.matrix, 0, sizeof(cal.matrix));
    assert(!IIS2MDC_SetCalibration(&dev, &cal));
    assert(IIS2MDC_GetCalibration(&dev, &cal) && cal.matrix[1][1] == 2);
    next_error = HAL_ERROR;
    assert(!IIS2MDC_Read(&dev, &out) && !out.valid && dev.io.status == SENSOR_IO_ERROR);
    assert(delay_calls == delays);
    reset(); mag_reset_stuck = true;
    assert(!IIS2MDC_Init(&dev, &bus) && !dev.initialized && dev.io.status == SENSOR_TIMEOUT);
    puts("IIS2MDC: incrementing burst, signed uT, overrun, full matrix and failures passed");
}
int main(void)
{
    test_bmp(); test_aht(); test_bmi(); test_mag();
    puts("All sensor host tests passed.");
    return 0;
}
