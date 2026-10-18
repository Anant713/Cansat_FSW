#ifndef CANSAT_BMI270_H
#define CANSAT_BMI270_H
#include "drivers/SensorI2C.h"

#define BMI270_ADDRESS_LOW  0x68U
#define BMI270_ADDRESS_HIGH 0x69U
#define BMI270_GRAVITY_MPS2 9.80665f
typedef enum {
    BMI270_CAL_IDLE = 0, BMI270_CAL_COLLECTING, BMI270_CAL_DONE,
    BMI270_CAL_MOTION, BMI270_CAL_TIMEOUT, BMI270_CAL_IO_ERROR
} BMI270CalibrationState;
typedef struct {
    float accelX, accelY, accelZ;    /* m/s^2, including gravity */
    float gyroX, gyroY, gyroZ;       /* degrees/second */
    uint32_t timestamp_ms;
    bool valid;
} BMI270Data;
typedef struct {
    SensorCalibration3D accel;
    float gyro_bias_dps[3];
} BMI270Calibration;
typedef struct {
    SensorI2C io;
    bool initialized;
    int8_t gyro_cross_zx;
    BMI270Calibration calibration;
    BMI270CalibrationState calibration_state;
    uint16_t calibration_target, calibration_count;
    uint32_t calibration_started_ms, calibration_timeout_ms;
    float gyro_mean[3], gyro_m2[3], accel_mean[3], accel_m2[3];
} BMI270;

/* Startup only: uploads the 8192-byte Bosch image. 100 Hz, +/-16 g,
 * +/-2000 deg/s, normal bandwidth/performance modes, no FIFO or IRQs. */
bool BMI270_Init(BMI270 *dev, I2C_HandleTypeDef *bus, uint8_t address);
bool BMI270_IsReady(BMI270 *dev);
/* Reads only when BOTH accel and gyro are fresh. Also advances any pending
 * gyro calibration by one sample; it never waits for the next sample. */
bool BMI270_Read(BMI270 *dev, BMI270Data *data);
bool BMI270_SetCalibration(BMI270 *dev, const BMI270Calibration *calibration);
bool BMI270_GetCalibration(const BMI270 *dev, BMI270Calibration *calibration);
/* Keep stationary throughout. 32..4096 samples; timeout <= 3600000 ms.
 * At a 2 Hz main loop, 64 samples require >=32 s. Existing bias remains until
 * successful completion. Inspect calibration_state after each Read(). */
bool BMI270_StartGyroCalibration(BMI270 *dev, uint16_t samples, uint32_t timeout_ms);
void BMI270_CancelGyroCalibration(BMI270 *dev);
#endif
