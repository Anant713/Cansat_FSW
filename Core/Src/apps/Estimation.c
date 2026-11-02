#include "apps/EstimationDcs.h"
#include "apps/Cansat.h"
#include "main.h"

#include <math.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f
#define MAX_MAG_AGE_MS 200u

static DcsFlightConfig config;
static DcsAttitudeState attitude;
static DcsNavState navigation;
static uint32_t last_bmi_ms, last_gga_ms, last_vtg_ms, last_baro_ms;
static bool initialized, fresh_velocity;

bool Estimation_Init(void)
{
    dcs_flight_config_load(&config);
    initialized = dcs_nav_init(&navigation, &config.navigation) &&
        dcs_attitude_init(&attitude, &config.attitude, HAL_GetTick());
    last_bmi_ms = last_gga_ms = last_vtg_ms = last_baro_ms = 0u;
    fresh_velocity = false;
    return initialized;
}

const DcsFlightConfig *Estimation_GetFlightConfig(void)
{
    return initialized ? &config : NULL;
}

void Estimation_Update(Cansat *c)
{
    if (!initialized || c == NULL) return;
    const uint32_t now = HAL_GetTick();
    fresh_velocity = false;

    if (c->bmi.valid && c->bmi.timestamp_ms != last_bmi_ms) {
        DcsInstalledAttitudeSample sample = {
            .system_time_ms = c->bmi.timestamp_ms,
            .gyro_bmi_rad_s = {c->bmi.gyroX * DEG_TO_RAD,
                               c->bmi.gyroY * DEG_TO_RAD,
                               c->bmi.gyroZ * DEG_TO_RAD},
            .accel_bmi_mps2 = {c->bmi.accelX, c->bmi.accelY, c->bmi.accelZ},
            .mag_iis2mdc_calibrated = {c->mag.magX, c->mag.magY, c->mag.magZ},
            .gyro_valid = true,
            .accel_valid = true,
            .mag_valid = c->mag.valid &&
                (uint32_t)(now - c->mag.timestamp_ms) <= MAX_MAG_AGE_MS,
            .accel_correction_allowed = true,
            .mag_correction_allowed = true,
            .body_alpha_z_rad_s2 = 0.0f,
        };
        if (dcs_attitude_update_installed(&attitude, &config.attitude, &sample)) {
            c->attitude.quaternion[0] = attitude.body_to_ned.w;
            c->attitude.quaternion[1] = attitude.body_to_ned.x;
            c->attitude.quaternion[2] = attitude.body_to_ned.y;
            c->attitude.quaternion[3] = attitude.body_to_ned.z;
            c->attitude.quaternionValid = attitude.orientation_valid;
            c->attitude.yaw = attitude.payload_heading_rad * RAD_TO_DEG;
            c->attitude.yawRate = attitude.payload_heading_rate_rad_s * RAD_TO_DEG;
            c->attitude.angularVelocity[2] = c->attitude.yawRate;
            c->attitude.angularVelocityValid = true;
        }
        last_bmi_ms = c->bmi.timestamp_ms;
    }

    DcsNavGnssFix fix, *fix_ptr = NULL;
    if (c->gnss.fixValid && c->gnss.speedValid && c->gnss.courseValid &&
        (c->gnss.ggaReceivedMs != last_gga_ms || c->gnss.vtgReceivedMs != last_vtg_ms)) {
        const float course = c->gnss.course * DEG_TO_RAD;
        fix = (DcsNavGnssFix){
            .gnss_epoch_ms = c->gnss.vtgReceivedMs,
            .system_time_ms = c->gnss.ggaReceivedMs,
            .lat_e7 = (int32_t)lround(c->gnss.latitude * 1.0e7),
            .lon_e7 = (int32_t)lround(c->gnss.longitude * 1.0e7),
            .ground_v_north_mps = c->gnss.speed * cosf(course),
            .ground_v_east_mps = c->gnss.speed * sinf(course),
            .valid = true,
        };
        fix_ptr = &fix;
        fresh_velocity = c->gnss.vtgReceivedMs != last_vtg_ms;
        last_gga_ms = c->gnss.ggaReceivedMs;
        last_vtg_ms = c->gnss.vtgReceivedMs;
    }
    DcsNavBaroSample baro, *baro_ptr = NULL;
    if (c->navigation.altitudeValid && c->navigation.altitudeTimestampMs != last_baro_ms) {
        baro = (DcsNavBaroSample){c->navigation.altitudeTimestampMs,
                                 c->navigation.altitude, true};
        baro_ptr = &baro;
        last_baro_ms = c->navigation.altitudeTimestampMs;
    }
    dcs_nav_step(&navigation, &config.navigation, now, fix_ptr, baro_ptr);
    c->navigation.targetDistance = hypotf(navigation.output.north_m,
                                          navigation.output.east_m);
    c->navigation.targetDistanceValid = navigation.output.horizontal_valid;
}

bool Estimation_MakeRuntimeInput(const Cansat *c, DcsRuntimeInput *input)
{
    if (!initialized || c == NULL || input == NULL) return false;
    *input = (DcsRuntimeInput){
        .now_ms = HAL_GetTick(),
        .navigation = {
            .position = {navigation.output.north_m, navigation.output.east_m},
            .ground_v_north_mps = navigation.output.ground_v_north_mps,
            .ground_v_east_mps = navigation.output.ground_v_east_mps,
            .height_above_target_m = -navigation.output.down_m,
            .horizontal_valid = navigation.output.horizontal_valid,
            .horizontal_propagated = navigation.output.horizontal_propagated,
            .vertical_valid = navigation.output.vertical_valid,
        },
        .has_fresh_gnss_velocity = fresh_velocity,
        .gnss_epoch_ms = c->gnss.vtgReceivedMs,
        .gnss_system_time_ms = c->gnss.vtgReceivedMs,
        .heading_rad = attitude.payload_heading_rad,
        .heading_rate_rad_s = attitude.payload_heading_rate_rad_s,
        .heading_valid = attitude.heading_valid,
    };
    return true;
}

bool AttitudeEstimator_Init(void) { return Estimation_Init(); }
void AttitudeEstimator_Update(Cansat *c) { Estimation_Update(c); }
bool NavigationEstimator_Init(void) { return initialized; }
void NavigationEstimator_Update(Cansat *c) { Estimation_Update(c); }
