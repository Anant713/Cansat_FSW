#include "apps/EstimationDcs.h"
#include "apps/Cansat.h"
#include "dcs_sensor_transform.h"
#include "main.h"

#include <math.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f
#define MAX_MAG_AGE_MS 200u

static DcsFlightConfig config;
static DcsAttitudeState attitude;
static DcsNavState navigation;
static uint32_t last_bmi_ms, last_gga_ms, last_vtg_ms, last_baro_ms;
static uint32_t last_vertical_ms;
static float last_vertical_altitude_m;
static bool initialized, fresh_velocity;
static bool bmi_seen, gga_seen, vtg_seen, baro_seen, vertical_seen;

static bool finite3(DcsVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static void publish_attitude(Cansat *c, DcsVec3 body_gyro_rad_s)
{
    const DcsQuaternion q = attitude.body_to_ned;
    const float roll_rad = atan2f(q.y * q.z + q.w * q.x,
                                  q.w * q.w + q.z * q.z - 0.5f);
    const float pitch_term = fmaxf(-1.0f, fminf(1.0f,
        2.0f * (q.w * q.y - q.x * q.z)));
    const float pitch_rad = asinf(pitch_term);
    c->attitude.quaternion[0] = q.w;
    c->attitude.quaternion[1] = q.x;
    c->attitude.quaternion[2] = q.y;
    c->attitude.quaternion[3] = q.z;
    c->attitude.quaternionValid = attitude.orientation_valid;
    c->attitude.roll = roll_rad * RAD_TO_DEG;
    c->attitude.pitch = pitch_rad * RAD_TO_DEG;
    c->attitude.yaw = attitude.payload_heading_rad * RAD_TO_DEG;
    c->attitude.angularVelocity[0] = body_gyro_rad_s.x * RAD_TO_DEG;
    c->attitude.angularVelocity[1] = body_gyro_rad_s.y * RAD_TO_DEG;
    c->attitude.angularVelocity[2] = body_gyro_rad_s.z * RAD_TO_DEG;
    c->attitude.angularVelocityValid = attitude.orientation_valid &&
        finite3(body_gyro_rad_s);
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float cos_pitch = cosf(pitch_rad);
    if (fabsf(cos_pitch) > 0.001f) {
        const float tan_pitch = tanf(pitch_rad);
        c->attitude.rollRate = (body_gyro_rad_s.x +
            sin_roll * tan_pitch * body_gyro_rad_s.y +
            cos_roll * tan_pitch * body_gyro_rad_s.z) * RAD_TO_DEG;
        c->attitude.pitchRate = (cos_roll * body_gyro_rad_s.y -
            sin_roll * body_gyro_rad_s.z) * RAD_TO_DEG;
    } else {
        c->attitude.rollRate = c->attitude.pitchRate = 0.0f;
    }
    c->attitude.yawRate = attitude.payload_heading_rate_rad_s * RAD_TO_DEG;
    c->attitude.timestampMs = c->bmi.timestamp_ms;
}

bool Estimation_Init(void)
{
    dcs_flight_config_load(&config);
    initialized = dcs_nav_init(&navigation, &config.navigation) &&
        dcs_attitude_init(&attitude, &config.attitude, HAL_GetTick());
    last_bmi_ms = last_gga_ms = last_vtg_ms = last_baro_ms = 0u;
    last_vertical_ms = 0u;
    last_vertical_altitude_m = 0.0f;
    bmi_seen = gga_seen = vtg_seen = baro_seen = vertical_seen = false;
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

    bool bmi_fresh = c->bmi.valid &&
        (uint32_t)(now - c->bmi.timestamp_ms) <= ESTIMATION_BMI_MAX_AGE_MS;
    if (!bmi_fresh) {
        c->attitude.quaternionValid = false;
        c->attitude.angularVelocityValid = false;
    }
    if (bmi_fresh && (!bmi_seen || c->bmi.timestamp_ms != last_bmi_ms)) {
        const DcsVec3 native_gyro = {c->bmi.gyroX * DEG_TO_RAD,
                                    c->bmi.gyroY * DEG_TO_RAD,
                                    c->bmi.gyroZ * DEG_TO_RAD};
        const DcsVec3 body_gyro = dcs_bmi_vector_to_payload(native_gyro);
        DcsInstalledAttitudeSample sample = {
            .system_time_ms = c->bmi.timestamp_ms,
            .gyro_bmi_rad_s = native_gyro,
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
            publish_attitude(c, body_gyro);
        } else {
            c->attitude.quaternionValid = false;
            c->attitude.angularVelocityValid = false;
        }
        last_bmi_ms = c->bmi.timestamp_ms;
        bmi_seen = true;
    }

    DcsNavGnssFix fix, *fix_ptr = NULL;
    bool new_gga = !gga_seen || c->gnss.ggaReceivedMs != last_gga_ms;
    bool new_vtg = !vtg_seen || c->gnss.vtgReceivedMs != last_vtg_ms;
    if (c->gnss.fixValid && c->gnss.speedValid && c->gnss.courseValid &&
        (new_gga || new_vtg)) {
        const float course = c->gnss.course * DEG_TO_RAD;
        fix = (DcsNavGnssFix){
            .gnss_epoch_ms = new_vtg ? c->gnss.vtgReceivedMs : c->gnss.ggaReceivedMs,
            .system_time_ms = c->gnss.ggaReceivedMs,
            .lat_e7 = (int32_t)lround(c->gnss.latitude * 1.0e7),
            .lon_e7 = (int32_t)lround(c->gnss.longitude * 1.0e7),
            .ground_v_north_mps = c->gnss.speed * cosf(course),
            .ground_v_east_mps = c->gnss.speed * sinf(course),
            .valid = true,
        };
        fix_ptr = &fix;
        fresh_velocity = new_vtg;
        last_gga_ms = c->gnss.ggaReceivedMs;
        last_vtg_ms = c->gnss.vtgReceivedMs;
        gga_seen = vtg_seen = true;
    }
    DcsNavBaroSample baro, *baro_ptr = NULL;
    if (c->navigation.altitudeValid &&
        (!baro_seen || c->navigation.altitudeTimestampMs != last_baro_ms)) {
        baro = (DcsNavBaroSample){c->navigation.altitudeTimestampMs,
                                 c->navigation.altitude, true};
        baro_ptr = &baro;
        if (vertical_seen) {
            uint32_t elapsed = c->navigation.altitudeTimestampMs - last_vertical_ms;
            c->navigation.verticalVelocityValid = elapsed > 0U &&
                elapsed <= ESTIMATION_VERTICAL_RATE_MAX_INTERVAL_MS;
            if (c->navigation.verticalVelocityValid) {
                c->navigation.verticalVelocity =
                    (c->navigation.altitude - last_vertical_altitude_m) *
                    1000.0f / (float)elapsed;
                c->navigation.verticalVelocityValid =
                    isfinite(c->navigation.verticalVelocity);
            }
        }
        last_vertical_ms = c->navigation.altitudeTimestampMs;
        last_vertical_altitude_m = c->navigation.altitude;
        vertical_seen = true;
        last_baro_ms = c->navigation.altitudeTimestampMs;
        baro_seen = true;
    }
    dcs_nav_step(&navigation, &config.navigation, now, fix_ptr, baro_ptr);
    c->navigation.targetDistance = hypotf(navigation.output.north_m,
                                          navigation.output.east_m);
    c->navigation.targetDistanceValid = navigation.output.horizontal_valid &&
        isfinite(c->navigation.targetDistance);
    c->navigation.horizontalVelocity = hypotf(
        navigation.output.ground_v_north_mps,
        navigation.output.ground_v_east_mps);
    c->navigation.horizontalVelocityValid = navigation.output.horizontal_valid &&
        isfinite(c->navigation.horizontalVelocity);
    if (!navigation.output.vertical_valid)
        c->navigation.verticalVelocityValid = false;
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
        .heading_valid = attitude.heading_valid && c->attitude.quaternionValid,
    };
    return true;
}

bool AttitudeEstimator_Init(void) { return Estimation_Init(); }
void AttitudeEstimator_Update(Cansat *c) { Estimation_Update(c); }
bool NavigationEstimator_Init(void) { return initialized; }
void NavigationEstimator_Update(Cansat *c) { Estimation_Update(c); }
