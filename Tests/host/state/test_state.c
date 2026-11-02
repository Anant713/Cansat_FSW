#include "apps/StateMachine.h"
#include "apps/CameraControl.h"
#include "apps/BarometricAltitude.h"
#include "drivers/MissionStore.h"
#include "apps/Logging.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint32_t flash[MISSION_STORE_SIZE / 4U], tick;
static int writesLeft = -1;
static bool dvr, capture, hdrm, buzzer;
static unsigned stops, finalRows;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t MissionStore_ReadWord(uint32_t offset) { return flash[offset / 4U]; }
bool MissionStore_WriteWord(uint32_t offset, uint32_t value)
{
    if (writesLeft == 0) return false;
    if (writesLeft > 0) --writesLeft;
    assert(flash[offset / 4U] == UINT32_MAX);
    flash[offset / 4U] = value;
    return true;
}
void Switches_DVR(bool value) { dvr = value; }
void Switch_Capture(bool value) { capture = value; }
void Switches_HDRM(bool value) { hdrm = value; }
void Switches_Buzzer(bool value) { buzzer = value; }
void Switches_Update(SwitchState *s) { *s = (SwitchState){.dvr=dvr,.capture=capture,.hdrm=hdrm,.buzzer=buzzer}; }
bool Logging_Stop(void) { ++stops; return true; }
bool Logging_Write(const Cansat *c) { assert(c); ++finalRows; return true; }
static void sample(Cansat *c, uint32_t time, float height)
{
    tick = time;
    c->bmp.relativeHeightM = height;
    c->bmp.valid = c->bmp.heightValid = true;
    c->bmp.timestamp_ms = time;
    StateMachine_Update(c);
}
static void reset_outputs(void) { dvr = capture = hdrm = buzzer = false; }
int main(void)
{
    /* Commit-last journal survives interruption at each word boundary. */
    for (int cut = 0; cut < 4; ++cut) {
        memset(flash, 0xff, sizeof(flash));
        uint32_t payload = 99;
        assert(MissionStore_Load(&payload) == MISSION_STORE_EMPTY);
        writesLeft = -1;
        assert(MissionStore_Save(1));
        writesLeft = cut;
        assert(!MissionStore_Save(0x101));
        assert(MissionStore_Load(&payload) == MISSION_STORE_VALID && payload == 1);
        writesLeft = -1;
        assert(MissionStore_Save(0x101));
        assert(MissionStore_Load(&payload) == MISSION_STORE_VALID && payload == 0x101);
    }
    memset(flash, 0xff, sizeof(flash));
    Cansat c = {0};
    StateMachine_SetCameraReady(false);
    assert(StateMachine_Init(&c));
    assert(!dvr && !c.camera.isOn);
    StateMachine_Update(&c); assert(!dvr);
    StateMachine_SetCameraReady(true);
    StateMachine_Update(&c);
    assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(dvr && capture && c.camera.phase == CAMERA_CAPTURE_HIGH);
    assert(!StateMachine_GetStatus()->recovered);
    sample(&c, 0, 30); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(!StateMachine_GetStatus()->maxHeightValid);
    sample(&c, 1, 31); assert(c.missionState == CANSAT_STATE_ASCENT && dvr && capture);
    assert(!StateMachine_GetStatus()->maxHeightValid); /* Seed on next ASCENT check. */
    sample(&c, 999, 100); assert(!c.camera.isRecording);
    sample(&c, 1001, 120);
    assert(c.camera.isRecording && !capture);
    assert(StateMachine_GetStatus()->maxHeightM == 120);
    /* Old arming bit is ignored; reboot seeds a new peak from valid BMP height. */
    reset_outputs(); c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c));
    assert(StateMachine_GetStatus()->recovered && !StateMachine_GetStatus()->maxHeightValid);
    assert(c.missionState == CANSAT_STATE_ASCENT && dvr && capture);
    c.navigation.altitude = 10000; c.navigation.altitudeValid = true;
    sample(&c, 0, 100);
    sample(&c, 1, 730); assert(StateMachine_GetStatus()->maxHeightM == 730);
    sample(&c, 2, 710); assert(c.missionState == CANSAT_STATE_ASCENT); /* Exactly 20: stay. */
    c.bmp.relativeHeightM = 10000; c.bmp.heightValid = false;
    StateMachine_Update(&c); assert(StateMachine_GetStatus()->maxHeightM == 730);
    c.bmp.heightValid = true; c.bmp.relativeHeightM = NAN;
    StateMachine_Update(&c); assert(StateMachine_GetStatus()->maxHeightM == 730);
    sample(&c, 3, 709); assert(c.missionState == CANSAT_STATE_DESCENT_1); /* Peak above 700 m; no time window. */
    sample(&c, 10001, 600); assert(!hdrm);
    assert(StateMachine_GetStatus()->maxHeightM == 730); /* Frozen in DESCENT_1. */
    sample(&c, 10002, 599); assert(hdrm && c.missionState == CANSAT_STATE_DESCENT_2);
    sample(&c, 10003, 700);
    assert(StateMachine_GetStatus()->maxHeightM == 730); /* Frozen in DESCENT_2. */
    sample(&c, 10002U + STATE_HDRM_PULSE_MS - 1U, 590); assert(hdrm);
    sample(&c, 10002U + STATE_HDRM_PULSE_MS, 590);
    assert(!hdrm && c.actuators.parafoilDeployed);
    reset_outputs(); c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c));
    assert(!hdrm && c.actuators.parafoilDeployed);
    sample(&c, 1, 20); assert(c.missionState == CANSAT_STATE_DESCENT_2);
    sample(&c, 2, 19); assert(c.missionState == CANSAT_STATE_TOUCHDOWN);
    assert(buzzer && !dvr && capture && stops && finalRows && !c.camera.isOn);
    reset_outputs(); c = (Cansat){0};
    assert(StateMachine_Init(&c)); assert(buzzer && !dvr && capture);
    assert(StateMachine_BeginNewMission(&c));
    assert(!buzzer && c.missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(!StateMachine_GetStatus()->maxHeightValid);
    /* Stale and invalid altitudes cannot advance state. */
    c.bmp.relativeHeightM = 100; c.bmp.timestamp_ms = 0;
    c.bmp.valid = c.bmp.heightValid = true; tick = 2000;
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    c.bmp.heightValid = false;
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    /* Failed persistence inhibits transition; launch-pad recording stays active. */
    writesLeft = 0;
    sample(&c, 2001, 100); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD && dvr);
    assert(!StateMachine_GetStatus()->persistenceOK);
    writesLeft = -1;
    /* Startup primes recording then stops; normal start is direct and wrap-safe. */
    CameraState camera = {0}; tick = 0;
    CameraControl_Init(&camera);
    assert(tick == CAMERA_START_PULSE_MS + CAMERA_INIT_RECORD_HOLD_MS + CAMERA_INIT_STOP_SETTLE_MS);
    assert(!dvr && capture && !camera.isOn && !camera.isRecording && camera.phase == CAMERA_OFF);
    tick = UINT32_MAX - 499U;
    CameraControl_Start(&camera);
    assert(dvr && capture && !camera.isRecording);
    uint32_t start = camera.lastCaptureHighMs;
    CameraControl_Start(&camera); assert(camera.lastCaptureHighMs == start);
    tick = 499U; CameraControl_Update(&camera); assert(capture && !camera.isRecording);
    tick = 500U; CameraControl_Update(&camera); assert(!capture && camera.isRecording);
    CameraControl_Start(&camera); assert(camera.isRecording && !capture);
    CameraControl_Stop(&camera); assert(capture && !dvr && !camera.isRecording);
    CameraControl_Start(&camera); CameraControl_Stop(&camera);
    tick += 10000; CameraControl_Update(&camera);
    assert(!dvr && capture && camera.phase == CAMERA_OFF);
    /* Reset during deployment repeats the pulse; completion survives reboot. */
    memset(flash, 0xff, sizeof(flash));
    uint32_t ignored;
    (void)MissionStore_Load(&ignored);
    assert(MissionStore_Save(0x103));
    c = (Cansat){0}; tick = 50;
    assert(StateMachine_Init(&c) && hdrm);
    tick = 1000; reset_outputs(); c = (Cansat){0};
    assert(StateMachine_Init(&c) && hdrm);
    tick = 1000U + STATE_HDRM_PULSE_MS - 1U; StateMachine_Update(&c); assert(hdrm);
    writesLeft = 0; tick = 1000U + STATE_HDRM_PULSE_MS; StateMachine_Update(&c);
    assert(!hdrm && !StateMachine_GetStatus()->persistenceOK);
    writesLeft = -1;
    /* A full journal refuses writes without erasing the recovered record. */
    memset(flash, 0xff, sizeof(flash));
    (void)MissionStore_Load(&ignored);
    for (unsigned i = 0; i < MISSION_STORE_SIZE / 16U; ++i) assert(MissionStore_Save(1));
    assert(!MissionStore_Save(2));
    assert(MissionStore_Load(&ignored) == MISSION_STORE_VALID && ignored == 1);
    /* Navigation uses the same restored BMP reference as telemetry and gates. */
    memset(flash, 0xff, sizeof(flash)); c = (Cansat){0}; tick = 100;
    assert(StateMachine_Init(&c));
    c.bmp = (BMP280Data){.pressure=101325, .groundPressurePa=101325,
        .valid=true, .heightValid=true, .timestamp_ms=tick};
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitudeValid && fabsf(c.navigation.altitude) < 0.01f);
    assert(c.navigation.groundPressurePa == 101325);
    c.bmp.pressure = 95461; tick = c.bmp.timestamp_ms = 200;
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitude > 499 && c.navigation.altitude < 501);
    c.bmp.relativeHeightM = 500;
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_ASCENT);
    c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c) && c.navigation.groundPressurePa == 0);
    /* Acquisition restores the reference; state recovery never recalibrates it. */
    c.bmp = (BMP280Data){.pressure=95461, .groundPressurePa=101325,
        .valid=true, .heightValid=true, .timestamp_ms=0};
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitude > 499 && c.navigation.altitude < 501);
    tick = 1001; BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    c.bmp.timestamp_ms = tick; c.bmp.pressure = NAN;
    BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    c.bmp.pressure = 102000; BarometricAltitude_Update(&c);
    assert(c.navigation.altitudeValid && c.navigation.altitude < 0);
    c.bmp.heightValid = false; BarometricAltitude_Update(&c);
    assert(!c.navigation.altitudeValid && c.navigation.groundPressurePa == 0);
    /* Old pressure-only mission records cannot stand in for the calibration pair. */
    memset(flash, 0xff, sizeof(flash)); (void)MissionStore_Load(&ignored);
    assert(MissionStore_Save(1U | (101325U << MISSION_PRESSURE_SHIFT)));
    c = (Cansat){0}; assert(StateMachine_Init(&c));
    assert(c.missionState == CANSAT_STATE_ASCENT && c.navigation.groundPressurePa == 0);
    c.bmp = (BMP280Data){.pressure=95461, .valid=true, .timestamp_ms=tick};
    BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    /* Nonempty flash with no committed record is not a fresh launch. */
    memset(flash, 0xff, sizeof(flash)); flash[0] = 0;
    assert(!StateMachine_Init(&c) && c.missionState == CANSAT_STATE_UNINITIALIZED);
    puts("State tests passed: gates, recovery, torn writes, sentinels, camera/HDRM timing and touchdown");
    return 0;
}



