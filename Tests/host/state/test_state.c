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
static unsigned stops;
uint32_t HAL_GetTick(void) { return tick; }
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
static void sample(Cansat *c, uint32_t time, float height)
{
    tick = time;
    c->navigation.altitude = height;
    c->navigation.altitudeValid = true;
    c->navigation.altitudeTimestampMs = time;
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
    assert(StateMachine_Init(&c));
    assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(!StateMachine_GetStatus()->recovered);
    sample(&c, 0, 30); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    sample(&c, 1, 31); assert(c.missionState == CANSAT_STATE_ASCENT && dvr && capture);
    sample(&c, 1000, 800); assert(!c.camera.isRecording);
    sample(&c, 1001, 801); assert(c.camera.isRecording && !capture);
    assert(StateMachine_GetStatus()->crossedArmHeight);
    /* Recover the latch and camera; require a new full five-second window. */
    reset_outputs(); c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c));
    assert(StateMachine_GetStatus()->recovered && StateMachine_GetStatus()->crossedArmHeight);
    assert(c.missionState == CANSAT_STATE_ASCENT && dvr && capture);
    sample(&c, 0, 810);
    sample(&c, 4999, 807); assert(c.missionState == CANSAT_STATE_ASCENT);
    sample(&c, 5000, 808); assert(c.missionState == CANSAT_STATE_ASCENT); /* Exactly 2 isn't >2. */
    sample(&c, 10000, 805); assert(c.missionState == CANSAT_STATE_DESCENT_1);
    assert(StateMachine_GetStatus()->reactionWheelRequested);
    assert(c.actuators.reactionWheel.enabled);
    sample(&c, 10001, 600); assert(!hdrm);
    sample(&c, 10002, 599); assert(hdrm && c.missionState == CANSAT_STATE_DESCENT_2);
    sample(&c, 12001, 590); assert(hdrm);
    sample(&c, 12002, 590); assert(!hdrm && c.actuators.parafoilDeployed);
    reset_outputs(); c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c));
    assert(!hdrm && c.actuators.parafoilDeployed);
    sample(&c, 1, 20); assert(c.missionState == CANSAT_STATE_DESCENT_2);
    sample(&c, 2, 19); assert(c.missionState == CANSAT_STATE_TOUCHDOWN);
    assert(buzzer && !dvr && capture && stops && !c.camera.isOn);
    assert(!c.actuators.reactionWheel.enabled);
    reset_outputs(); c = (Cansat){0};
    assert(StateMachine_Init(&c)); assert(buzzer && !dvr && capture);
    assert(StateMachine_BeginNewMission(&c));
    assert(!buzzer && c.missionState == CANSAT_STATE_LAUNCH_PAD);
    /* Stale and invalid altitudes cannot advance state. */
    c.navigation.altitude = 100; c.navigation.altitudeTimestampMs = 0;
    c.navigation.altitudeValid = true; tick = 2000;
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    c.navigation.altitudeValid = false;
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    /* Failed persistence inhibits transition and camera activation. */
    writesLeft = 0;
    sample(&c, 2001, 100); assert(c.missionState == CANSAT_STATE_LAUNCH_PAD && !dvr);
    assert(!StateMachine_GetStatus()->persistenceOK);
    writesLeft = -1;
    /* Camera pulse handles tick rollover. */
    CameraState camera = {0}; tick = UINT32_MAX - 499U;
    CameraControl_Start(&camera); tick = 499U; CameraControl_Update(&camera);
    assert(capture && !camera.isRecording);
    tick = 500U; CameraControl_Update(&camera); assert(!capture && camera.isRecording);
    CameraControl_Stop(&camera); assert(capture && !dvr && !camera.isRecording);
    /* Reset during deployment repeats the pulse; completion survives reboot. */
    memset(flash, 0xff, sizeof(flash));
    uint32_t ignored;
    (void)MissionStore_Load(&ignored);
    assert(MissionStore_Save(0x103));
    c = (Cansat){0}; tick = 50;
    assert(StateMachine_Init(&c) && hdrm);
    tick = 1000; reset_outputs(); c = (Cansat){0};
    assert(StateMachine_Init(&c) && hdrm);
    tick = 2999; StateMachine_Update(&c); assert(hdrm);
    writesLeft = 0; tick = 3000; StateMachine_Update(&c);
    assert(!hdrm && !StateMachine_GetStatus()->persistenceOK);
    writesLeft = -1;
    /* A full journal refuses writes without erasing the recovered record. */
    memset(flash, 0xff, sizeof(flash));
    (void)MissionStore_Load(&ignored);
    for (unsigned i = 0; i < MISSION_STORE_SIZE / 16U; ++i) assert(MissionStore_Save(1));
    assert(!MissionStore_Save(2));
    assert(MissionStore_Load(&ignored) == MISSION_STORE_VALID && ignored == 1);
    /* Barometric ground calibration persists through ascent reset. */
    memset(flash, 0xff, sizeof(flash)); c = (Cansat){0}; tick = 100;
    assert(StateMachine_Init(&c));
    c.bmp = (BMP280Data){.pressure=101325, .valid=true, .timestamp_ms=tick};
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitudeValid && fabsf(c.navigation.altitude) < 0.01f);
    assert(c.navigation.groundPressurePa == 101325);
    c.bmp.pressure = 95461; tick = c.bmp.timestamp_ms = 200;
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitude > 499 && c.navigation.altitude < 501);
    StateMachine_Update(&c); assert(c.missionState == CANSAT_STATE_ASCENT);
    c = (Cansat){0}; tick = 0;
    assert(StateMachine_Init(&c) && c.navigation.groundPressurePa == 101325);
    c.bmp = (BMP280Data){.pressure=95461, .valid=true, .timestamp_ms=0};
    BarometricAltitude_Update(&c);
    assert(c.navigation.altitude > 499 && c.navigation.altitude < 501);
    tick = 1001; BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    c.bmp.timestamp_ms = tick; c.bmp.pressure = NAN;
    BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    c.bmp.pressure = 0; BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    c.bmp.pressure = 102000; BarometricAltitude_Update(&c);
    assert(c.navigation.altitudeValid && c.navigation.altitude < 0);
    assert(StateMachine_BeginNewMission(&c) && c.navigation.groundPressurePa == 0);
    writesLeft = 0; BarometricAltitude_Update(&c);
    assert(!c.navigation.altitudeValid && c.navigation.groundPressurePa == 0);
    writesLeft = -1;
    /* Legacy mid-flight record without a reference must not auto-zero in air. */
    memset(flash, 0xff, sizeof(flash)); (void)MissionStore_Load(&ignored);
    assert(MissionStore_Save(1)); c = (Cansat){0};
    assert(StateMachine_Init(&c));
    c.bmp = (BMP280Data){.pressure=95461, .valid=true, .timestamp_ms=tick};
    BarometricAltitude_Update(&c); assert(!c.navigation.altitudeValid);
    /* Nonempty flash with no committed record is not a fresh launch. */
    memset(flash, 0xff, sizeof(flash)); flash[0] = 0;
    assert(!StateMachine_Init(&c) && c.missionState == CANSAT_STATE_UNINITIALIZED);
    puts("State tests passed: gates, recovery, torn writes, sentinels, camera/HDRM timing and touchdown");
}
