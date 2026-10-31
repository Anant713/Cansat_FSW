/* Reuse flash/output fakes, but exercise injected pressure through real gates. */
#define main legacy_state_suite
#include "../state/test_state.c"
#undef main
#include "hils/HilsInput.h"
#include "drivers/CalibrationStore.h"

bool __wrap_BMP280_Init(BMP280 *, I2C_HandleTypeDef *, uint8_t);
bool __wrap_BMP280_Read(BMP280 *, BMP280Data *);
bool __wrap_BMP280_CalibrateGroundPressure(BMP280 *);
/* Real BMP setter has no I/O; equivalent mock avoids linking I2C compensation. */
bool BMP280_SetGroundPressure(BMP280 *bmp, float value)
{
    if (!bmp || !bmp->initialized || !isfinite(value) || value < 30000 || value > 110000) return false;
    bmp->groundPressurePa = value; bmp->groundPressureValid = true; return true;
}
static HilsMessage packet(const char *body, uint32_t now, uint8_t *id)
{
    uint16_t crc = 0xffff;
    for (const unsigned char *p = (const unsigned char *)body; *p; ++p) {
        crc ^= (uint16_t)*p << 8;
        for (unsigned i = 0; i < 8; ++i) crc = crc & 0x8000 ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    char line[160]; snprintf(line, sizeof(line), "%s*%04X", body, crc);
    return HilsInput_Parse(line, now, id);
}
static uint32_t sequence;
static void pressure(BMP280 *bmp, Cansat *c, float height)
{
    char body[100]; tick += 100;
    float p = 101325.0f * powf(1.0f - height / 44330.0f, 1.0f / 0.1903f);
    snprintf(body, sizeof(body), "H1,P,%lu,%.3f,25", (unsigned long)++sequence, (double)p);
    assert(packet(body, tick, NULL) == HILS_PRESSURE);
    assert(__wrap_BMP280_Read(bmp, &c->bmp));
    assert(!__wrap_BMP280_Read(bmp, &(BMP280Data){0})); /* No repeated sample. */
    BarometricAltitude_Update(c);
    StateMachine_Update(c);
}
int main(void)
{
    HilsInput_Init();
    uint8_t id = 0;
    assert(packet("H1,C,3", 0, &id) == HILS_COMMAND && id == 3);
    assert(packet("H1,C,4", 0, &id) == HILS_NONE);
    assert(packet("H1,NEW", 0, NULL) == HILS_NEW_MISSION);
    assert(packet("H1,RESET", 0, NULL) == HILS_RESET);
    assert(packet("H1,STATUS", 0, NULL) == HILS_STATUS);
    assert(HilsInput_Parse("H1,STATUS*0000", 0, NULL) == HILS_NONE);
    const char *bad[] = {"H1,P,0,nan,25", "H1,P,0,100000,inf", "H1,P,0,29999,25",
        "H1,P,-1,100000,25", "H1,P,4294967296,100000,25", "H1,P,0,100000,25,0",
        "H1,P,0,100000,86", "H1,P,,100000,25", "H1,P,0,110001,25"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) assert(packet(bad[i], 0, NULL) == HILS_NONE);
    assert(packet("H1,P,4294967295,101325,25", UINT32_MAX - 100, NULL) == HILS_PRESSURE);
    assert(packet("H1,P,4294967295,101325,25", 0, NULL) == HILS_NONE);
    assert(packet("H1,P,0,101325,25", 0, NULL) == HILS_PRESSURE);
    assert(packet("H1,P,4294967294,101325,25", 1, NULL) == HILS_NONE);
    HilsSample result;
    assert(!HilsInput_Take(1001, &result));
    HilsInput_Init(); tick = 0;
    memset(flash, 0xff, sizeof(flash));
    Cansat c = {0}; BMP280 bmp; I2C_HandleTypeDef bus = {0};
    assert(__wrap_BMP280_Init(&bmp, &bus, BMP280_ADDRESS_HIGH));
    assert(StateMachine_Init(&c));
    assert(!__wrap_BMP280_CalibrateGroundPressure(&bmp));
    for (unsigned i = 0; i < HILS_GROUND_SAMPLES; ++i) pressure(&bmp, &c, 0);
    assert(!c.bmp.heightValid && c.missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(__wrap_BMP280_CalibrateGroundPressure(&bmp));
    SensorCalibrationRecord saved = {.groundPressurePa = bmp.groundPressurePa, .gyroBiasDps = {0.1f, -0.2f, 0.3f}};
    assert(CalibrationStore_Save(&saved));
    for (unsigned i = 1; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) pressure(&bmp, &c, 40);
    assert(c.missionState == CANSAT_STATE_LAUNCH_PAD);
    /* A pause resets the partial confirmation; actuator timers still run. */
    tick += HILS_INPUT_MAX_AGE_MS + 1; StateMachine_Update(&c);
    assert(StateMachine_GetStatus()->heightConfirmCount == 0);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) pressure(&bmp, &c, 40);
    assert(c.missionState == CANSAT_STATE_ASCENT && dvr);
    assert(fabsf(c.bmp.relativeHeightM - 40) < 0.1f);
    assert(fabsf(c.navigation.altitude - c.bmp.relativeHeightM) < 0.01f);
    for (unsigned i = 0; i < 30; ++i) pressure(&bmp, &c, 850);
    assert(c.camera.isRecording);
    pressure(&bmp, &c, 800); pressure(&bmp, &c, 850); /* Noise breaks confirmation. */
    assert(c.missionState == CANSAT_STATE_ASCENT);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) pressure(&bmp, &c, 800);
    assert(c.missionState == CANSAT_STATE_DESCENT_1);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) pressure(&bmp, &c, 590);
    assert(c.missionState == CANSAT_STATE_DESCENT_2 && hdrm);
    /* Reboot during HDRM: real journal restores state and repeats full pulse. */
    c = (Cansat){0}; reset_outputs(); tick = 0; HilsInput_Init();
    assert(__wrap_BMP280_Init(&bmp, &bus, BMP280_ADDRESS_HIGH));
    SensorCalibrationRecord recovered;
    assert(CalibrationStore_Load(&recovered) == CAL_STORE_VALID);
    assert(memcmp(&saved, &recovered, sizeof(saved)) == 0);
    assert(BMP280_SetGroundPressure(&bmp, recovered.groundPressurePa));
    assert(StateMachine_Init(&c));
    assert(c.missionState == CANSAT_STATE_DESCENT_2 && hdrm && StateMachine_GetStatus()->recovered);
    tick = STATE_HDRM_PULSE_MS - 1; StateMachine_Update(&c); assert(hdrm);
    ++tick; StateMachine_Update(&c); assert(!hdrm && c.actuators.parafoilDeployed);
    c = (Cansat){0}; reset_outputs(); assert(StateMachine_Init(&c));
    assert(!hdrm && c.actuators.parafoilDeployed);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) pressure(&bmp, &c, 10);
    assert(c.missionState == CANSAT_STATE_TOUCHDOWN && buzzer && stops && !dvr);
    assert(StateMachine_BeginNewMission(&c));
    assert(c.missionState == CANSAT_STATE_LAUNCH_PAD && !hdrm && !buzzer);
    assert(CalibrationStore_Load(&recovered) == CAL_STORE_VALID);
    puts("HILS tests passed: framing, ranges, replay, stale input, pressure gates, camera, HDRM, touchdown and flash recovery.");
    return 0;
}
