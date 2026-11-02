#ifdef TEST_HDRM_DRONE
#include "../../HDRM_drone_test.c"
#else
#include "../../test_telecommands.c"
#endif
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>

UART_HandleTypeDef huart4;
static uint32_t tick, acquisitions, transmissions, rows, calibrationStarts, missionWrites;
static SensorCalibrationStatus calibration;
static bool saved, finishCalibration;
static uint8_t rx[32];
static uint16_t rxLength;
static char output[16000];
static bool ledOutput;
static ServoData startupServo;
bool Servo_Init(void) { startupServo = (ServoData){0}; return true; }
void Servo_SetAngles(float a, float b) { assert(a == 0 && b == 0); startupServo.servo1Angle = a; startupServo.servo2Angle = b; }
void Servo_Enable(void) { startupServo.enabled = true; }
bool Servo_GetData(ServoData *s) { *s = startupServo; return true; }
void Switches_LED(bool enabled) { ledOutput = enabled; }
#ifdef TEST_HDRM_DRONE
static bool hdrmOutput;
static unsigned hdrmRises;
bool Switches_Init(void) { hdrmOutput = false; return true; }
void Switches_HDRM(bool enabled) { hdrmOutput = enabled; if (enabled) ++hdrmRises; }
void Switches_Update(SwitchState *state) { state->hdrm = hdrmOutput; state->led = ledOutput; }
#else
bool Switches_Init(void) { ledOutput = false; return true; }
void Switches_Update(SwitchState *state) { state->led = ledOutput; }
#endif
uint32_t HAL_GetTick(void) { return tick; }
bool Console_Init(void) { return true; }
void Console_Process(void) {}
bool Console_Printf(const char *format, ...)
{
    va_list args; va_start(args, format);
    size_t used = strlen(output);
    int written = vsnprintf(output + used, sizeof(output) - used, format, args);
    va_end(args);
    assert(written >= 0 && (size_t)written < sizeof(output) - used);
    return true;
}
bool Console_Write(const char *s) { return Console_Printf("%s", s); }
ConsoleLineResult Console_ReadLine(char *s, size_t capacity)
{ (void)s; (void)capacity; return CONSOLE_NO_LINE; }
MissionStoreResult MissionStore_Load(uint32_t *payload) { *payload = 0; return MISSION_STORE_EMPTY; }
bool MissionStore_Save(uint32_t payload) { assert(payload == 0); ++missionWrites; return true; }
bool SensorAcquisition_Init(void) { return true; }
bool SensorAcquisition_CalibrateAtStartup(Cansat *c)
{ assert(c); saved = true; calibration = SENSOR_CAL_SUCCESS; return true; }
void SensorAcquisition_Update(Cansat *c)
{
    ++acquisitions;
    c->bmp.valid = true;
    if (finishCalibration && calibration == SENSOR_CAL_RUNNING) {
        calibration = SENSOR_CAL_SUCCESS; saved = true;
    }
}
bool SensorAcquisition_StartCalibration(Cansat *c)
{
    assert(c->missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(strstr(output, "Calibration requested")); /* Visible before blocking work. */
    ++calibrationStarts; calibration = SENSOR_CAL_RUNNING; return true;
}
SensorCalibrationStatus SensorAcquisition_GetCalibrationStatus(void) { return calibration; }
bool SensorAcquisition_HasCalibration(void) { return saved; }
float SensorAcquisition_GetGroundPressure(void) { return saved ? 101234.0f : 0.0f; }
bool Logging_Init(void) { return true; }
bool Logging_IsReady(void) { return true; }
const char *Logging_GetFilename(void) { return "0:/LOG0000.CSV"; }
bool Logging_Write(const Cansat *c) { assert(c->bmp.valid); ++rows; return true; }
XBeeResult XBee_Init(UART_HandleTypeDef *port) { assert(port == &huart4); return XBEE_OK; }
void XBee_Process(void) {}
bool XBee_IsTxBusy(void) { return false; }
XBeeResult XBee_Send(const uint8_t *bytes, uint16_t length)
{ assert(bytes && length == TELEMETRY_BINARY_SIZE); ++transmissions; return XBEE_OK; }
XBeeResult XBee_Read(uint8_t *bytes, uint16_t capacity, uint16_t *length)
{ assert(capacity >= rxLength); memcpy(bytes, rx, rxLength); *length = rxLength; rxLength = 0; return XBEE_OK; }
void XBee_GetDiagnostics(XBeeDiagnostics *d) { memset(d, 0, sizeof(*d)); }
XBeeStatus XBee_GetStatus(void) { return XBEE_READY; }
static void receive(const uint8_t *bytes, uint16_t length)
{ assert(length <= sizeof(rx)); memcpy(rx, bytes, length); rxLength = length; App_RunOnce(); }
int main(void)
{
    App_Init(); assert(missionWrites == 1 && !test_telecommands_data.telemetry.enabled);
    App_RunOnce(); assert(acquisitions == 1);
    tick = 1000; App_RunOnce(); assert(rows == 1 && transmissions == 0);
    const uint8_t junk[] = {0x41, 0x35, 0x20, 0x30, 0x31};
    receive(junk, sizeof(junk));
    assert(strstr(output, "XBee RAW RX (5 bytes): 41 35 20 30 31"));
    assert(test_telecommands_data.telemetry.receivedPacketCount == 0);
    const uint8_t on[] = {0xa5, 1, 0x5a}, off[] = {0xa5, 2, 0x5a}, cal[] = {0xa5, 3, 0x5a};
    receive(on, 1); receive(on + 1, 1);
    assert(test_telecommands_data.telemetry.receivedPacketCount == 0);
    receive(on + 2, 1);
    assert(strstr(output, "RX command 1: TELEMETRY_ON"));
    assert(strstr(output, "telemetry ON"));
    tick = 1500; App_RunOnce(); assert(transmissions == 1);
    receive(off, 3); tick = 2000; App_RunOnce();
    assert(strstr(output, "XBee RAW RX (3 bytes): A5 02 5A"));
    assert(transmissions == 1 && rows == 2 && strstr(output, "telemetry OFF"));
    receive(cal, 3);
    receive(cal, 3);
    assert(calibrationStarts == 0 && strstr(output, "Calibration command ignored"));
    assert(saved && calibration == SENSOR_CAL_SUCCESS);
    assert(!test_telecommands_data.telemetry.commandExecuted);
    assert(test_telecommands_data.telemetry.lastExecutedCommand == 2);
    output[0] = 0;
    const uint8_t joined[] = {0xa5, 1, 0x5a, 0xa5, 2, 0x5a};
    receive(joined, sizeof(joined));
    assert(strstr(output, "RX command 1") && strstr(output, "RX command 2"));
    assert(!test_telecommands_data.telemetry.enabled);
    tick = 5000; App_RunOnce(); assert(rows == 3 && strstr(output, "TX accepted=1"));
#ifdef TEST_HDRM_DRONE
    tick = hdrm_drone_started_ms + HDRM_FIRE - 1U; App_RunOnce();
    assert(!hdrmOutput && hdrmRises == 0);
    ++tick; App_RunOnce();
    assert(hdrmOutput && hdrmRises == 1 && hdrm_drone_phase == HDRM_ACTIVE);
    tick += HDRM_ON - 1U; App_RunOnce(); assert(hdrmOutput);
    ++tick; App_RunOnce();
    assert(!hdrmOutput && hdrm_drone_phase == HDRM_COMPLETE);
    assert(test_telecommands_data.actuators.parafoilDeployed);
    assert(hdrm_drone_off_ms - hdrm_drone_fired_ms == HDRM_ON);
    tick += HDRM_FIRE + HDRM_ON; App_RunOnce(); assert(hdrmRises == 1);
    /* Reboot arms a fresh one-shot; elapsed comparisons survive tick wrap. */
    output[0] = 0; tick = UINT32_MAX - 1000U; App_Init();
    assert(!hdrmOutput && !test_telecommands_data.actuators.parafoilDeployed);
    tick += HDRM_FIRE; App_RunOnce(); assert(hdrmOutput && hdrmRises == 2);
    tick += HDRM_ON; App_RunOnce(); assert(!hdrmOutput);
    assert(strstr(output, "HDRM ON") && strstr(output, "HDRM OFF"));
    puts("HDRM drone timing passed: boundaries, one-shot, continued tasks, reboot and rollover");
#endif
    puts("Telecommand bench test: real parser, fragmented/batched RX, TX gating, continuous logs and calibration events passed");
    return 0;
}
