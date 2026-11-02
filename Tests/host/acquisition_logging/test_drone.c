#define TEST_DRONE_NO_COMMANDS
#define main combined_test_main
#include "test_combined.c"
#undef main

static bool calibrationOK = true;
static unsigned calibrations, discardedReads, fires;
static bool hdrm;
static bool led;
void Switches_LED(bool on) { led = on; }
static ServoData servo;
bool Servo_Init(void) { servo = (ServoData){0}; return true; }
void Servo_SetAngles(float first, float second)
{ assert(first == SERVO_MIN_ANGLE_DEG && second == SERVO_MIN_ANGLE_DEG); servo.servo1Angle = first; servo.servo2Angle = second; }
void Servo_Enable(void) { servo.enabled = true; }
bool Servo_GetData(ServoData *out) { *out = servo; return true; }
bool Switches_Init(void) { hdrm = false; return true; }
void Switches_HDRM(bool on) { hdrm = on; if (on) ++fires; }
void Switches_Update(SwitchState *state) { state->hdrm = hdrm; state->led = led; }
MissionStoreResult MissionStore_Load(uint32_t *payload) { *payload = 0; return MISSION_STORE_VALID; }
bool MissionStore_Save(uint32_t payload) { (void)payload; return true; }
bool SensorAcquisition_CalibrateAtStartup(Cansat *c)
{
    assert(c->missionState == CANSAT_STATE_LAUNCH_PAD);
    assert(servo.enabled && servo.servo1Angle == SERVO_MIN_ANGLE_DEG &&
           servo.servo2Angle == SERVO_MIN_ANGLE_DEG);
    ++calibrations;
    assert(!hdrm);
    return calibrationOK;
}
float SensorAcquisition_GetGroundPressure(void) { return 101325; }
void XBee_Process(void) {}
XBeeResult XBee_Read(uint8_t *bytes, uint16_t capacity, uint16_t *length)
{
    assert(capacity >= 3);
    bytes[0] = 0xa5; bytes[1] = 2; bytes[2] = 0x5a; *length = 3;
    ++discardedReads;
    return XBEE_OK;
}
int main(void)
{
    App_Init();
    assert(drone_servos_ok && test_acquisition_logging_data.actuators.servoValid);
    assert(test_acquisition_logging_data.actuators.servo.enabled);
    assert(test_acquisition_logging_data.actuators.servo.servo1Angle == SERVO_MIN_ANGLE_DEG);
    assert(test_acquisition_logging_data.actuators.servo.servo2Angle == SERVO_MIN_ANGLE_DEG);
    assert(calibrations == 1 && drone_phase == DRONE_WAITING && !hdrm);
    advance(1000); App_RunOnce();
    assert(drone_phase == DRONE_WAITING && !hdrm);
    assert(telemetryCalls > 0 && logCalls > 0 && communicationUpdates == 0 && discardedReads > 0);
    assert(test_acquisition_logging_data.telemetry.enabled);
    App_RunOnce();
    assert(drone_phase == DRONE_WAITING);
    /* Exercise timer boundaries directly, independent of simulated task runtimes. */
    tick = drone_countdown_ms + HDRM_FIRE - 1; droneService(); assert(!hdrm);
    ++tick; droneService(); assert(hdrm && fires == 1);
    tick += HDRM_ON - 1; droneService(); assert(hdrm);
    ++tick; droneService(); assert(!hdrm && drone_phase == DRONE_COMPLETE);
    assert(test_acquisition_logging_data.actuators.parafoilDeployed);
    unsigned sent = telemetryCalls;
    advance(2000); App_RunOnce(); assert(telemetryCalls > sent && fires == 1);
    /* Calibration failure disables firing, even after the nominal delay. */
    consoleOutput[0] = 0; telemetryCalls = 0; calibrationOK = false; App_Init();
    advance(HDRM_FIRE + HDRM_ON); App_RunOnce();
    assert(drone_phase == DRONE_DISABLED && !hdrm && fires == 1);
    consoleOutput[0] = 0; telemetryCalls = 0; calibrationOK = true;
    tick = UINT32_MAX - 100; App_Init();
    tick += HDRM_FIRE; droneService(); assert(hdrm);
    pendingCommand = "stop"; App_RunOnce();
    assert(stopped && !hdrm && drone_phase == DRONE_DISABLED);
    advance(HDRM_ON + HDRM_FIRE); App_RunOnce(); assert(!hdrm);
    assert(servo.enabled && servo.servo1Angle == SERVO_MIN_ANGLE_DEG &&
           servo.servo2Angle == SERVO_MIN_ANGLE_DEG);
    puts("No-command drone: startup calibration, countdown gating, always-on TX, RX discard, pulse, rollover and stop passed");
    return 0;
}
