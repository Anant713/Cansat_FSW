#include "apps/Communication.h"
#include "apps/Cansat.h"
#include "apps/SensorAcquisition.h"
#include "apps/StateMachine.h"
#include "drivers/XBee.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart4;
static uint32_t tick;
static bool busy;
static XBeeResult sendResult = XBEE_OK, readResult = XBEE_OK, initResult = XBEE_OK;
static unsigned sends, processes, reads;
static uint8_t sent[XBEE_TX_CAPACITY + 1U];
static uint16_t sentLength;
static const uint8_t incoming[] = {'C', 'M', 'D', 0, 'X', '\r'};
static StateMachineStatus missionStatus;
static SensorCalibrationStatus calibrationResult;
static bool calibrationStartOK, referenceSaveOK;
static unsigned calibrationStarts, referenceSaves;
const StateMachineStatus *StateMachine_GetStatus(void) { return &missionStatus; }
bool SensorAcquisition_StartCalibration(Cansat *c)
{
    assert(c != NULL); ++calibrationStarts;
    calibrationResult = calibrationStartOK ? SENSOR_CAL_RUNNING : SENSOR_CAL_FAILED;
    return calibrationStartOK;
}
SensorCalibrationStatus SensorAcquisition_GetCalibrationStatus(void) { return calibrationResult; }
float SensorAcquisition_GetGroundPressure(void) { return 101234.0f; }
bool StateMachine_RecalibrateGroundPressure(Cansat *c, float pressure)
{
    assert(c != NULL && pressure == 101234.0f); ++referenceSaves;
    return referenceSaveOK && c->missionState == CANSAT_STATE_LAUNCH_PAD;
}

uint32_t HAL_GetTick(void) { return tick; }
XBeeResult XBee_Init(UART_HandleTypeDef *uart)
{
    assert(uart == &huart4);
    return initResult;
}
bool XBee_IsTxBusy(void) { return busy; }
void XBee_Process(void) { ++processes; }
XBeeResult XBee_Send(const uint8_t *bytes, uint16_t length)
{
    assert(length > 0U && length <= XBEE_TX_CAPACITY);
    ++sends;
    if (sendResult != XBEE_OK) return sendResult;
    memcpy(sent, bytes, length);
    sent[length] = 0;
    sentLength = length;
    return XBEE_OK;
}
XBeeResult XBee_Read(uint8_t *bytes, uint16_t capacity, uint16_t *length)
{
    assert(capacity == COMMUNICATION_RX_BUDGET);
    ++reads;
    *length = 0;
    if (readResult != XBEE_OK) return readResult;
    memcpy(bytes, incoming, sizeof(incoming));
    *length = sizeof(incoming);
    return XBEE_OK;
}

static Cansat fixture(void)
{
    Cansat c;
    Cansat_Init(&c);
    c.telemetry.uptimeMs = 123456;
    c.telemetry.packetCount = 7;
    c.telemetry.lastReceivedCommand = 9;
    c.telemetry.commandExecuted = true;
    c.telemetry.receivedPacketCount = 42;
    c.navigation.altitude = 999.0f; /* Deliberately differs from BMP telemetry altitude. */
    c.navigation.targetDistance = 456.7f;
    c.navigation.headingError = -12.34f;
    c.navigation.altitudeValid = c.navigation.targetDistanceValid = c.navigation.headingErrorValid = true;
    c.bmp = (BMP280Data){.pressure = 101325, .temperature = -12.5f, .valid = true, .relativeHeightM = 123.4f, .heightValid = true};
    c.battery = (BatteryData){.voltage = 7.25f, .valid = true};
    c.gnss = (GNSSData){.utcTime = 123459, .utcMilliseconds = 875,
        .latitude = -12.3456, .longitude = -76.5432,
        .latitudeHemisphere = 'S', .longitudeHemisphere = 'W',
        .altitude = 222.5f, .satellites = 9, .speed = 12.5f, .course = 123.45f,
        .valid = true, .fixValid = true, .utcValid = true, .altitudeValid = true,
        .vtgValid = true, .speedValid = true, .courseValid = true};
    c.bmi = (BMI270Data){.accelX = 1.25f, .accelY = -2.5f, .accelZ = 9.75f,
        .gyroX = 12.3f, .gyroY = -45.6f, .gyroZ = 78.9f, .valid = true};
    c.missionState = CANSAT_STATE_DESCENT_2;
    c.env = (AHT20Data){.humidity = 45.625f, .valid = true};
    c.mag = (IIS2MDCData){.magX = 18.25f, .magY = -20, .magZ = 51.25f, .valid = true};
    c.attitude.quaternion[0] = 0.5f;
    c.attitude.quaternion[1] = 0.5f;
    c.attitude.quaternion[2] = -0.5f;
    c.attitude.quaternion[3] = 0.5f;
    c.attitude.quaternionValid = true;
    c.attitude.angularVelocity[0] = 1.25f;
    c.attitude.angularVelocity[1] = -2.25f;
    c.attitude.angularVelocity[2] = 3.25f;
    c.attitude.angularVelocityValid = true;
    c.actuators.parafoilDeployed = true;
    c.actuators.servo = (ServoData){.servo1Angle = 45, .servo2Angle = 135};
    c.actuators.servoValid = true;
    return c;
}


static const unsigned widths[44] = {27,3,2,2,3,2,2,1,1,2,3,3,1,1,2,1,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,1,1,1,1,2};
static const bool signedField[44] = {
    [5]=true, [16]=true, [17]=true, [18]=true, [19]=true, [20]=true,
    [21]=true, [24]=true, [25]=true, [26]=true, [29]=true, [30]=true,
    [31]=true, [32]=true, [33]=true, [34]=true, [35]=true, [38]=true
};
static size_t offset(unsigned index)
{
    size_t position = 0;
    for (unsigned i = 0; i < index; ++i) position += widths[i] + 1U;
    return position;
}
static uint32_t value(const uint8_t *bytes, unsigned index)
{
    assert(index > 0 && index < 44);
    size_t position = offset(index);
    uint32_t result = 0;
    for (unsigned i = 0; i < widths[index]; ++i)
        result |= (uint32_t)bytes[position + i] << (8U * i);
    return result;
}
static void layout(const uint8_t *bytes)
{
    for (unsigned i = 0; i < 43; ++i) assert(bytes[offset(i) + widths[i]] == ',');
    assert(offset(43) == 148 && bytes[150] == '\r');
}

static void test_packet(void)
{
    Cansat c = fixture(), before = c;
    uint8_t bytes[COMMUNICATION_TELEMETRY_CAPACITY + 1U];
    static const uint8_t expected[] = {
        0x32,0x30,0x32,0x36,0x2d,0x49,0x4e,0x2d,0x53,0x50,0x41,0x43,
        0x65,0x2d,0x43,0x41,0x4e,0x2d,0x37,0x55,0x53,0x41,0x54,0x2d,
        0x30,0x31,0x36,0x2c,0x40,0xe2,0x01,0x2c,0x07,0x00,0x2c,0xd2,
        0x04,0x2c,0xcd,0x8b,0x01,0x2c,0x1e,0xfb,0x2c,0x52,0x1c,0x2c,
        0x0c,0x2c,0x22,0x2c,0xe3,0xe9,0x2c,0x40,0xe2,0x01,0x2c,0xf8,
        0xad,0x0b,0x2c,0x53,0x2c,0x57,0x2c,0xb1,0x08,0x2c,0x09,0x2c,
        0x7d,0x00,0x2c,0x06,0xff,0x2c,0xcf,0x03,0x2c,0x7b,0x00,0x2c,
        0x38,0xfe,0x2c,0x15,0x03,0x2c,0x03,0x2c,0x39,0xb2,
        0x00,0x2c,0x21,0x07,0x2c,0x30,0xf8,0x2c,0x05,0x14,0x2c,0xe2,
        0x04,0x2c,0x39,0x30,0x2c,0x88,0x13,0x2c,0x88,0x13,0x2c,0x78,
        0xec,0x2c,0x88,0x13,0x2c,0x0d,0x00,0x2c,0xe9,0xff,0x2c,0x21,
        0x00,0x2c,0x01,0x2c,0xd7,0x11,0x2c,0x2e,0xfb,0x2c,0x2d,0x2c,
        0x87,0x2c,0x09,0x2c,0x01,0x2c,0x2a,0x00,0x0d
    };
    _Static_assert(sizeof(expected) == 151U, "PDF packet budget");
    memset(bytes, 0xaa, sizeof(bytes));
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == sizeof(expected));
    assert(memcmp(bytes, expected, sizeof(expected)) == 0 && bytes[151] == 0xaa);
    assert(memcmp(&c, &before, sizeof(c)) == 0);
    layout(bytes);
    for (size_t capacity = 0; capacity < 151; ++capacity) {
        memset(bytes, 0xaa, sizeof(bytes));
        assert(Communication_FormatTelemetry(&c, bytes, capacity) == 0);
        for (unsigned i = 0; i < sizeof(bytes); ++i) assert(bytes[i] == 0xaa);
    }
    assert(Communication_FormatTelemetry(&c, bytes, 151) == 151);
    assert(!Communication_FormatTelemetry(NULL, bytes, sizeof(bytes)));
    assert(!Communication_FormatTelemetry(&c, NULL, sizeof(bytes)));
    strcpy(c.telemetry.teamId, "TEAM");
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(!memcmp(bytes, "TEAM", 4));
    for (unsigned i = 4; i < 27; ++i) assert(bytes[i] == 0);
    layout(bytes);
    strcpy(c.telemetry.teamId, "TEAM,INJECT");
    assert(!Communication_FormatTelemetry(&c, bytes, sizeof(bytes)));
    memset(c.telemetry.teamId, 'X', sizeof(c.telemetry.teamId));
    assert(!Communication_FormatTelemetry(&c, bytes, sizeof(bytes)));
    puts("Exact 151-byte golden packet, little-endian signed/24-bit encoding and bounds passed");
}

static void test_invalid_and_limits(void)
{
    Cansat c = fixture();
    uint8_t bytes[151];
    c.bmp.heightValid = false;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(value(bytes, 3) == 65535 && value(bytes, 14) == 2225);
    c = fixture();
    c.telemetry.uptimeMs = 0x1000000;
    c.bmp.relativeHeightM = -1;
    c.bmp.pressure = INFINITY; c.bmp.temperature = NAN;
    c.battery.valid = c.gnss.valid = c.bmi.valid = c.env.valid = c.mag.valid = false;
    c.attitude.quaternionValid = c.attitude.angularVelocityValid = false;
    c.navigation.targetDistanceValid = c.navigation.headingErrorValid = false;
    c.actuators.servoValid = false;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    layout(bytes);
    for (unsigned i = 1; i < 41; ++i) {
        if (i == 2 || i == 22 || i == 36) continue;
        uint32_t sentinel = signedField[i] ? 0x8000U : (1UL << (widths[i] * 8U)) - 1U;
        assert(value(bytes, i) == sentinel);
    }
    c = fixture();
    c.telemetry.uptimeMs = 0xfffffe;
    c.bmp.temperature = -327.67f; c.bmp.pressure = 16777214;
    c.battery.voltage = 65.534f;
    c.bmp.relativeHeightM = 6553.4f;
    c.gnss.utcTime = 235960; c.gnss.utcMilliseconds = 999;
    c.gnss.latitude = -90; c.gnss.longitude = -180;
    c.env.humidity = 100;
    c.mag.magX = 327.67f; c.mag.magY = -327.67f;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(value(bytes, 1) == 0xfffffe && value(bytes, 3) == 65534);
    assert(value(bytes, 4) == 16777214 && value(bytes, 5) == 0x8001);
    assert(value(bytes, 6) == 65534 && value(bytes, 9) == 60999);
    assert(value(bytes, 10) == 900000 && value(bytes, 11) == 1800000);
    assert(value(bytes, 23) == 100000);
    assert(value(bytes, 24) == 32767 && value(bytes, 25) == 0x8001);
    /* Sentinel endpoints cannot be mistaken for measurements. Known counters
     * and booleans still have their full binary range. */
    c.telemetry.uptimeMs = 0xffffff;
    c.bmp.temperature = -327.68f; c.bmp.pressure = 16777215;
    c.battery.voltage = 65.535f;
    c.telemetry.packetCount = c.telemetry.receivedPacketCount = 65535;
    c.telemetry.lastReceivedCommand = 255;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(value(bytes, 1) == 0xffffff && value(bytes, 4) == 0xffffff);
    assert(value(bytes, 5) == 0x8000 && value(bytes, 6) == 0xffff);
    assert(value(bytes, 2) == 0xffff);
    assert(value(bytes, 43) == 0xffff);
    assert(value(bytes, 41) == 255 && value(bytes, 42) == 1);
    c.gnss.fixValid = false;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(value(bytes, 7) == 23 && value(bytes, 15) == 9);
    assert(value(bytes, 10) == 0xffffff && value(bytes, 12) == 0xff);
    assert(value(bytes, 14) == 0xffff && value(bytes, 27) == 0xffff);
    puts("Reserved sentinels, finite/range checks, full-range counters and GNSS validity passed");
}

static void test_binary_payload(void)
{
    Cansat c = fixture();
    uint8_t bytes[151];
    c.bmi.accelX = 72.25f;
    c.bmi.accelY = -72.25f;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(bytes[72] == 0x39 && bytes[73] == 0x1c); /* 7225, exactly 2 bytes */
    assert(bytes[75] == 0xc7 && bytes[76] == 0xe3); /* -7225, two's complement */
    c.bmi.accelX = 0.44f; c.bmi.accelY = 0.13f; c.bmi.accelZ = 0;
    assert(Communication_FormatTelemetry(&c, bytes, sizeof(bytes)) == 151);
    assert(bytes[72] == ',' && bytes[75] == '\r' && bytes[78] == 0);
    layout(bytes); /* Embedded delimiters are payload, never split points. */
    busy = false; sendResult = XBEE_OK;
    assert(Communication_SendTelemetry(&c));
    assert(sentLength == 151 && sent[72] == ',' && sent[75] == '\r' && sent[78] == 0);
    puts("7225 -> 39 1C; binary NUL/comma/CR payload transmitted without truncation passed");
}

static void test_send_and_service(void)
{
    Cansat c = fixture();
    c.telemetry.packetCount = 0;
    sends = processes = reads = 0;
    tick = UINT32_MAX - 500U;
    assert(Communication_Init());
    c.telemetry.enabled = true;
    Communication_Update(&c);
    tick += 2000U;
    Communication_Update(&c);
    assert(sends == 0 && processes == 2 && reads == 2); /* Scheduling is in FlightApp. */
    assert(Communication_SendTelemetry(&c));
    assert(sends == 1 && c.telemetry.packetCount == 1);
    assert(sentLength == 151 && value(sent, 2) == 1); layout(sent);
    busy = true;
    assert(!Communication_SendTelemetry(&c) && c.telemetry.packetCount == 1 && sends == 1);
    busy = false;
    XBeeResult failures[] = {XBEE_BUSY, XBEE_NOT_READY, XBEE_IO_ERROR};
    for (unsigned i = 0; i < sizeof(failures)/sizeof(failures[0]); ++i) {
        sendResult = failures[i];
        assert(!Communication_SendTelemetry(&c) && c.telemetry.packetCount == 1);
    }
    sendResult = XBEE_OK; c.telemetry.packetCount = UINT16_MAX;
    assert(Communication_SendTelemetry(&c) && c.telemetry.packetCount == 0 && value(sent, 2) == 0);
    readResult = XBEE_RX_LOST; Communication_Update(&c);
    assert(c.telemetry.receivedPacketCount == 42);
    Cansat before = c;
    Communication_ProcessCommand(&c, incoming, sizeof(incoming));
    assert(memcmp(&before, &c, sizeof(c)) == 0);
    assert(!Communication_SendTelemetry(NULL)); Communication_Update(NULL);
    initResult = XBEE_IO_ERROR; assert(!Communication_Init());
    puts("UART acceptance, counter rollover and continuous RX service passed");
}

static void test_commands(void)
{
    const uint8_t on[] = {0xa5, 1, 0x5a};
    const uint8_t off[] = {0xa5, 2, 0x5a};
    const uint8_t calibrate[] = {0xa5, 3, 0x5a};
    Cansat c;
    initResult = readResult = XBEE_OK;
    for (size_t split = 0; split <= sizeof(on); ++split) {
        Cansat_Init(&c);
        assert(Communication_Init());
        Communication_ProcessCommand(&c, on, split);
        if (split < sizeof(on)) assert(!c.telemetry.enabled);
        Communication_ProcessCommand(&c, on + split, sizeof(on) - split);
        assert(c.telemetry.enabled && c.telemetry.lastReceivedCommand == 1);
        assert(c.telemetry.commandExecuted && c.telemetry.lastExecutedCommand == 1);
        assert(c.telemetry.receivedPacketCount == 1);
    }
    Communication_ProcessCommand(&c, off, sizeof(off));
    assert(!c.telemetry.enabled && c.telemetry.lastExecutedCommand == 2);
    assert(c.telemetry.lastReceivedCommand == 2 && c.telemetry.commandExecuted);
    Communication_ProcessCommand(&c, calibrate, sizeof(calibrate));
    assert(c.telemetry.lastReceivedCommand == 3 && !c.telemetry.commandExecuted);
    assert(c.telemetry.lastExecutedCommand == 2 && c.telemetry.receivedPacketCount == 3);

    const uint8_t malformed[] = {0, 0xa5, 4, 0x5a, 0xa5, 1, 0, 0x5a,
                                 'A', '5', ' ', '0', '1', ' ', '5', 'A'};
    Cansat before = c;
    Communication_ProcessCommand(&c, malformed, sizeof(malformed));
    assert(memcmp(&before, &c, sizeof(c)) == 0);
    const uint8_t joined[] = {0xa5, 0xa5, 1, 0x5a, 0xa5, 2, 0x5a, 0xa5, 1, 0x5a};
    Communication_ProcessCommand(&c, joined, sizeof(joined));
    assert(c.telemetry.receivedPacketCount == 6 && c.telemetry.enabled);
    assert(c.telemetry.lastReceivedCommand == 1 && c.telemetry.commandExecuted);
    c.telemetry.receivedPacketCount = UINT16_MAX;
    Communication_ProcessCommand(&c, on, sizeof(on));
    assert(c.telemetry.receivedPacketCount == 0); /* Repeated valid packets count. */

    XBeeResult errors[] = {XBEE_RX_LOST, XBEE_IO_ERROR, XBEE_NOT_READY};
    for (unsigned i = 0; i < sizeof(errors)/sizeof(errors[0]); ++i) {
        Communication_ProcessCommand(&c, off, 2);
        readResult = errors[i];
        Communication_Update(&c);
        Communication_ProcessCommand(&c, off + 2, 1);
        assert(c.telemetry.enabled && c.telemetry.receivedPacketCount == 0);
    }
    Communication_ProcessCommand(NULL, on, sizeof(on));
    Communication_ProcessCommand(&c, NULL, sizeof(on));
    readResult = XBEE_OK;
    puts("Command framing, chunk splits, resynchronization, status and RX-loss tests passed");
}

static void test_calibration_command(void)
{
    Cansat c;
    Cansat_Init(&c);
    assert(Communication_Init());
    const uint8_t cal[] = {0xa5, 3, 0x5a}, off[] = {0xa5, 2, 0x5a};
    missionStatus.persistenceOK = calibrationStartOK = referenceSaveOK = true;
    calibrationStarts = referenceSaves = 0;
    c.missionState = CANSAT_STATE_ASCENT;
    Communication_ProcessCommand(&c, cal, 3);
    assert(calibrationStarts == 0 && !c.telemetry.commandExecuted);
    c.missionState = CANSAT_STATE_LAUNCH_PAD;
    Communication_ProcessCommand(&c, cal, 3);
    Communication_ProcessCommand(&c, cal, 3);
    assert(calibrationStarts == 1 && !c.telemetry.commandExecuted);
    Communication_Update(&c);
    assert(referenceSaves == 0 && !c.telemetry.commandExecuted);
    calibrationResult = SENSOR_CAL_SUCCESS;
    Communication_Update(&c);
    assert(referenceSaves == 1 && c.telemetry.commandExecuted && c.telemetry.lastExecutedCommand == 3);
    Communication_Update(&c); assert(referenceSaves == 1);
    Communication_ProcessCommand(&c, cal, 3);
    Communication_ProcessCommand(&c, off, 3);
    calibrationResult = SENSOR_CAL_SUCCESS;
    Communication_Update(&c);
    assert(c.telemetry.lastReceivedCommand == 2 && c.telemetry.commandExecuted);
    assert(c.telemetry.lastExecutedCommand == 3 && !c.telemetry.enabled);
    Communication_ProcessCommand(&c, off, 3);
    Communication_ProcessCommand(&c, cal, 3);
    calibrationResult = SENSOR_CAL_FAILED;
    Communication_Update(&c);
    assert(!c.telemetry.commandExecuted && c.telemetry.lastExecutedCommand == 2);
    Communication_ProcessCommand(&c, cal, 3);
    calibrationResult = SENSOR_CAL_SUCCESS; referenceSaveOK = false;
    Communication_Update(&c);
    assert(!c.telemetry.commandExecuted && c.telemetry.lastExecutedCommand == 2);
    calibrationStartOK = false;
    Communication_ProcessCommand(&c, cal, 3);
    assert(!c.telemetry.commandExecuted);
    puts("Calibration gating, async completion, duplicate requests, failure and command association passed");
}

int main(void)
{
    test_packet();
    test_invalid_and_limits();
    test_binary_payload();
    test_send_and_service();
    test_commands();
    test_calibration_command();
    puts("All binary communication host tests passed");
    return 0;
}
