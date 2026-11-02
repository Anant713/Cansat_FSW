#include "apps/Telemetry.h"
#include "apps/Cansat.h"
#include "drivers/Servo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum { UNSIGNED_FIELD, SIGNED_FIELD, TEAM_FIELD, HEMISPHERE_FIELD } FieldKind;
typedef struct { unsigned width; FieldKind kind; } FieldLayout;
/* PDF field order; data widths exclude comma separators. */
static const FieldLayout layout[TELEMETRY_FIELD_COUNT] = {
    {27,TEAM_FIELD}, {3,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD},
    {3,UNSIGNED_FIELD}, {2,SIGNED_FIELD}, {2,UNSIGNED_FIELD}, {1,UNSIGNED_FIELD},
    {1,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD}, {3,UNSIGNED_FIELD}, {3,UNSIGNED_FIELD},
    {1,HEMISPHERE_FIELD}, {1,HEMISPHERE_FIELD}, {2,UNSIGNED_FIELD}, {1,UNSIGNED_FIELD},
    {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD},
    {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD},
    {1,UNSIGNED_FIELD}, {3,UNSIGNED_FIELD},
    {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD},
    {2,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD},
    {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD},
    {2,SIGNED_FIELD}, {2,SIGNED_FIELD}, {2,SIGNED_FIELD},
    {1,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD}, {2,SIGNED_FIELD},
    {1,UNSIGNED_FIELD}, {1,UNSIGNED_FIELD},
    {1,UNSIGNED_FIELD}, {1,UNSIGNED_FIELD}, {2,UNSIGNED_FIELD}
};

const char *Telemetry_CSVHeader(void)
{
    return "TEAM_ID,TIME_MS,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,VOLTAGE,"
           "GNSS_HOUR,GNSS_MINUTE,GNSS_SECOND_MS,GNSS_LATITUDE,GNSS_LONGITUDE,"
           "GNSS_LAT_HEMISPHERE,GNSS_LON_HEMISPHERE,GNSS_ALTITUDE,GNSS_SATELLITES,"
           "ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,FLIGHT_STATE,"
           "HUMIDITY,MAG_X,MAG_Y,MAG_Z,GNSS_SPEED,COURSE,QUAT_W,QUAT_X,QUAT_Y,QUAT_Z,"
           "ANGULAR_VELOCITY_X,ANGULAR_VELOCITY_Y,ANGULAR_VELOCITY_Z,PARAFOIL_STATUS,"
           "TARGET_DISTANCE,HEADING_ERROR,LEFT_SERVO,RIGHT_SERVO,LAST_COMMAND,"
           "COMMAND_EXECUTION,RECEIVED_PACKET_COUNT\r\n";
}

size_t Telemetry_FormatCSV(const Cansat *cansat, char *buffer, size_t capacity)
{
    if (buffer == NULL || capacity == 0U) return 0U;
    buffer[0] = '\0';
    uint8_t packet[TELEMETRY_BINARY_SIZE];
    if (Telemetry_Encode(cansat, packet, sizeof(packet)) != sizeof(packet)) return 0U;
    size_t offset = 0, used = 0;
    for (unsigned i = 0; i < TELEMETRY_FIELD_COUNT; ++i) {
        FieldLayout field = layout[i];
        if (offset + field.width >= sizeof(packet)) goto failure;
        char text[CANSAT_TEAM_ID_MAX + 1U];
        size_t length;
        if (field.kind == TEAM_FIELD) {
            length = 0;
            while (length < field.width && packet[offset + length] != 0U) ++length;
            memcpy(text, packet + offset, length);
            text[length] = '\0';
        } else {
            uint32_t value = 0;
            for (unsigned byte = 0; byte < field.width; ++byte)
                value |= (uint32_t)packet[offset + byte] << (8U * byte);
            if (field.kind == HEMISPHERE_FIELD && value != UINT8_MAX) {
                text[0] = (char)value; text[1] = '\0'; length = 1;
            } else {
                int count;
                if (field.kind == SIGNED_FIELD) {
                    int32_t signedValue = value >= 0x8000U ? (int32_t)value - 65536 : (int32_t)value;
                    count = snprintf(text, sizeof(text), "%ld", (long)signedValue);
                } else {
                    count = snprintf(text, sizeof(text), "%lu", (unsigned long)value);
                }
                if (count < 0 || (size_t)count >= sizeof(text)) goto failure;
                length = (size_t)count;
            }
        }
        size_t separator = i == 0U ? 0U : 1U;
        if (separator + length >= capacity - used) goto failure;
        if (separator) buffer[used++] = ',';
        memcpy(buffer + used, text, length);
        used += length;
        offset += field.width;
        if (packet[offset++] != (i + 1U == TELEMETRY_FIELD_COUNT ? '\r' : ',')) goto failure;
    }
    if (offset != sizeof(packet) || capacity - used < 3U) goto failure;
    buffer[used++] = '\r'; buffer[used++] = '\n'; buffer[used] = '\0';
    return used;
failure:
    buffer[0] = '\0';
    return 0U;
}

typedef struct {
    uint8_t *buffer;
    size_t capacity, length;
    unsigned fields;
    bool ok;
} Packet;

static void field(Packet *packet, const uint8_t *bytes, size_t length)
{
    if (!packet->ok) return;
    size_t separator = packet->fields != 0U ? 1U : 0U;
    if (separator + length > packet->capacity - packet->length) {
        packet->ok = false;
        return;
    }
    if (separator) packet->buffer[packet->length++] = ',';
    memcpy(packet->buffer + packet->length, bytes, length);
    packet->length += length;
    ++packet->fields;
}

static void integer(Packet *packet, uint32_t value, unsigned width)
{
    uint8_t bytes[3];
    /* Explicit packing avoids struct padding, alignment, and host-endian dependencies. */
    for (unsigned i = 0; i < width; ++i) bytes[i] = (uint8_t)(value >> (8U * i));
    field(packet, bytes, width);
}

static void number(Packet *packet, double value, double scale,
                   int32_t minimum, int32_t maximum, bool valid, unsigned width)
{
    bool isSigned = minimum < 0;
    int32_t sentinel = isSigned ? minimum : (int32_t)((1UL << (8U * width)) - 1U);
    double scaled = round(value * scale);
    int32_t encoded = sentinel;
    if (valid && isfinite(scaled) && scaled >= minimum && scaled <= maximum &&
        scaled != sentinel)
        encoded = (int32_t)scaled;
    /* Conversion to unsigned is modulo 2^32; low 16 bits encode signed fields. */
    integer(packet, (uint32_t)encoded, width);
}

static void hemisphere(Packet *packet, char value, char positive, char negative,
                       double coordinate, double limit, bool valid)
{
    bool consistent = (value == positive && coordinate >= 0.0) ||
                      (value == negative && coordinate <= 0.0);
    integer(packet, valid && isfinite(coordinate) && fabs(coordinate) <= limit &&
               consistent ? (uint8_t)value : UINT8_MAX, 1);
}

static bool team_id_valid(const char *teamId)
{
    for (size_t i = 0; i <= CANSAT_TEAM_ID_MAX; ++i) {
        unsigned char c = (unsigned char)teamId[i];
        if (c == '\0') return i != 0U;
        if (c < 33U || c > 126U || c == ',' || c == '"') return false;
    }
    return false;
}

static bool fresh(bool valid, uint32_t sample_ms, uint32_t now_ms,
                  uint32_t maximum_age_ms)
{
    return valid && (uint32_t)(now_ms - sample_ms) <= maximum_age_ms;
}

size_t Telemetry_Encode(const Cansat *c, uint8_t *buffer, size_t capacity)
{
    if (buffer == NULL || capacity < TELEMETRY_BINARY_SIZE ||
        c == NULL || !team_id_valid(c->telemetry.teamId)) return 0U;
    Packet packet = {buffer, capacity, 0U, 0U, true};
    const GNSSData *g = &c->gnss;
    bool bmpFresh = fresh(c->bmp.valid, c->bmp.timestamp_ms,
                          c->telemetry.uptimeMs, TELEMETRY_BMP_MAX_AGE_MS);
    bool bmiFresh = fresh(c->bmi.valid, c->bmi.timestamp_ms,
                          c->telemetry.uptimeMs, TELEMETRY_BMI_MAX_AGE_MS);
    bool envFresh = fresh(c->env.valid, c->env.timestamp_ms,
                          c->telemetry.uptimeMs, TELEMETRY_AHT_MAX_AGE_MS);
    bool magFresh = fresh(c->mag.valid, c->mag.timestamp_ms,
                          c->telemetry.uptimeMs, TELEMETRY_MAG_MAX_AGE_MS);
    bool attitudeFresh = fresh(c->attitude.quaternionValid,
                               c->attitude.timestampMs,
                               c->telemetry.uptimeMs,
                               TELEMETRY_ATTITUDE_MAX_AGE_MS);
    unsigned hour = g->utcTime / 10000U;
    unsigned minute = g->utcTime / 100U % 100U;
    unsigned second = g->utcTime % 100U;
    bool utcValid = g->valid && g->utcValid && hour < 24U && minute < 60U &&
                    second <= 60U && g->utcMilliseconds < 1000U;
    bool fixValid = g->valid && g->fixValid;

    uint8_t teamId[CANSAT_TEAM_ID_MAX] = {0};
    memcpy(teamId, c->telemetry.teamId, strlen(c->telemetry.teamId));
    field(&packet, teamId, sizeof(teamId));
    number(&packet, c->telemetry.uptimeMs, 1, 0, 0xffffff, true, 3);
    integer(&packet, c->telemetry.packetCount, 2);
    /* Ground-level noise can produce valid negative heights. Clamp only the
     * transmitted/logged value; preserve raw height for flight processing. */
    double height = c->bmp.relativeHeightM;
    if (isfinite(height) && height < 0.0) height = 0.0;
    number(&packet, height, 10, 0, UINT16_MAX, bmpFresh && c->bmp.heightValid, 2);
    number(&packet, c->bmp.pressure, 1, 0, 0xffffff, bmpFresh, 3);
    number(&packet, c->bmp.temperature, 100, INT16_MIN, INT16_MAX, bmpFresh, 2);
    number(&packet, c->battery.voltage, 1000, 0, UINT16_MAX, c->battery.valid, 2);
    number(&packet, hour, 1, 0, 23, utcValid, 1);
    number(&packet, minute, 1, 0, 59, utcValid, 1);
    number(&packet, second * 1000U + g->utcMilliseconds, 1, 0, UINT16_MAX, utcValid, 2);
    number(&packet, fabs(g->latitude), 10000, 0, 900000, fixValid, 3);
    number(&packet, fabs(g->longitude), 10000, 0, 1800000, fixValid, 3);
    hemisphere(&packet, g->latitudeHemisphere, 'N', 'S', g->latitude, 90, fixValid);
    hemisphere(&packet, g->longitudeHemisphere, 'E', 'W', g->longitude, 180, fixValid);
    number(&packet, g->altitude, 10, 0, UINT16_MAX, fixValid && g->altitudeValid, 2);
    number(&packet, g->satellites, 1, 0, UINT8_MAX, g->valid, 1);
    number(&packet, c->bmi.accelX, 100, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->bmi.accelY, 100, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->bmi.accelZ, 100, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->bmi.gyroX, 10, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->bmi.gyroY, 10, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->bmi.gyroZ, 10, INT16_MIN, INT16_MAX, bmiFresh, 2);
    number(&packet, c->missionState, 1, CANSAT_STATE_LAUNCH_PAD, CANSAT_STATE_TOUCHDOWN, true, 1);
    number(&packet, c->env.humidity, 1000, 0, 100000, envFresh, 3);
    /* Driver gives microtesla: /100 -> gauss, *10000 -> 0.0001 gauss counts. */
    number(&packet, c->mag.magX, 100, INT16_MIN, INT16_MAX, magFresh, 2);
    number(&packet, c->mag.magY, 100, INT16_MIN, INT16_MAX, magFresh, 2);
    number(&packet, c->mag.magZ, 100, INT16_MIN, INT16_MAX, magFresh, 2);
    number(&packet, g->speed, 100, 0, UINT16_MAX, fixValid && g->speedValid, 2);
    number(&packet, g->course, 100, 0, UINT16_MAX, fixValid && g->courseValid, 2);
    for (unsigned i = 0; i < 4U; ++i)
        number(&packet, c->attitude.quaternion[i], 10000, INT16_MIN, INT16_MAX,
               attitudeFresh, 2);
    for (unsigned i = 0; i < 3U; ++i)
        number(&packet, c->attitude.angularVelocity[i], 10, INT16_MIN, INT16_MAX,
               attitudeFresh && c->attitude.angularVelocityValid, 2);
    integer(&packet, c->actuators.parafoilDeployed, 1);
    number(&packet, c->navigation.targetDistance, 10, 0, UINT16_MAX, c->navigation.targetDistanceValid, 2);
    number(&packet, c->navigation.headingError, 100, INT16_MIN, INT16_MAX, c->navigation.headingErrorValid, 2);
    /* Board convention: servo1 = left, servo2 = right; commands, not feedback. */
    number(&packet, c->actuators.servo.servo1Angle, 1, SERVO_MIN_ANGLE_DEG,
           SERVO_MAX_ANGLE_DEG, c->actuators.servoValid, 1);
    number(&packet, c->actuators.servo.servo2Angle, 1, SERVO_MIN_ANGLE_DEG,
           SERVO_MAX_ANGLE_DEG, c->actuators.servoValid, 1);
    integer(&packet, c->telemetry.lastReceivedCommand, 1);
    integer(&packet, c->telemetry.commandExecuted, 1);
    integer(&packet, c->telemetry.receivedPacketCount, 2);

    if (!packet.ok || packet.fields != TELEMETRY_FIELD_COUNT ||
        packet.length != TELEMETRY_BINARY_SIZE - 1U) return 0U;
    buffer[packet.length++] = '\r';
    return packet.length;
}

