#include "HilsInput.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

static HilsSample latest;
static bool pending;
static float ground[HILS_GROUND_SAMPLES];
static uint32_t times[HILS_GROUND_SAMPLES];
static unsigned count, next;

void HilsInput_Init(void)
{
    latest = (HilsSample){0}; pending = false; count = next = 0;
}
const HilsSample *HilsInput_Latest(void) { return &latest; }

static uint16_t crc16(const char *text, size_t size)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)(uint8_t)text[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}
static bool number(const char *text, float *out)
{
    char *end;
    errno = 0;
    *out = strtof(text, &end);
    return end != text && *end == '\0' && !errno && isfinite(*out);
}
HilsMessage HilsInput_Parse(const char *line, uint32_t now, uint8_t *command)
{
    if (!line) return HILS_NONE;
    const char *star = strchr(line, '*');
    if (!star || star - line >= 96 || strlen(star + 1) != 4) return HILS_NONE;
    unsigned expected = 0;
    for (unsigned i = 1; i <= 4; ++i) {
        char ch = star[i];
        unsigned digit;
        if (ch >= '0' && ch <= '9') digit = (unsigned)(ch - '0');
        else if (ch >= 'A' && ch <= 'F') digit = (unsigned)(ch - 'A' + 10);
        else if (ch >= 'a' && ch <= 'f') digit = (unsigned)(ch - 'a' + 10);
        else return HILS_NONE;
        expected = (expected << 4) | digit;
    }
    if (crc16(line, (size_t)(star - line)) != expected) return HILS_NONE;
    char body[96];
    memcpy(body, line, (size_t)(star - line)); body[star - line] = '\0';
    if (!strcmp(body, "H1,STATUS")) return HILS_STATUS;
    if (!strcmp(body, "H1,NEW")) return HILS_NEW_MISSION;
    if (!strcmp(body, "H1,RESET")) return HILS_RESET;
    if (strlen(body) == 6 && !strncmp(body, "H1,C,", 5) && body[5] >= '1' && body[5] <= '3') {
        if (command) *command = (uint8_t)(body[5] - '0');
        return HILS_COMMAND;
    }
    if (strncmp(body, "H1,P,", 5)) return HILS_NONE;
    char *seq = body + 5, *p = strchr(seq, ','), *t;
    if (!p) return HILS_NONE;
    *p++ = '\0'; t = strchr(p, ',');
    if (!t) return HILS_NONE;
    *t++ = '\0';
    if (!*seq) return HILS_NONE;
    for (const char *c = seq; *c; ++c) if (*c < '0' || *c > '9') return HILS_NONE;
    char *end; errno = 0;
    unsigned long parsed = strtoul(seq, &end, 10);
    if (errno || *end || parsed > UINT32_MAX) return HILS_NONE;
    uint32_t sequence = (uint32_t)parsed;
    if (latest.present && (sequence - latest.sequence == 0 || sequence - latest.sequence > INT32_MAX))
        return HILS_NONE;
    float pressure, temperature;
    if (!number(p, &pressure) || !number(t, &temperature) ||
        pressure < 30000 || pressure > 110000 || temperature < -40 || temperature > 85)
        return HILS_NONE;
    if (latest.present && (uint32_t)(now - latest.receivedMs) > HILS_INPUT_MAX_AGE_MS)
        count = next = 0;
    latest = (HilsSample){pressure, temperature, sequence, now, true};
    pending = true;
    ground[next] = pressure; times[next] = now;
    next = (next + 1) % HILS_GROUND_SAMPLES;
    if (count < HILS_GROUND_SAMPLES) ++count;
    return HILS_PRESSURE;
}
bool HilsInput_Take(uint32_t now, HilsSample *sample)
{
    if (!sample || !pending || (uint32_t)(now - latest.receivedMs) > HILS_INPUT_MAX_AGE_MS) return false;
    *sample = latest; pending = false; return true;
}
bool HilsInput_GroundPressure(uint32_t now, float *pressure)
{
    if (!pressure || count < HILS_GROUND_SAMPLES ||
        (uint32_t)(now - latest.receivedMs) > HILS_INPUT_MAX_AGE_MS ||
        (uint32_t)(latest.receivedMs - times[next]) < HILS_GROUND_MIN_SPAN_MS) return false;
    float sum = 0, low = ground[0], high = ground[0];
    for (unsigned i = 0; i < HILS_GROUND_SAMPLES; ++i) {
        sum += ground[i];
        if (ground[i] < low) low = ground[i];
        if (ground[i] > high) high = ground[i];
    }
    if (high - low > HILS_GROUND_MAX_SPREAD_PA) return false;
    *pressure = sum / HILS_GROUND_SAMPLES; return true;
}
