#include "drivers/GNSS.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define LINE_SIZE 128U
#define CONFIG_TIMEOUT_MS 2500U
#define QUEUE_MASK (GNSS_QUEUE_SIZE-1U)
_Static_assert((GNSS_QUEUE_SIZE & QUEUE_MASK) == 0, "Queue size must be a power of two");

static UART_HandleTypeDef *port;
static uint8_t dma_rx[GNSS_DMA_SIZE] __attribute__((aligned(4)));
static uint8_t queue[GNSS_QUEUE_SIZE];
static uint32_t arrival[GNSS_QUEUE_SIZE];
static volatile uint32_t head, tail, loss_epoch;
static uint32_t seen_loss;
static uint16_t dma_position;
static volatile GNSSDiagnostics diagnostics;
static GNSSData latest;
static bool have_gga, have_vtg;
static char line[LINE_SIZE];
static uint16_t line_length;
static uint32_t parser_tick;
static uint8_t tx_frame[40], config_step, ack;
static volatile bool tx_busy;
static bool waiting, query_reply;
static uint32_t command_started_ms, observation_started_ms;
static uint32_t previous_utc_ms;
static bool have_previous_utc;
static const uint16_t query_ids[] = {865, 51, 63, 67};
static bool setting_sentences;
static uint8_t setting_index;
static bool begin_verification(bool set_sentences);
static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }

static void reset_parser(void) { line_length = 0; }
static void invalidate(void)
{
    memset(&latest, 0, sizeof(latest));
    have_gga = have_vtg = false;
    reset_parser();
}

bool GNSS_StopReception(void)
{
    if (port == NULL) return false;
    diagnostics.receiving = false;
    if (diagnostics.configState == GNSS_CONFIG_BUSY) diagnostics.configState = GNSS_CONFIG_IO_ERROR;
    bool ok = HAL_UART_AbortReceive(port) == HAL_OK;
    invalidate();
    return ok;
}

bool GNSS_StartReception(void)
{
    if (port == NULL || port->hdmarx == NULL || port->hdmarx->Init.Mode != DMA_CIRCULAR) return false;
    if (diagnostics.receiving) return true;
    if (HAL_UART_AbortReceive(port) != HAL_OK) return false;
    uint32_t p = lock();
    head = tail = 0; dma_position = 0; seen_loss = loss_epoch;
    invalidate();
    diagnostics.receiving = true;
    unlock(p);
    if (HAL_UARTEx_ReceiveToIdle_DMA(port, dma_rx, GNSS_DMA_SIZE) != HAL_OK) {
        diagnostics.receiving = false;
        return false;
    }
    return true;
}

bool GNSS_Init(UART_HandleTypeDef *uart, uint32_t currentBaud)
{
    if (uart == NULL || uart->hdmarx == NULL || currentBaud != GNSS_EXPECTED_BAUD ||
        uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        uart->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        uart->hdmarx->Init.MemInc != DMA_MINC_ENABLE ||
        uart->hdmarx->Init.PeriphInc != DMA_PINC_DISABLE ||
        uart->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
        uart->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) return false;
    if (port != NULL) {
        if (!GNSS_StopReception() || HAL_UART_AbortTransmit(port) != HAL_OK) return false;
    }
    port = uart;
    diagnostics = (GNSSDiagnostics){0};
    tx_busy = waiting = false;
    loss_epoch = seen_loss = 0;
    uart->Init.BaudRate = currentBaud;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(uart) != HAL_OK) return false;
    diagnostics.baudRate = currentBaud;
    if (!GNSS_StartReception()) return false;
    return begin_verification(true);
}

void GNSS_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size;
    if (uart != port || !diagnostics.receiving) return;
    /* Sample the current DMA position rather than the event's historic Size.
     * This prevents duplicate copying if IDLE and HT/TC are serviced out of order.
     * IRQs must still be serviced within a half-buffer interval. */
    uint16_t position = (uint16_t)(GNSS_DMA_SIZE-__HAL_DMA_GET_COUNTER(port->hdmarx));
    if (position >= GNSS_DMA_SIZE) position = 0;
    uint16_t count = (uint16_t)((position+GNSS_DMA_SIZE-dma_position)%GNSS_DMA_SIZE);
    if (count == 0) return;
    uint32_t now = HAL_GetTick();
    diagnostics.lastByteMs = now;
    diagnostics.receivedBytes += count;
    uint32_t write = head;
    if ((uint32_t)(write-tail) + count > GNSS_QUEUE_SIZE) {
        diagnostics.queueDrops += count;
        ++loss_epoch;
    } else {
        for (uint16_t i = 0; i < count; ++i) {
            queue[(write+i)&QUEUE_MASK] = dma_rx[(dma_position+i)%GNSS_DMA_SIZE];
            arrival[(write+i)&QUEUE_MASK] = now;
        }
        __DMB();
        head = write+count;
    }
    dma_position = position;
}

void GNSS_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    diagnostics.lastUartError = uart->ErrorCode;
    ++diagnostics.uartErrors;
    diagnostics.receiving = false;
    ++loss_epoch;
    if (diagnostics.configState == GNSS_CONFIG_BUSY)
        diagnostics.configState = GNSS_CONFIG_IO_ERROR;
    /* No HAL restarts or parsing in this callback. Explicit main-loop recovery. */
}

void GNSS_TxCompleteCallback(UART_HandleTypeDef *uart)
{
    if (uart == port) tx_busy = false;
}

/* Strict decimal syntax: rejects empty, NaN/Inf, exponents and trailing junk. */
static bool number(const char *s, double *value)
{
    bool negative = false, dot = false, digit = false;
    double n = 0, scale = 1;
    if (*s == '-') { negative = true; ++s; }
    for (; *s; ++s) {
        if (*s == '.' && !dot) { dot = true; continue; }
        if (*s < '0' || *s > '9') return false;
        digit = true;
        if (dot) { scale *= 0.1; n += (*s-'0')*scale; }
        else n = n*10+(*s-'0');
        if (n > 1.0e9) return false;
    }
    if (!digit) return false;
    *value = negative ? -n : n;
    return true;
}
static bool integer(const char *s, unsigned max, unsigned *value)
{
    if (!*s) return false;
    unsigned n = 0;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9') return false;
        n = n*10U+(unsigned)(*s-'0');
        if (n > max) return false;
    }
    *value = n; return true;
}
static bool utc(const char *s, uint32_t *hhmmss, uint16_t *ms)
{
    size_t n = strlen(s);
    if (n < 6 || (n > 6 && (s[6] != '.' || n == 7))) return false;
    unsigned time = 0;
    for (unsigned i = 0; i < 6; ++i) {
        if (s[i] < '0' || s[i] > '9') return false;
        time = time*10U+(unsigned)(s[i]-'0');
    }
    if (time/10000U > 23 || (time/100U)%100U > 59 || time%100U > 60) return false;
    if (time%100U == 60 && time/100U != 2359) return false;
    *ms = 0;
    unsigned weight = 100;
    for (size_t i = 7; i < n; ++i) {
        if (s[i] < '0' || s[i] > '9') return false;
        *ms += (uint16_t)((s[i]-'0')*weight); weight /= 10;
    }
    *hhmmss = time; return true;
}
static bool coordinate(const char *s, const char *hemisphere, bool latitude, double *out)
{
    unsigned digits = latitude ? 4U : 5U;
    if (strlen(s) < digits || (s[digits] && s[digits] != '.') || strlen(hemisphere) != 1) return false;
    for (unsigned i = 0; i < digits; ++i) if (s[i] < '0' || s[i] > '9') return false;
    double n;
    if (!number(s, &n) || n < 0) return false;
    unsigned degrees = (unsigned)(n/100.0);
    double minutes = n-degrees*100.0;
    unsigned limit = latitude ? 90U : 180U;
    if (minutes >= 60 || degrees > limit || (degrees == limit && minutes != 0)) return false;
    char h = *hemisphere;
    if (latitude ? (h != 'N' && h != 'S') : (h != 'E' && h != 'W')) return false;
    *out = (degrees+minutes/60.0)*((h == 'S' || h == 'W') ? -1.0 : 1.0);
    return true;
}
static bool gga(char **f, unsigned count, uint32_t tick)
{
    if (count < 15) return false;
    GNSSData next = latest;
    unsigned quality, satellites;
    if (!integer(f[6], 8, &quality) || !integer(f[7], 99, &satellites)) return false;
    next.fixQuality = (uint8_t)quality; next.satellites = (uint8_t)satellites;
    next.fixValid = quality == 1 || quality == 2;
    next.utcValid = *f[1] != 0;
    next.utcTime = 0; next.utcMilliseconds = 0;
    if (next.utcValid && !utc(f[1], &next.utcTime, &next.utcMilliseconds)) return false;
    next.latitude = next.longitude = 0;
    next.latitudeHemisphere = next.longitudeHemisphere = 0;
    next.altitude = next.hdop = 0;
    next.altitudeValid = false;
    next.hdopValid = false;
    if (next.fixValid) {
        if (!next.utcValid || !coordinate(f[2], f[3], true, &next.latitude) ||
            !coordinate(f[4], f[5], false, &next.longitude)) return false;
        next.latitudeHemisphere = *f[3]; next.longitudeHemisphere = *f[5];
        double v;
        if (*f[8]) {
            if (!number(f[8], &v) || v < 0 || v > 999) return false;
            next.hdop = (float)v;
            next.hdopValid = true;
        }
        if (*f[9]) {
            if (strcmp(f[10], "M") || !number(f[9], &v) || fabs(v) > 100000) return false;
            next.altitude = (float)v; next.altitudeValid = true;
        }
    }
    next.valid = true; next.ggaReceivedMs = tick;
    latest = next; have_gga = true; ++diagnostics.goodGga;
    if (next.utcValid) {
        uint32_t ms = (next.utcTime / 10000U * 3600U + next.utcTime / 100U % 100U * 60U + next.utcTime % 100U) * 1000U + next.utcMilliseconds;
        if (have_previous_utc) {
            uint32_t delta = (ms + 86400000U - previous_utc_ms) % 86400000U;
            diagnostics.observedPeriodMs = delta;
            if (delta == GNSS_EXPECTED_PERIOD_MS) ++diagnostics.rateMatches;
            else diagnostics.rateMatches = 0;
        }
        previous_utc_ms = ms; have_previous_utc = true;
    }
    return true;
}
static bool vtg(char **f, unsigned count, uint32_t tick)
{
    if (count < 9) return false;
    bool mode_valid = true;
    if (count > 9) {
        if (strlen(f[9]) != 1 || strchr("ADEMN", *f[9]) == NULL) return false;
        mode_valid = *f[9] == 'A' || *f[9] == 'D';
    }
    /* NEO-6 can emit $GPVTG,,,,,,,,,N*30 without a navigation solution.
     * Accept that explicit unavailable report, but keep unit validation for
     * populated sentences and never expose it as valid velocity. */
    bool empty_no_fix = count == 10 && !strcmp(f[9], "N");
    for (unsigned i = 1; i <= 8; ++i)
        if (*f[i]) empty_no_fix = false;
    if (!empty_no_fix && (strcmp(f[2], "T") || strcmp(f[4], "M") ||
        strcmp(f[6], "N") || strcmp(f[8], "K"))) return false;
    float speed = 0, course = 0;
    bool speed_ok = false, course_ok = false;
    double v;
    if (*f[1]) {
        if (!number(f[1], &v) || v < 0 || v >= 360) return false;
        course = (float)v; course_ok = mode_valid;
    }
    if (*f[7]) {
        if (!number(f[7], &v) || v < 0 || v > 10000) return false;
        speed = (float)(v/3.6); speed_ok = mode_valid;
    } else if (*f[5]) {
        if (!number(f[5], &v) || v < 0 || v > 10000) return false;
        speed = (float)(v*1852.0/3600.0); speed_ok = mode_valid;
    }
    latest.speed = speed; latest.course = course;
    latest.speedValid = speed_ok; latest.courseValid = course_ok;
    latest.vtgValid = mode_valid; latest.vtgReceivedMs = tick;
    have_vtg = true; ++diagnostics.goodVtg;
    return true;
}

/* Query replies are accepted only during the matching outstanding request. */
static bool pair_reply(char **f, unsigned count)
{
    unsigned id, value;
    if (diagnostics.configState != GNSS_CONFIG_BUSY || !waiting || config_step >= 4) return true;
    if (!strcmp(f[0], "$PAIR001")) {
        if (count != 3 || !integer(f[1], 999, &id) || !integer(f[2], 255, &value)) return false;
        if (id == (setting_sentences ? 62U : query_ids[config_step])) {
            /* 1 = processing; 0 = success; other results are errors. */
            if (value == 0) ack = 1;
            else if (value != 1) ack = 2;
        }
        return true;
    }
    if (setting_sentences) return true;
    if (config_step == 0 && !strcmp(f[0], "$PAIR865")) {
        if (count != 2 || !integer(f[1], 1000000, &value)) return false;
        diagnostics.configuredBaud = value;
        diagnostics.baudVerified = value == GNSS_EXPECTED_BAUD;
        query_reply = true;
    } else if (config_step == 1 && !strcmp(f[0], "$PAIR051")) {
        if (count != 2 || !integer(f[1], 65535, &value)) return false;
        diagnostics.configuredPeriodMs = value;
        diagnostics.rateVerified = value == GNSS_EXPECTED_PERIOD_MS;
        query_reply = true;
    } else if (config_step == 2 && !strcmp(f[0], "$PAIR063")) {
        if (count != 3 || !integer(f[1], 9, &id) || !integer(f[2], 20, &value)) return false;
        diagnostics.messageRates[id] = (uint8_t)value;
        diagnostics.messageReplyMask |= (uint16_t)(1U << id);
        query_reply = diagnostics.messageReplyMask == 0x3ffU;
        if (query_reply) {
            bool matches = true;
            for (unsigned i=0; i<10; ++i)
                if (diagnostics.messageRates[i] != ((i == 0 || i == 5) ? 1 : 0)) matches = false;
            diagnostics.sentencesVerified = matches;
        }
    } else if (config_step == 3 && !strcmp(f[0], "$PAIR067")) {
        if (count != 7) return false;
        uint8_t mask = 0;
        for (unsigned i=0; i<6; ++i) {
            if (!integer(f[i+1], 1, &value)) return false;
            mask |= (uint8_t)(value << i);
        }
        diagnostics.constellationMask = mask;
        /* GPS/Galileo/NavIC required; QZSS may be enabled by firmware default. */
        diagnostics.constellationsVerified = (mask & 0x2fU) == 0x25U;
        query_reply = true;
    }
    return true;
}
static int hex(char c)
{
    if (c >= '0' && c <= '9') return c-'0';
    if (c >= 'A' && c <= 'F') return c-'A'+10;
    if (c >= 'a' && c <= 'f') return c-'a'+10;
    return -1;
}
static void sentence(uint32_t tick)
{
    line[line_length] = 0;
    char *star = strchr(line, '*');
    if (!star || strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) {
        ++diagnostics.malformed; return;
    }
    uint8_t sum = 0;
    for (char *p=line+1; p<star; ++p) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)(hex(star[1])*16+hex(star[2]))) { ++diagnostics.badChecksum; return; }
    *star = 0;
    char *f[24]; unsigned count = 1; f[0] = line;
    for (char *p=line; *p; ++p) if (*p == ',') {
        if (count == 24) { ++diagnostics.malformed; return; }
        *p = 0; f[count++] = p+1;
    }
    if (!strncmp(f[0], "$PAIR", 5)) {
        if (!pair_reply(f, count)) ++diagnostics.malformed;
        return;
    }
    if (strlen(f[0]) != 6 || line[1] < 'A' || line[1] > 'Z' || line[2] < 'A' || line[2] > 'Z') {
        ++diagnostics.ignored; return;
    }
    bool ok;
    if (!strcmp(line+3, "GGA")) {
        ok = gga(f, count, tick);
        if (ok && !strncmp(line, "$GN", 3)) diagnostics.seenGnGga = true;
    } else if (!strcmp(line+3, "VTG")) {
        ok = vtg(f, count, tick);
        if (ok && !strncmp(line, "$GN", 3)) diagnostics.seenGnVtg = true;
    } else { ++diagnostics.ignored; return; }
    if (!ok) ++diagnostics.malformed;
}
static void feed(uint8_t b, uint32_t tick)
{
    if (line_length && (uint32_t)(tick-parser_tick) > 1500U) reset_parser();
    parser_tick = tick;
    if (b == '$') { line_length = 1; line[0] = '$'; return; }
    if (!line_length) return;
    if (b == '\r') return;
    if (b == '\n') { sentence(tick); line_length = 0; return; }
    if (b < 32 || b > 126 || line_length >= LINE_SIZE-1) {
        ++diagnostics.malformed; reset_parser(); return;
    }
    line[line_length++] = (char)b;
}
static bool begin_verification(bool set_sentences)
{
    if (!port || !diagnostics.receiving || tx_busy || diagnostics.configState == GNSS_CONFIG_BUSY) return false;
    config_step = 0; waiting = false;
    setting_sentences = set_sentences; setting_index = 0;
    diagnostics.configState = GNSS_CONFIG_BUSY; diagnostics.configStep = 0;
    diagnostics.baudVerified = diagnostics.rateVerified = diagnostics.sentencesVerified = diagnostics.constellationsVerified = false;
    diagnostics.configuredBaud = diagnostics.configuredPeriodMs = 0;
    diagnostics.messageReplyMask = 0; diagnostics.constellationMask = 0;
    for (unsigned i=0; i<10; ++i) diagnostics.messageRates[i] = 0;
    diagnostics.rateMatches = 0; diagnostics.observedPeriodMs = 0;
    diagnostics.seenGnGga = diagnostics.seenGnVtg = false;
    have_previous_utc = false;
    return true;
}
bool GNSS_VerifyConfiguration(void) { return begin_verification(false); }
bool GNSS_Configure(uint16_t periodMs, uint32_t targetBaud)
{
    return periodMs == GNSS_EXPECTED_PERIOD_MS && targetBaud == GNSS_EXPECTED_BAUD && GNSS_VerifyConfiguration();
}
static void configure_step(void)
{
    if (diagnostics.configState != GNSS_CONFIG_BUSY) return;
    uint32_t now = HAL_GetTick();
    if (setting_sentences) {
        /* PAIR062 applies only the NMEA mask, once at initialization.
         * IDs 0..8 are documented; query also checks firmware-specific ID 9. */
        diagnostics.configStep = (uint8_t)(10U + setting_index);
        if (waiting) {
            if (ack == 2) { diagnostics.configState = GNSS_CONFIG_NAK; return; }
            if (!tx_busy && ack == 1) {
                waiting = false;
                if (++setting_index == 9) {
                    setting_sentences = false; diagnostics.configStep = 0;
                }
            } else {
                if ((uint32_t)(now-command_started_ms) >= CONFIG_TIMEOUT_MS) diagnostics.configState = GNSS_CONFIG_TIMEOUT;
                return;
            }
        }
        if (setting_sentences) {
            char body[24];
            snprintf(body,sizeof(body),"PAIR062,%u,%u",(unsigned)setting_index,
                (setting_index == 0 || setting_index == 5) ? 1U : 0U);
            uint8_t checksum=0;
            for (const char *p=body; *p; ++p) checksum ^= (uint8_t)*p;
            int length=snprintf((char *)tx_frame,sizeof(tx_frame),"$%s*%02X\r\n",body,(unsigned)checksum);
            ack=0; waiting=true; tx_busy=true; command_started_ms=now;
            if (HAL_UART_Transmit_IT(port,tx_frame,(uint16_t)length) != HAL_OK) {
                tx_busy=waiting=false; diagnostics.configState=GNSS_CONFIG_IO_ERROR;
            }
            return;
        }
    }
    if (config_step == 4) {
        if (diagnostics.seenGnGga && diagnostics.seenGnVtg && diagnostics.rateMatches >= 5U &&
            (uint32_t)(now-latest.ggaReceivedMs) < GNSS_FRESH_MS &&
            (uint32_t)(now-latest.vtgReceivedMs) < GNSS_FRESH_MS) diagnostics.configState = GNSS_CONFIG_DONE;
        else if ((uint32_t)(now-observation_started_ms) >= 3000U) diagnostics.configState = GNSS_CONFIG_TIMEOUT;
        return;
    }
    if (waiting) {
        if (ack == 2) { diagnostics.configState = GNSS_CONFIG_NAK; return; }
        if (!tx_busy && ack == 1 && query_reply) {
            bool match = config_step == 0 ? diagnostics.baudVerified : config_step == 1 ? diagnostics.rateVerified :
                config_step == 2 ? diagnostics.sentencesVerified : diagnostics.constellationsVerified;
            if (!match) { diagnostics.configState = GNSS_CONFIG_MISMATCH; return; }
            waiting = false; ++config_step; diagnostics.configStep = config_step;
            if (config_step == 4) { observation_started_ms = now; return; }
        } else {
            if ((uint32_t)(now-command_started_ms) >= CONFIG_TIMEOUT_MS) diagnostics.configState = GNSS_CONFIG_TIMEOUT;
            return;
        }
    }
    static const char *const queries[] = {"PAIR865,0,0", "PAIR051", "PAIR063,-1", "PAIR067"};
    const char *body = queries[config_step]; uint8_t checksum = 0;
    for (const char *p=body; *p; ++p) checksum ^= (uint8_t)*p;
    int length = snprintf((char *)tx_frame, sizeof(tx_frame), "$%s*%02X\r\n", body, (unsigned)checksum);
    ack = 0; query_reply = false; waiting = true; tx_busy = true; command_started_ms = now;
    if (HAL_UART_Transmit_IT(port, tx_frame, (uint16_t)length) != HAL_OK) {
        tx_busy = waiting = false; diagnostics.configState = GNSS_CONFIG_IO_ERROR;
    }
}
static bool discard_lost_stream(void)
{
    if (seen_loss == loss_epoch) return false;
    uint32_t p = lock();
    tail = head; seen_loss = loss_epoch;
    unlock(p);
    invalidate(); ++diagnostics.streamResets;
    if (diagnostics.configState == GNSS_CONFIG_BUSY)
        diagnostics.configState = GNSS_CONFIG_IO_ERROR;
    return true;
}

void GNSS_Process(void)
{
    if (port == NULL) return;
    if (!discard_lost_stream()) {
        uint32_t end = head;
        __DMB();
        unsigned budget = GNSS_PROCESS_BUDGET;
        while (tail != end && budget--) {
            uint32_t read = tail;
            uint8_t b = queue[read&QUEUE_MASK]; uint32_t tick = arrival[read&QUEUE_MASK];
            __DMB(); tail = read+1;
            feed(b, tick);
            if (seen_loss != loss_epoch) { (void)discard_lost_stream(); break; }
        }
    }
    configure_step();
}

bool GNSS_GetData(GNSSData *data)
{
    if (data == NULL) return false;
    *data = latest;
    uint32_t now = HAL_GetTick();
    data->valid = diagnostics.receiving && seen_loss == loss_epoch && have_gga &&
                  (uint32_t)(now-data->ggaReceivedMs) < GNSS_FRESH_MS;
    data->fixValid &= data->valid;
    data->utcValid &= data->valid;
    data->altitudeValid &= data->fixValid;
    data->hdopValid &= data->fixValid;
    data->vtgValid &= data->fixValid && have_vtg && (uint32_t)(now-data->vtgReceivedMs) < GNSS_FRESH_MS;
    data->speedValid &= data->vtgValid; data->courseValid &= data->vtgValid;
    return data->valid;
}
bool GNSS_IsDataReady(void) { GNSSData data; return GNSS_GetData(&data); }
bool GNSS_IsFixValid(void) { GNSSData data; return GNSS_GetData(&data) && data.fixValid; }
void GNSS_GetDiagnostics(GNSSDiagnostics *out)
{
    if (out == NULL) return;
    uint32_t p = lock();
    *out = diagnostics; out->queuedBytes = head-tail;
    unlock(p);
}
