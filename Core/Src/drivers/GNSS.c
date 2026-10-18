#include "drivers/GNSS.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

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
/* UBX and NMEA share the wire. Consume binary frames without interpreting
 * payload '$'/newlines as NMEA. Keep only the small configuration payload. */
static uint8_t ubx_state, ubx_class, ubx_id, ubx_a, ubx_b, ubx_payload[20];
static uint16_t ubx_length, ubx_index;
static bool ubx_checksum_ok;
static uint8_t tx_frame[28], config_step, expected_id, ack;
static volatile bool tx_busy;
static volatile uint32_t tx_finished_ms;
static bool waiting, port_verified;
static volatile bool changing_baud;
static uint32_t command_started_ms, target_baud;
static uint16_t measurement_ms;

static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static bool valid_baud(uint32_t baud)
{
    return baud == 9600 || baud == 19200 || baud == 38400 || baud == 57600 || baud == 115200;
}
static void reset_parser(void) { line_length = 0; ubx_state = 0; }
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
    if (uart == NULL || uart->hdmarx == NULL || !valid_baud(currentBaud) ||
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
    tx_busy = waiting = changing_baud = false;
    loss_epoch = seen_loss = 0;
    uart->Init.BaudRate = currentBaud;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(uart) != HAL_OK) return false;
    diagnostics.baudRate = currentBaud;
    return GNSS_StartReception();
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
    if (diagnostics.configState == GNSS_CONFIG_BUSY && !changing_baud)
        diagnostics.configState = GNSS_CONFIG_IO_ERROR;
    /* No HAL restarts or parsing in this callback. Explicit main-loop recovery. */
}

void GNSS_TxCompleteCallback(UART_HandleTypeDef *uart)
{
    if (uart == port) { tx_finished_ms = HAL_GetTick(); tx_busy = false; }
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
    if (next.fixValid) {
        if (!next.utcValid || !coordinate(f[2], f[3], true, &next.latitude) ||
            !coordinate(f[4], f[5], false, &next.longitude)) return false;
        next.latitudeHemisphere = *f[3]; next.longitudeHemisphere = *f[5];
        double v;
        if (*f[8]) {
            if (!number(f[8], &v) || v < 0 || v > 999) return false;
            next.hdop = (float)v;
        }
        if (*f[9]) {
            if (strcmp(f[10], "M") || !number(f[9], &v) || fabs(v) > 100000) return false;
            next.altitude = (float)v; next.altitudeValid = true;
        }
    }
    next.valid = true; next.ggaReceivedMs = tick;
    latest = next; have_gga = true; ++diagnostics.goodGga;
    return true;
}
static bool vtg(char **f, unsigned count, uint32_t tick)
{
    if (count < 9 || strcmp(f[2], "T") || strcmp(f[4], "M") ||
        strcmp(f[6], "N") || strcmp(f[8], "K")) return false;
    bool mode_valid = true;
    if (count > 9) {
        if (strlen(f[9]) != 1 || strchr("ADEN", *f[9]) == NULL) return false;
        mode_valid = *f[9] == 'A' || *f[9] == 'D';
    }
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
    if (star == NULL || strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) {
        ++diagnostics.malformed; return;
    }
    uint8_t sum = 0;
    for (char *p = line+1; p < star; ++p) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)(hex(star[1])*16+hex(star[2]))) { ++diagnostics.badChecksum; return; }
    *star = 0;
    char *fields[20]; unsigned count = 1;
    fields[0] = line;
    for (char *p = line; *p; ++p) if (*p == ',') {
        if (count == 20) { ++diagnostics.malformed; return; }
        *p = 0; fields[count++] = p+1;
    }
    if (strlen(fields[0]) != 6 || line[1] < 'A' || line[1] > 'Z' || line[2] < 'A' || line[2] > 'Z') {
        ++diagnostics.ignored; return;
    }
    bool ok;
    if (!strcmp(line+3, "GGA")) ok = gga(fields, count, tick);
    else if (!strcmp(line+3, "VTG")) ok = vtg(fields, count, tick);
    else { ++diagnostics.ignored; return; }
    if (!ok) ++diagnostics.malformed;
}
static uint32_t u32(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void ubx_complete(void)
{
    if (diagnostics.configState != GNSS_CONFIG_BUSY || !waiting) return;
    if (ubx_class == 5 && ubx_length == 2 && ubx_payload[0] == 6 && ubx_payload[1] == expected_id) {
        if (ubx_id == 1) ack = 1;
        if (ubx_id == 0) ack = 2;
    }
    if (config_step == 14 && ubx_class == 6 && ubx_id == 0 && ubx_length == 20 &&
        ubx_payload[0] == 1 && u32(ubx_payload+8) == target_baud &&
        u32(ubx_payload+4) == 0x8d0 && (ubx_payload[12]&3U) == 3 && (ubx_payload[14]&3U) == 3)
        port_verified = true;
}
static void feed(uint8_t b, uint32_t tick)
{
    if ((line_length || ubx_state) && (uint32_t)(tick-parser_tick) > 1500U) reset_parser();
    parser_tick = tick;
    if (ubx_state) {
        if (ubx_state == 1) {
            if (b == 0x62) { ubx_state = 2; ubx_a = ubx_b = 0; return; }
            ubx_state = 0; /* reconsider this byte as a possible new start */
        } else {
            if (ubx_state >= 2 && ubx_state <= 6) { ubx_a += b; ubx_b += ubx_a; }
            switch (ubx_state) {
            case 2: ubx_class = b; ubx_state = 3; break;
            case 3: ubx_id = b; ubx_state = 4; break;
            case 4: ubx_length = b; ubx_state = 5; break;
            case 5:
                ubx_length |= (uint16_t)b << 8; ubx_index = 0;
                if (ubx_length > 512) { ++diagnostics.malformed; reset_parser(); }
                else ubx_state = ubx_length ? 6 : 7;
                break;
            case 6:
                if (ubx_index < sizeof(ubx_payload)) ubx_payload[ubx_index] = b;
                if (++ubx_index == ubx_length) ubx_state = 7;
                break;
            case 7: ubx_checksum_ok = b == ubx_a; ubx_state = 8; break;
            case 8:
                if (ubx_checksum_ok && b == ubx_b) ubx_complete();
                else ++diagnostics.badChecksum;
                ubx_state = 0; break;
            default: reset_parser(); break;
            }
            return;
        }
    }
    if (b == '$') { line_length = 1; line[0] = '$'; return; }
    if (!line_length) { if (b == 0xb5) ubx_state = 1; return; }
    if (b == '\r') return;
    if (b == '\n') { sentence(tick); line_length = 0; return; }
    if (b < 32 || b > 126 || line_length >= LINE_SIZE-1) {
        ++diagnostics.malformed; reset_parser(); return;
    }
    line[line_length++] = (char)b;
}

static bool send_ubx(uint8_t id, const uint8_t *payload, uint8_t length)
{
    tx_frame[0] = 0xb5; tx_frame[1] = 0x62; tx_frame[2] = 6; tx_frame[3] = id;
    tx_frame[4] = length; tx_frame[5] = 0;
    memcpy(tx_frame+6, payload, length);
    uint8_t a = 0, b = 0;
    for (unsigned i = 2; i < 6U+length; ++i) { a += tx_frame[i]; b += a; }
    tx_frame[6+length] = a; tx_frame[7+length] = b;
    ack = 0; port_verified = false; expected_id = id;
    command_started_ms = HAL_GetTick(); waiting = true; tx_busy = true;
    if (HAL_UART_Transmit_IT(port, tx_frame, length+8U) != HAL_OK) {
        tx_busy = false; waiting = false; diagnostics.configState = GNSS_CONFIG_IO_ERROR; return false;
    }
    return true;
}
static void configure_step(void)
{
    if (diagnostics.configState != GNSS_CONFIG_BUSY) return;
    uint32_t now = HAL_GetTick();
    if (waiting) {
        if (ack == 2) { diagnostics.configState = GNSS_CONFIG_NAK; return; }
        if (config_step == 13) {
            /* Port ACK may be at either baud. After physical TX completion,
             * allow receiver processing, change MCU baud and poll the port. */
            if (!tx_busy && (uint32_t)(now-tx_finished_ms) >= 100U) {
                diagnostics.receiving = false;
                if (HAL_UART_AbortReceive(port) != HAL_OK) { diagnostics.configState = GNSS_CONFIG_IO_ERROR; return; }
                port->Init.BaudRate = target_baud;
                if (HAL_UART_Init(port) != HAL_OK || !GNSS_StartReception()) {
                    diagnostics.configState = GNSS_CONFIG_IO_ERROR; return;
                }
                diagnostics.baudRate = target_baud;
                changing_baud = false; waiting = false; config_step = 14;
            }
        } else if (!tx_busy && ((config_step == 14 && port_verified) || (config_step != 14 && ack == 1))) {
            waiting = false; ++config_step;
            if (config_step == 13 && target_baud == diagnostics.baudRate) config_step = 15;
        }
        if (waiting && (uint32_t)(now-command_started_ms) >= CONFIG_TIMEOUT_MS) {
            diagnostics.configState = config_step >= 13 ? GNSS_CONFIG_BAUD_UNCONFIRMED : GNSS_CONFIG_TIMEOUT;
            changing_baud = false;
            return;
        }
        if (waiting) return;
    }
    diagnostics.configStep = config_step;
    uint8_t payload[20] = {0};
    if (config_step < 11) {
        payload[0] = 0xf0; payload[1] = config_step;
        payload[2] = (config_step == 0 || config_step == 5) ? 1 : 0;
        (void)send_ubx(1, payload, 3); /* current UART only */
    } else if (config_step == 11) {
        payload[0] = 1; /* CFG-INF: disable NMEA TXT, not a CFG-MSG sentence */
        (void)send_ubx(2, payload, 10);
    } else if (config_step == 12) {
        payload[0] = (uint8_t)measurement_ms; payload[1] = (uint8_t)(measurement_ms >> 8);
        payload[2] = 1; /* navRate=1; timeRef=UTC */
        (void)send_ubx(8, payload, 6);
    } else if (config_step == 13) {
        payload[0] = 1; payload[4] = 0xd0; payload[5] = 0x08; /* UART1, 8N1 */
        for (unsigned i = 0; i < 4; ++i) payload[8+i] = (uint8_t)(target_baud >> (8*i));
        payload[12] = payload[14] = 3; /* UBX + NMEA input/output */
        changing_baud = true;
        (void)send_ubx(0, payload, 20);
    } else if (config_step == 14) {
        payload[0] = 1; (void)send_ubx(0, payload, 1);
    } else {
        diagnostics.configState = GNSS_CONFIG_DONE;
    }
}

bool GNSS_Configure(uint16_t periodMs, uint32_t targetBaud)
{
    if (port == NULL || !diagnostics.receiving || tx_busy ||
        diagnostics.configState == GNSS_CONFIG_BUSY || !valid_baud(targetBaud) ||
        (periodMs != 1000 && periodMs != 500 && periodMs != 200)) return false;
    target_baud = targetBaud; measurement_ms = periodMs;
    config_step = 0; waiting = changing_baud = false;
    diagnostics.configState = GNSS_CONFIG_BUSY; diagnostics.configStep = 0;
    return true;
}

static bool discard_lost_stream(void)
{
    if (seen_loss == loss_epoch) return false;
    uint32_t p = lock();
    tail = head; seen_loss = loss_epoch;
    unlock(p);
    invalidate(); ++diagnostics.streamResets;
    if (diagnostics.configState == GNSS_CONFIG_BUSY && !changing_baud)
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
