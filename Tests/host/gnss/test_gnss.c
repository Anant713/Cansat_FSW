#include "drivers/GNSS.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart;
static uint8_t *rx;
static uint16_t rx_size, rx_pos;
static uint32_t tick, transmissions;
static uint8_t transmitted[28];
static uint16_t transmitted_size;
static HAL_StatusTypeDef tx_result;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *p) { CHECK(p == &uart); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p) { CHECK(p == &uart); rx = NULL; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *p) { CHECK(p == &uart); return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *p, uint8_t *data, uint16_t n)
{
    CHECK(p == &uart && n == GNSS_DMA_SIZE);
    rx = data; rx_size = n; rx_pos = 0; dma.counter = n;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *p, uint8_t *data, uint16_t n)
{
    CHECK(p == &uart && n <= sizeof(transmitted));
    if (tx_result != HAL_OK) return tx_result;
    memcpy(transmitted, data, n); transmitted_size = n; ++transmissions;
    return HAL_OK;
}
static void reset(void)
{
    tick = 0; transmissions = 0; tx_result = HAL_OK;
    dma.Init.Mode = DMA_CIRCULAR; dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma.Init.MemInc = DMA_MINC_ENABLE; dma.Init.PeriphInc = DMA_PINC_DISABLE;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart.hdmarx = &dma; uart.ErrorCode = 0;
    CHECK(GNSS_Init(&uart, 9600));
}
static GNSSDiagnostics stats(void) { GNSSDiagnostics d; GNSS_GetDiagnostics(&d); return d; }
static void input(const void *bytes, size_t n, bool idle)
{
    const uint8_t *p = bytes;
    CHECK(rx != NULL);
    for (size_t i = 0; i < n; ++i) {
        rx[rx_pos++] = p[i];
        if (rx_pos == rx_size) rx_pos = 0;
        dma.counter = rx_size-rx_pos;
        if (rx_pos == rx_size/2 || rx_pos == 0)
            HAL_UARTEx_RxEventCallback(&uart, rx_pos ? rx_pos : rx_size);
    }
    if (idle) HAL_UARTEx_RxEventCallback(&uart, rx_pos ? rx_pos : rx_size);
}
static size_t nmea(char *out, const char *body)
{
    uint8_t crc = 0;
    for (const char *p = body; *p; ++p) crc ^= (uint8_t)*p;
    int n = sprintf(out, "$%s*%02X\r\n", body, crc);
    return (size_t)n;
}
static void message(const char *body)
{
    char out[256]; size_t n = nmea(out, body); input(out, n, true); GNSS_Process();
}
static void ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint8_t n)
{
    uint8_t frame[64] = {0xb5,0x62,cls,id,n,0};
    memcpy(frame+6, payload, n);
    uint8_t a = 0,b = 0;
    for (unsigned i = 2; i < n+6U; ++i) { a += frame[i]; b += a; }
    frame[n+6] = a; frame[n+7] = b;
    input(frame, n+8U, true);
}
static const char *fix = "GPGGA,123519.25,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
static const char *velocity = "GPVTG,054.7,T,,M,005.5,N,010.2,K,A";
static void near(double x, double y) { CHECK(fabs(x-y) < 0.0001); }
static void test_parser(void)
{
    reset(); GNSSData data;
    CHECK(!GNSS_GetData(&data) && !data.valid);
    /* Every possible split, including the checksum and CR/LF boundary. */
    char frame[256]; size_t n = nmea(frame, fix);
    for (size_t split = 1; split < n; ++split) {
        CHECK(GNSS_StopReception()); CHECK(GNSS_StartReception());
        input("noise,partial\r\n", 15, true);
        input(frame, split, true); GNSS_Process(); CHECK(!GNSS_GetData(&data));
        input(frame+split, n-split, true); GNSS_Process();
        CHECK(GNSS_GetData(&data) && data.fixValid && data.satellites == 8);
        near(data.latitude, 48.1173); near(data.longitude, 11.5166666667);
        near(data.altitude, 545.4); CHECK(data.utcTime == 123519 && data.utcMilliseconds == 250);
    }
    message(velocity); CHECK(GNSS_GetData(&data) && data.speedValid && data.courseValid);
    near(data.speed, 10.2/3.6); near(data.course, 54.7);
    message("GNGGA,235960.125,3350.000,S,15112.000,W,2,12,1.0,-12.5,M,0,M,,");
    CHECK(GNSS_GetData(&data) && data.fixValid); near(data.latitude, -33.8333333333); near(data.longitude,-151.2);
    CHECK(data.utcTime == 235960); near(data.altitude,-12.5);
    message("GPGGA,123520.00,,,,,0,00,99.99,,,,,,");
    CHECK(GNSS_GetData(&data) && !data.fixValid && !data.altitudeValid && !data.speedValid && data.satellites == 0);
    message(fix); message("GPVTG,,T,,M,0.0,N,0.0,K,N");
    CHECK(GNSS_GetData(&data) && !data.vtgValid && !data.speedValid);
    message("GPVTG,,T,,M,1.0,N,,K,A");
    CHECK(GNSS_GetData(&data) && data.speedValid && !data.courseValid); near(data.speed,1852.0/3600.0);
    puts("NMEA: all sentence splits, units, hemispheres, leap second, no-fix and VTG validity passed");
}
static void test_bad_frames(void)
{
    reset(); GNSSData data;
    char frame[256]; size_t n = nmea(frame, fix);
    frame[20] ^= 1; input(frame,n,true); GNSS_Process();
    CHECK(!GNSS_GetData(&data) && stats().badChecksum == 1);
    message("GPGGA,123519.00,4867.038,N,01131.000,E,1,08,0.9,1,M,0,M,,");
    message("GPGGA,253519.00,4807.038,N,01131.000,E,1,08,0.9,1,M,0,M,,");
    message("GPGGA,123519.00,9000.001,N,01131.000,E,1,08,0.9,1,M,0,M,,");
    CHECK(!GNSS_GetData(&data) && stats().malformed == 3);
    message("GPRMC,irrelevant,fields"); CHECK(stats().ignored == 1);
    char overflow[180]; memset(overflow,'X',sizeof(overflow)); overflow[0]='$';
    input(overflow,sizeof(overflow),true); input("\n",1,true); GNSS_Process();
    input("$GPGGA,partial",14,true); GNSS_Process();
    message(fix); CHECK(GNSS_GetData(&data));
    CHECK(!GNSS_GetData(NULL));
    uint8_t other_payload[] = {'$', 'G', 'P', 'G', 'G', 'A', '\n'};
    uint32_t old = stats().goodGga;
    ubx(1,2,other_payload,sizeof(other_payload)); GNSS_Process();
    CHECK(stats().goodGga == old);
    puts("Parser recovery: corrupt checksum, malformed fields, overlength, noise and mixed UBX passed");
}
static void test_dma_and_freshness(void)
{
    reset(); GNSSData data;
    /* Begin a sentence just before the end of DMA so it straddles the wrap. */
    uint8_t filler[GNSS_DMA_SIZE-12]; memset(filler,'x',sizeof(filler));
    input(filler,sizeof(filler),true); GNSS_Process();
    message(fix); CHECK(GNSS_GetData(&data));
    uint32_t count = stats().receivedBytes;
    HAL_UARTEx_RxEventCallback(&uart, GNSS_DMA_SIZE/2); /* delayed historical event */
    HAL_UARTEx_RxEventCallback(&uart, GNSS_DMA_SIZE);
    CHECK(stats().receivedBytes == count);
    UART_HandleTypeDef other = {0}; HAL_UARTEx_RxEventCallback(&other, 20);
    HAL_UART_ErrorCallback(&other); CHECK(stats().uartErrors == 0);
    /* Multiple turns in the DMA buffer with main-loop parsing deferred. */
    char frame[256]; size_t n = nmea(frame,fix);
    for (unsigned i=0;i<20;++i) input(frame,n,true);
    for (unsigned i=0;i<3;++i) GNSS_Process();
    CHECK(stats().goodGga == 21 && stats().queueDrops == 0);
    tick += GNSS_FRESH_MS;
    CHECK(!GNSS_GetData(&data) && !data.fixValid && !data.utcValid);
    /* Age is RX time, not the later time the main loop parses the sentence. */
    input(frame,n,true); tick += GNSS_FRESH_MS+1; GNSS_Process(); CHECK(!GNSS_GetData(&data));
    tick = UINT32_MAX-500U; message(fix); tick += 1000;
    CHECK(GNSS_GetData(&data)); tick += 2000; CHECK(!GNSS_GetData(&data));
    message(fix);
    for (unsigned i=0;i<20;++i) input(filler,sizeof(filler),true);
    CHECK(stats().queueDrops > 0 && !GNSS_GetData(&data));
    GNSS_Process(); CHECK(stats().streamResets > 0 && !GNSS_GetData(&data));
    message(fix); CHECK(GNSS_GetData(&data));
    uart.ErrorCode = 8; HAL_UART_ErrorCallback(&uart);
    CHECK(!GNSS_GetData(&data) && stats().uartErrors == 1 && !stats().receiving);
    GNSS_Process(); CHECK(GNSS_StartReception()); message(fix); CHECK(GNSS_GetData(&data));
    puts("DMA: wrap, duplicate/reordered events, deferred parsing, overflow, stale RX and UART recovery passed");
}
static void ack_command(bool accepted)
{
    CHECK(transmitted[0] == 0xb5 && transmitted[1] == 0x62 && transmitted[2] == 6);
    uint8_t a=0,b=0;
    for (unsigned i=2;i<transmitted_size-2U;++i) { a+=transmitted[i]; b+=a; }
    CHECK(a == transmitted[transmitted_size-2] && b == transmitted[transmitted_size-1]);
    HAL_UART_TxCpltCallback(&uart);
    uint8_t payload[] = {6,transmitted[3]};
    ubx(5,accepted?1:0,payload,2);
    GNSS_Process();
}
static void settings_until_baud(uint32_t target)
{
    CHECK(GNSS_Configure(500,target)); GNSS_Process();
    for (unsigned i=0;i<11;++i) {
        CHECK(transmitted[3] == 1 && transmitted[4] == 3 && transmitted[6] == 0xf0);
        CHECK(transmitted[7] == i && transmitted[8] == ((i==0 || i==5)?1:0));
        ack_command(true);
    }
    CHECK(transmitted[3] == 2 && transmitted[4] == 10 && transmitted[6] == 1);
    ack_command(true);
    CHECK(transmitted[3] == 8 && transmitted[4] == 6);
    CHECK(transmitted[6] == 0xf4 && transmitted[7] == 1 && transmitted[8] == 1);
    ack_command(true);
}
static void test_configuration(void)
{
    reset(); CHECK(!GNSS_Configure(100,9600)); CHECK(!GNSS_Configure(500,12345));
    settings_until_baud(9600); CHECK(stats().configState == GNSS_CONFIG_DONE && transmissions == 13);
    reset(); settings_until_baud(38400);
    CHECK(transmitted[3] == 0 && transmitted[4] == 20 && transmitted[6] == 1);
    CHECK(transmitted[10] == 0xd0 && transmitted[11] == 8 && transmitted[14] == 0 && transmitted[15] == 0x96);
    HAL_UART_TxCpltCallback(&uart); tick += 99; GNSS_Process(); CHECK(uart.Init.BaudRate == 9600);
    tick += 1; GNSS_Process();
    CHECK(uart.Init.BaudRate == 38400 && transmitted[3] == 0 && transmitted[4] == 1);
    HAL_UART_TxCpltCallback(&uart);
    uint8_t config[20] = {1,0,0,0,0xd0,8,0,0,0,0x96,0,0,3,0,3,0,0,0,0,0};
    ubx(6,0,config,20); GNSS_Process(); CHECK(stats().configState == GNSS_CONFIG_DONE);
    reset(); CHECK(GNSS_Configure(1000,9600)); GNSS_Process(); ack_command(false);
    CHECK(stats().configState == GNSS_CONFIG_NAK);
    reset(); CHECK(GNSS_Configure(1000,9600)); GNSS_Process();
    HAL_UART_TxCpltCallback(&uart); tick += 2500; GNSS_Process(); CHECK(stats().configState == GNSS_CONFIG_TIMEOUT);
    reset(); settings_until_baud(38400); HAL_UART_TxCpltCallback(&uart);
    tick += 100; GNSS_Process(); HAL_UART_TxCpltCallback(&uart);
    tick += 2500; GNSS_Process(); CHECK(stats().configState == GNSS_CONFIG_BAUD_UNCONFIRMED);
    reset(); tx_result = HAL_BUSY; CHECK(GNSS_Configure(200,9600)); GNSS_Process();
    CHECK(stats().configState == GNSS_CONFIG_IO_ERROR);
    puts("UBX: sentence/rate payloads, checksums, ACK/NAK, baud readback and timeout handling passed");
}
int main(void)
{
    test_parser(); test_bad_frames(); test_dma_and_freshness(); test_configuration();
    puts("All GNSS host tests passed."); return 0;
}
