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
static uint8_t transmitted[40];
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
    CHECK(GNSS_Init(&uart, GNSS_EXPECTED_BAUD));
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
    /* Exact hardware capture: valid checksum, all fields empty, mode N.
     * It must clear previously valid velocity rather than count as malformed. */
    const char unavailable[] = "$GPVTG,,,,,,,,,N*30\r\n";
    uint32_t good_vtg = stats().goodVtg, malformed = stats().malformed;
    input(unavailable, sizeof(unavailable)-1, true); GNSS_Process();
    CHECK(stats().goodVtg == good_vtg+1 && stats().malformed == malformed);
    CHECK(GNSS_GetData(&data) && !data.vtgValid && !data.speedValid && !data.courseValid);
    near(data.speed, 0); near(data.course, 0);
    message("GPVTG,,,,,,,,,A"); /* missing units cannot claim valid navigation */
    message("GPVTG,054.7,X,,M,005.5,N,010.2,K,A");
    CHECK(stats().malformed == malformed+2 && stats().goodVtg == good_vtg+1);
    message(velocity);
    CHECK(GNSS_GetData(&data) && data.speedValid && data.courseValid);
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
    puts("Parser recovery: corrupt checksum, malformed fields, overlength and noise passed");
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
    tick = UINT32_MAX-500U; message(fix); tick += 100;
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

static void expect_query(const char *body)
{
    char frame[128]; size_t n = nmea(frame, body);
    CHECK(transmitted_size == n && !memcmp(transmitted, frame, n));
    HAL_UART_TxCpltCallback(&uart);
}
static void query_replies(bool wrong_messages)
{
    GNSS_Process();
    for (unsigned i=0; i<9; ++i) {
        char body[40]; snprintf(body,sizeof(body),"PAIR062,%u,%u",i,(i==0 || i==5)?1U:0U);
        expect_query(body); message("PAIR001,062,0");
    }
    expect_query("PAIR865,0,0");
    message("PAIR001,865,0"); message("PAIR865,115200");
    expect_query("PAIR051");
    message("PAIR051,100"); message("PAIR001,051,0"); /* reply before ACK also allowed */
    expect_query("PAIR063,-1");
    message("PAIR001,063,0");
    for (unsigned i=0; i<10; ++i) {
        char body[40];
        unsigned rate = (i==0 || i==(wrong_messages ? 4U : 5U)) ? 1U : 0U;
        snprintf(body,sizeof(body),"PAIR063,%u,%u",i,rate); message(body);
    }
    if (wrong_messages) return;
    expect_query("PAIR067");
    message("PAIR001,067,0"); message("PAIR067,1,0,1,0,1,1");
    CHECK(stats().configState == GNSS_CONFIG_BUSY && stats().configStep == 4);
}
static void gga_at(unsigned ms)
{
    char body[128];
    snprintf(body,sizeof(body),"GNGGA,123519.%03u,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",ms);
    message(body);
}
static void test_verification(void)
{
    reset(); CHECK(stats().configState==GNSS_CONFIG_BUSY);
    query_replies(false);
    for (unsigned i=0; i<6; ++i) { tick+=100; gga_at(i*100); message("GNVTG,054.7,T,,M,005.5,N,010.2,K,A"); }
    CHECK(stats().configState==GNSS_CONFIG_DONE && stats().rateMatches==5 && transmissions==13);
    CHECK(stats().sentencesVerified && stats().constellationsVerified && stats().observedPeriodMs==100);
    GNSSData d; CHECK(GNSS_GetData(&d) && d.speedValid); near(d.speed,10.2/3.6);
    message("GNVTG,,T,,M,,N,,K,N"); CHECK(GNSS_GetData(&d) && !d.speedValid);
    message("GNVTG,0,T,,M,0,N,0,K,M"); CHECK(GNSS_GetData(&d) && !d.vtgValid);
    message("GNRMC,ignored"); message("PQTMANTENNASTATUS,1,1,1"); CHECK(stats().ignored==2);
    CHECK(!GNSS_Configure(200,38400)); CHECK(GNSS_Configure(100,115200));
    GNSS_Process(); expect_query("PAIR865,0,0"); /* explicit recheck does not set mask again */
    reset(); query_replies(true); CHECK(stats().configState==GNSS_CONFIG_MISMATCH && !stats().sentencesVerified);
    reset(); GNSS_StopReception(); GNSS_StartReception(); CHECK(GNSS_VerifyConfiguration());
    GNSS_Process(); expect_query("PAIR865,0,0");
    message("PAIR001,865,0"); message("PAIR865,9600"); CHECK(stats().configState==GNSS_CONFIG_MISMATCH);
    reset(); GNSS_Process(); expect_query("PAIR062,0,1");
    message("PAIR001,062,2"); CHECK(stats().configState==GNSS_CONFIG_NAK);
    reset(); GNSS_StopReception(); GNSS_StartReception(); CHECK(GNSS_VerifyConfiguration());
    GNSS_Process(); expect_query("PAIR865,0,0");
    message("PAIR001,051,0"); message("PAIR051,100");
    tick+=2500; GNSS_Process(); CHECK(stats().configState==GNSS_CONFIG_TIMEOUT);
    reset(); query_replies(false); tick+=3000; GNSS_Process();
    CHECK(stats().configState==GNSS_CONFIG_TIMEOUT); /* no UTC evidence, never false DONE */
    reset(); tx_result=HAL_ERROR; GNSS_Process(); CHECK(stats().configState==GNSS_CONFIG_IO_ERROR);
    reset(); GNSS_Process(); uart.ErrorCode=8; HAL_UART_ErrorCallback(&uart);
    CHECK(stats().configState==GNSS_CONFIG_IO_ERROR);
    CHECK(GNSS_StartReception()); HAL_UART_TxCpltCallback(&uart); CHECK(GNSS_VerifyConfiguration());
    puts("Quectel: startup GGA/VTG mask, read-only recheck, ACK/reply order, 10Hz evidence and failures passed");
}
static void test_10hz_backlog(void)
{
    reset(); char frame[256];
    for (unsigned i=0; i<10; ++i) {
        tick+=100;
        char body[128];
        snprintf(body,sizeof(body),"GNGGA,123519.%03u,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",i*100);
        size_t n=nmea(frame,body); input(frame,n,true);
        n=nmea(frame,"GNVTG,054.7,T,,M,005.5,N,010.2,K,A"); input(frame,n,true);
    }
    GNSS_Process(); GNSS_Process();
    CHECK(stats().queueDrops==0 && stats().goodGga==10 && stats().goodVtg==10);
    GNSSData d; CHECK(GNSS_GetData(&d) && d.utcMilliseconds==900 && d.speedValid);
    puts("10Hz GNGGA/GNVTG: one-second deferred parsing across DMA wraps passed");
}
int main(void)
{
    test_parser(); test_bad_frames(); test_dma_and_freshness(); test_verification();
    test_10hz_backlog();
    puts("All L89HA GNSS host tests passed.");
    return 0;
}
