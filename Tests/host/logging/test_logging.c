#include "apps/Logging.h"
#include "apps/Telemetry.h"
#include "apps/Cansat.h"
#include "drivers/SDCard.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool mountOK = true, writeOK = true, syncOK = true, closeOK = true;
static FRESULT createResult = FR_OK, lastResult;
static unsigned collisions, creates, writes, syncs, closes;
static char lastWrite[1024], createdName[32];
bool SDCard_Init(void) { return mountOK; }
bool SDCard_CreateFile(const char *name)
{
    ++creates;
    strcpy(createdName, name);
    lastResult = collisions ? FR_EXIST : createResult;
    if (collisions) --collisions;
    return lastResult == FR_OK;
}
FRESULT SDCard_GetLastResult(void) { return lastResult; }
bool SDCard_Write(const uint8_t *data, uint32_t length)
{
    ++writes;
    assert(length < sizeof(lastWrite) && memchr(data, 0, length) == NULL);
    memcpy(lastWrite, data, length); lastWrite[length] = 0;
    return writeOK;
}
bool SDCard_Sync(void) { ++syncs; return syncOK; }
bool SDCard_CloseFile(void) { ++closes; return closeOK; }

static void compare_fields(const Cansat *c)
{
    static const unsigned widths[] = {27,3,2,2,3,2,2,1,1,2,3,3,1,1,2,1,
        2,2,2,2,2,2,1,1,3,2,2,2,2,2,2,2,2,2,2,2,2,1,2,2,1,1,1,1,2};
    static const bool signedField[45] = {[5]=true,[16]=true,[17]=true,[18]=true,
        [19]=true,[20]=true,[21]=true,[25]=true,[26]=true,[27]=true,[30]=true,
        [31]=true,[32]=true,[33]=true,[34]=true,[35]=true,[36]=true,[39]=true};
    uint8_t packet[153];
    char csv[TELEMETRY_CSV_CAPACITY];
    assert(Telemetry_Encode(c, packet, sizeof(packet)) == 153);
    size_t length = Telemetry_FormatCSV(c, csv, sizeof(csv));
    assert(length && csv[length-2] == '\r' && csv[length-1] == '\n');
    size_t offset = 0;
    char *start = csv;
    for (unsigned i = 0; i < 45; ++i) {
        char *end = strchr(start, i == 44 ? '\r' : ',');
        assert(end); *end = 0;
        if (i == 0) assert(!strcmp(start, c->telemetry.teamId));
        else {
            uint32_t raw = 0;
            for (unsigned b = 0; b < widths[i]; ++b) raw |= (uint32_t)packet[offset+b] << (8*b);
            if ((i == 12 || i == 13) && raw != 255) {
                assert(start[0] == (char)raw && start[1] == 0);
            } else {
                long expected = signedField[i] && raw >= 32768 ? (long)raw - 65536 : (long)raw;
                char *tail;
                assert(strtol(start, &tail, 10) == expected && *tail == 0);
            }
        }
        offset += widths[i] + 1;
        start = end + 1;
    }
    assert(offset == 153);
}

static void test_csv(void)
{
    Cansat c;
    Cansat_Init(&c);
    compare_fields(&c); /* Every unavailable measurement retains its sentinel. */
    c.telemetry.packetCount = 65535;
    c.bmp = (BMP280Data){.valid=true,.temperature=-12.5f,.pressure=101325};
    c.bmi = (BMI270Data){.valid=true,.accelX=72.25f,.accelY=-72.25f,.gyroX=1.25f};
    c.mag = (IIS2MDCData){.valid=true,.magX=-20};
    c.gnss = (GNSSData){.valid=true,.fixValid=true,.latitude=-12.3456,
        .longitude=76.5432,.latitudeHemisphere='S',.longitudeHemisphere='E'};
    compare_fields(&c);
    Cansat before = c;
    char csv[TELEMETRY_CSV_CAPACITY];
    size_t length = Telemetry_FormatCSV(&c, csv, sizeof(csv));
    assert(strstr(csv, ",7225,-7225,") && strstr(csv, ",-1250,"));
    assert(!memcmp(&before, &c, sizeof(c)));
    for (size_t cap = 1; cap <= length; ++cap) {
        memset(csv, '!', sizeof(csv));
        assert(!Telemetry_FormatCSV(&c, csv, cap) && csv[0] == 0 && csv[cap] == '!');
    }
    assert(Telemetry_FormatCSV(&c, csv, length+1) == length);
    assert(!Telemetry_FormatCSV(NULL, csv, sizeof(csv)) && !csv[0]);
    assert(!Telemetry_FormatCSV(&c, NULL, 0));
    const char *header = Telemetry_CSVHeader();
    unsigned commas = 0;
    for (const char *p = header; *p; ++p) commas += *p == ',';
    assert(commas == 44 && !strcmp(header + strlen(header)-2, "\r\n"));
}

static void test_files(void)
{
    Cansat c; Cansat_Init(&c);
    assert(!Logging_IsReady() && !Logging_Write(&c));
    mountOK = false; assert(!Logging_Init() && creates == 0); mountOK = true;
    collisions = 2;
    assert(Logging_Init() && Logging_IsReady());
    assert(creates == 3 && !strcmp(createdName, "0:/LOG0002.CSV"));
    assert(!strcmp(Logging_GetFilename(), createdName));
    assert(writes == 1 && syncs == 1 && !strcmp(lastWrite, Telemetry_CSVHeader()));
    assert(Logging_Init() && creates == 3 && writes == 1);
    char expected[TELEMETRY_CSV_CAPACITY];
    assert(Telemetry_FormatCSV(&c, expected, sizeof(expected)));
    assert(Logging_Write(&c) && !strcmp(lastWrite, expected) && writes == 2 && syncs == 2);
    assert(c.telemetry.packetCount == 0);
    assert(!Logging_Write(NULL) && Logging_IsReady() && writes == 2);
    writeOK = false;
    assert(!Logging_Write(&c) && !Logging_IsReady() && writes == 3 && syncs == 2);
    assert(!Logging_Write(&c) && !Logging_Init() && writes == 3);
    closeOK = false; assert(!Logging_Stop()); closeOK = true; assert(Logging_Stop());
    writeOK = true;
    syncOK = false; assert(!Logging_Init() && !Logging_IsReady()); /* Header sync failure. */
    assert(Logging_Stop()); syncOK = true;
    assert(Logging_Init()); syncOK = false;
    assert(!Logging_Write(&c) && !Logging_IsReady()); /* Row sync failure. */
    assert(Logging_Stop()); syncOK = true;
    writeOK = false; assert(!Logging_Init()); assert(Logging_Stop()); writeOK = true;
    createResult = FR_DISK_ERR;
    unsigned previous = creates;
    assert(!Logging_Init() && creates == previous+1 && !Logging_GetFilename()[0]);
    createResult = FR_OK; collisions = 10000; previous = creates;
    assert(!Logging_Init() && creates == previous+10000 && !Logging_GetFilename()[0]);
    assert(Logging_Stop());
}

int main(void)
{
    test_csv(); test_files();
    puts("Logging tests passed: CSV/binary field equality, sentinels, bounds, file lifecycle and SD failures");
    return 0;
}
