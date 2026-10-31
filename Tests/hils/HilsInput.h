#pragma once
#include <stdbool.h>
#include <stdint.h>

#define HILS_INPUT_MAX_AGE_MS 1000U
#define HILS_GROUND_SAMPLES 16U
#define HILS_GROUND_MIN_SPAN_MS 1400U
#define HILS_GROUND_MAX_SPREAD_PA 50.0f

typedef enum { HILS_NONE, HILS_PRESSURE, HILS_COMMAND, HILS_STATUS,
               HILS_NEW_MISSION, HILS_RESET } HilsMessage;
typedef struct {
    float pressurePa, temperatureC;
    uint32_t sequence, receivedMs;
    bool present;
} HilsSample;

void HilsInput_Init(void);
/* H1,P,sequence,pressurePa,tempC*CCCC; CCCC = CRC16-CCITT-FALSE of body.
 * Other bodies: H1,C,1..3 / H1,STATUS / H1,NEW / H1,RESET.
 * No CR/LF in this argument. Invalid/replayed frames have no effect. */
HilsMessage HilsInput_Parse(const char *line, uint32_t now, uint8_t *command);
const HilsSample *HilsInput_Latest(void);
bool HilsInput_Take(uint32_t now, HilsSample *sample);
bool HilsInput_GroundPressure(uint32_t now, float *pressure);
