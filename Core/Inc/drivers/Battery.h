#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

/* VREF+/VDDA, not the nominal battery voltage. Adjust to measured board value. */
#ifndef BATTERY_ADC_REFERENCE_V
#define BATTERY_ADC_REFERENCE_V 3.3f
#endif

/* (Rtop + Rbottom) / Rbottom; ADC input is battery voltage / 4. */
#ifndef BATTERY_DIVIDER_RATIO
#define BATTERY_DIVIDER_RATIO 4.0f
#endif

typedef struct
{
    uint16_t adcRaw;
    float voltage; /* Reconstructed battery voltage in volts. */

    bool valid; /* Successful conversion, not a battery-health indication. */

} BatteryData;

/* Call after MX_ADC1_Init. ADC1 is dedicated to PC2 / channel 12. */
bool Battery_Init(void);

/* One synchronous conversion. Poll timeout is 2 ms (HAL tick granularity).
 * Call from main/thread context with interrupts and the HAL tick running.
 * Returns false on NULL, missing Init or HAL failure. Non-NULL output is
 * cleared on failure so stale voltage cannot be mistaken for a new sample.
 * No filtering, state-of-charge estimate, or low-voltage cutoff is applied. */
bool Battery_Read(BatteryData *data);
#endif
