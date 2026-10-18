#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>

/* Initialize standalone instances with CameraState state = {0}. */
typedef struct
{
    bool isOn;
    bool isRecording;
    /* Application writes HAL_GetTick() when Switch_Capture goes high.
     * Milliseconds since boot; initialized to 0 by Cansat_Init or {0}. */
    uint32_t lastCaptureHighMs;
} CameraState;

#endif
