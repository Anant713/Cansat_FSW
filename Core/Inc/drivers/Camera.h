#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>

/* Initialize standalone instances with CameraState state = {0}. */
typedef enum {
    /* INIT_HIGH/INIT_WAIT retained as reserved legacy values; startup Init now
     * performs the normal record/stop cycle synchronously. */
    CAMERA_OFF = 0, CAMERA_INIT_HIGH, CAMERA_INIT_WAIT,
    CAMERA_CAPTURE_HIGH, CAMERA_RECORDING
} CameraPhase;
typedef struct
{
    bool isOn;
    bool isRecording;
    /* Application writes HAL_GetTick() when Switch_Capture goes high.
     * Milliseconds since boot; initialized to 0 by Cansat_Init or {0}. */
    uint32_t lastCaptureHighMs;
    CameraPhase phase;
    uint32_t phaseStartedMs;
} CameraState;

#endif
