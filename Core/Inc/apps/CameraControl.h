#pragma once
#include "drivers/Camera.h"
#define CAMERA_START_PULSE_MS 1000U
#define CAMERA_INIT_PULSE_MS 1000U
#ifndef CAMERA_INIT_DELAY_MS
#define CAMERA_INIT_DELAY_MS 200U /* DVR off -> recording startup; tune on hardware. */
#endif
/* Nonblocking: DVR high 1 s, low, wait INIT_DELAY, then Start automatically.
 * Call Update every loop. Init/Start during an active sequence are idempotent.
 * Start from OFF also performs initialization. Stop cancels every phase. */
void CameraControl_Init(CameraState *camera);
void CameraControl_Start(CameraState *camera);
void CameraControl_Update(CameraState *camera);
void CameraControl_Stop(CameraState *camera);
