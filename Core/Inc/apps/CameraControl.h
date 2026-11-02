#pragma once
#include "drivers/Camera.h"
#define CAMERA_START_PULSE_MS 1000U
#ifndef CAMERA_INIT_RECORD_HOLD_MS
#define CAMERA_INIT_RECORD_HOLD_MS 500U
#endif
#ifndef CAMERA_INIT_STOP_SETTLE_MS
#define CAMERA_INIT_STOP_SETTLE_MS 500U
#endif
/* Startup only, blocking (~2 s): normal Start + full capture pulse, hold 500 ms,
 * Stop, settle 500 ms. Leaves camera OFF, ready for the first real Start.
 * Call after GPIO/switch initialization, before state recovery/entry actions. */
void CameraControl_Init(CameraState *camera);
/* Direct nonblocking recording start: DVR high + Capture high; Update lowers
 * Capture after CAMERA_START_PULSE_MS. Repeated Start does not restart the pulse. */
void CameraControl_Start(CameraState *camera);
void CameraControl_Update(CameraState *camera);
void CameraControl_Stop(CameraState *camera);
