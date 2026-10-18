#pragma once
#include "drivers/Camera.h"
#define CAMERA_START_PULSE_MS 1000U
void CameraControl_Start(CameraState *camera);
void CameraControl_Update(CameraState *camera);
void CameraControl_Stop(CameraState *camera);
