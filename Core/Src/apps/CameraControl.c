#include "apps/CameraControl.h"
#include "drivers/Switches.h"
#include "main.h"

void CameraControl_Init(CameraState *camera)
{
    if (!camera || camera->phase != CAMERA_OFF || camera->isOn) return;
    CameraControl_Start(camera);
    HAL_Delay(CAMERA_START_PULSE_MS);
    CameraControl_Update(camera);
    HAL_Delay(CAMERA_INIT_RECORD_HOLD_MS);
    CameraControl_Stop(camera);
    HAL_Delay(CAMERA_INIT_STOP_SETTLE_MS);
}

void CameraControl_Start(CameraState *camera)
{
    if (!camera || camera->phase != CAMERA_OFF) return;
    Switches_DVR(true);
    camera->isOn = true;
    Switch_Capture(true);
    camera->lastCaptureHighMs = HAL_GetTick();
    camera->isRecording = false;
    camera->phase = CAMERA_CAPTURE_HIGH;
}
void CameraControl_Update(CameraState *camera)
{
    if (!camera) return;
    uint32_t now = HAL_GetTick();
    if (camera->phase == CAMERA_CAPTURE_HIGH &&
               (uint32_t)(now - camera->lastCaptureHighMs) >= CAMERA_START_PULSE_MS) {
        Switch_Capture(false);
        camera->isRecording = true;
        camera->phase = CAMERA_RECORDING;
    }
}
void CameraControl_Stop(CameraState *camera)
{
    if (!camera) return;
    Switch_Capture(true);
    camera->lastCaptureHighMs = HAL_GetTick();
    camera->isRecording = false;
    Switches_DVR(false);
    camera->isOn = false;
    camera->phase = CAMERA_OFF;
}
