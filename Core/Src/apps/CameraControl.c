#include "apps/CameraControl.h"
#include "drivers/Switches.h"
#include "main.h"

void CameraControl_Start(CameraState *camera)
{
    if (!camera || camera->isOn) return;
    Switches_DVR(true);
    camera->isOn = true;
    Switch_Capture(true);
    camera->lastCaptureHighMs = HAL_GetTick();
    camera->isRecording = false;
}
void CameraControl_Update(CameraState *camera)
{
    if (camera && camera->isOn && !camera->isRecording &&
        (uint32_t)(HAL_GetTick() - camera->lastCaptureHighMs) >= CAMERA_START_PULSE_MS) {
        Switch_Capture(false);
        camera->isRecording = true;
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
}
