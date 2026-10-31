#include "apps/CameraControl.h"
#include "drivers/Switches.h"
#include "main.h"

void CameraControl_Init(CameraState *camera)
{
    if (!camera || camera->phase != CAMERA_OFF || camera->isOn) return;
    Switch_Capture(false);
    Switches_DVR(true);
    camera->isOn = true;
    camera->isRecording = false;
    camera->phase = CAMERA_INIT_HIGH;
    camera->phaseStartedMs = HAL_GetTick();
}

void CameraControl_Start(CameraState *camera)
{
    if (!camera) return;
    if (camera->phase == CAMERA_OFF) {
        CameraControl_Init(camera);
        return;
    }
    if (camera->phase != CAMERA_INIT_WAIT ||
        (uint32_t)(HAL_GetTick() - camera->phaseStartedMs) < CAMERA_INIT_DELAY_MS) return;
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
    if (camera->phase == CAMERA_INIT_HIGH &&
        (uint32_t)(now - camera->phaseStartedMs) >= CAMERA_INIT_PULSE_MS) {
        Switches_DVR(false);
        camera->isOn = false;
        camera->phase = CAMERA_INIT_WAIT;
        camera->phaseStartedMs = now; /* Full delay after the actual falling edge. */
    } else if (camera->phase == CAMERA_INIT_WAIT) {
        CameraControl_Start(camera);
    } else if (camera->phase == CAMERA_CAPTURE_HIGH &&
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
