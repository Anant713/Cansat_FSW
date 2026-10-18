#include "App.h"
#include "main.h"
#include "apps/CameraControl.h"
#include "drivers/Switches.h"
#include "Console.h"

CameraState test_camera;
bool test_camera_complete;
static bool recordingStarted;
static uint32_t recordingStart;

void App_Init(void)
{
    (void)Console_Init();
    (void)Switches_Init();
    test_camera = (CameraState){0};
    recordingStarted = test_camera_complete = false;
    CameraControl_Start(&test_camera);
    Console_Write("Camera powered; capture high for 1 second.\r\n");
}
void App_RunOnce(void)
{
    Console_Process();
    if (test_camera_complete) return;
    CameraControl_Update(&test_camera);
    if (test_camera.isRecording && !recordingStarted) {
        recordingStarted = true;
        recordingStart = HAL_GetTick();
        Console_Write("Recording for 10 seconds.\r\n");
    }
    if (recordingStarted && (uint32_t)(HAL_GetTick() - recordingStart) >= 10000U) {
        CameraControl_Stop(&test_camera);
        test_camera_complete = true;
        Console_Write("Capture high, DVR off; sequence complete. Check camera recording.\r\n");
    }
}
