#include "App.h"
#include "main.h"
#include "apps/CameraControl.h"
#include "drivers/Switches.h"
#include "Console.h"
#include <string.h>

CameraState test_camera;
bool test_camera_complete;
volatile uint32_t test_camera_reset_flags;
bool test_camera_console_ok;
static char command[CONSOLE_LINE_MAX + 1U];

/* Shared by the automatic startup cycle and the console commands. */
static void recordCamera(void)
{
    test_camera_complete = false;
    CameraControl_Start(&test_camera);
}

static void stopCamera(void)
{
    CameraControl_Stop(&test_camera);
    test_camera_complete = true;
}

void App_Init(void)
{
    test_camera_reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    test_camera_console_ok = Console_Init();
    (void)Switches_Init();
    test_camera = (CameraState){0};
    test_camera_complete = false;
    Console_Printf("Boot reset flags: 0x%08lX; BOR=%u POR=%u PIN=%u SW=%u IWDG=%u WWDG=%u\r\n",
        (unsigned long)test_camera_reset_flags,
        (unsigned)((test_camera_reset_flags & RCC_CSR_BORRSTF) != 0),
        (unsigned)((test_camera_reset_flags & RCC_CSR_PORRSTF) != 0),
        (unsigned)((test_camera_reset_flags & RCC_CSR_PINRSTF) != 0),
        (unsigned)((test_camera_reset_flags & RCC_CSR_SFTRSTF) != 0),
        (unsigned)((test_camera_reset_flags & RCC_CSR_IWDGRSTF) != 0),
        (unsigned)((test_camera_reset_flags & RCC_CSR_WWDGRSTF) != 0));
    Console_Write("Camera startup: automatic record/stop cycle; wait for ready.\r\n");
    CameraControl_Init(&test_camera);
    test_camera_complete = true;
    Console_Write("Camera ready. Commands: record, stop, status (press Enter).\r\n");
}
void App_RunOnce(void)
{
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) {
        if (strcmp(command, "record") == 0) {
            if (test_camera.phase != CAMERA_OFF) {
                Console_Write(test_camera.isRecording ? "Already recording.\r\n" : "Recording startup in progress.\r\n");
            } else {
                recordCamera();
                Console_Printf("record received; DVR on, capture high for %lu ms.\r\n",
                    (unsigned long)CAMERA_START_PULSE_MS);
            }
        } else if (strcmp(command, "stop") == 0) {
            stopCamera();
            Console_Write("Recording stopped; DVR off.\r\n");
        } else if (strcmp(command, "status") == 0) {
            Console_Printf("Camera on=%u recording=%u phase=%u; RX bytes=%lu lines=%lu errors=%lu dropped=%lu\r\n",
                (unsigned)test_camera.isOn, (unsigned)test_camera.isRecording,
                (unsigned)test_camera.phase,
                (unsigned long)console_diagnostics.rx_received_bytes,
                (unsigned long)console_diagnostics.completed_lines,
                (unsigned long)console_diagnostics.rx_errors,
                (unsigned long)console_diagnostics.rx_dropped_bytes);
        } else {
            Console_Write("Unknown command. Use record, stop or status.\r\n");
        }
    } else if (input < 0) {
        Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    }
    bool wasRecording = test_camera.isRecording;
    CameraControl_Update(&test_camera);
    if (!wasRecording && test_camera.isRecording)
        Console_Write("Recording. Send stop to finish.\r\n");
}
