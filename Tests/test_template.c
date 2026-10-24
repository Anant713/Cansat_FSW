#include "App.h"
#include "Console.h"

static char command[CONSOLE_LINE_MAX + 1U];

/* Build scaffold only. This template does not perform a functional test.
 * Copy it for each subunit test and link only that test and its dependencies.
 * main.c has already initialized the CubeMX-configured peripherals.
 */
void App_Init(void)
{
    bool console_ok = Console_Init();
    Console_Write(console_ok ? "Console RX started. USART3: 115200 8N1, no flow control.\r\n" :
        "Console RX start failed; main loop will retry.\r\n");
    Console_Write("Select Newline or CRLF in the serial monitor. Send text to receive an echo.\r\n");
    /* TODO: Initialize only the driver/module under test and its dependencies.
     * Set up test inputs and result storage; check initialization results.
     */
    

}

void App_RunOnce(void)
{
    Console_Process();
    ConsoleLineResult input = Console_ReadLine(command, sizeof(command));
    if (input == CONSOLE_LINE_READY) Console_Printf("Received: %s\r\n", command);
    else if (input < 0) Console_Write("Input rejected/lost. Press Enter and resend.\r\n");
    /* TODO: Exercise one test step, compare actual and expected results,
     * and expose pass/fail results through debugger-visible variables or
     * an explicitly configured output interface.
     * Use timing conditions for periodic tests. For a one-shot test, retain
     * a completion flag and return without repeating the test once finished.
     */



}
