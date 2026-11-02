# Telecommand reception bench test

Build with:

```powershell
cmake --preset TestTelecommands
cmake --build --preset TestTelecommands
```

Flash `build/TestTelecommands/test_telecommands.elf`. Open the USART3 console at
115200 baud, 8N1. XBee uses UART4 at its configured 115200 baud. The test runs real
sensor acquisition every ACQUISITION_PERIOD_MS (10 ms), CSV logging every
LOGGING_PERIOD_MS (1000 ms), and enabled telemetry every
COMMUNICATION_TELEMETRY_PERIOD_MS (500 ms). Missed periods do not cause catch-up bursts.

Telemetry starts OFF. Send these three **binary bytes** from the ground XBee;
do not send an ASCII hex string or a console command:

| Bytes | Expected console result |
| --- | --- |
| A5 01 5A | RX command 1; SUCCESS, telemetry ON |
| A5 02 5A | RX command 2; SUCCESS, telemetry OFF; SD logging continues |
| A5 03 5A | RX command 3; ignored (temporarily disabled, no flash write) |

The test uses production Communication, Telemetry, SensorAcquisition and Logging.
An optional main-loop command observer reports every recognized packet, even
multiple packets in one read. It reports receipt and execution results. Unknown
IDs and malformed packets do not execute; XBee RX byte/error counters help diagnose
such traffic. The observer is unset in ordinary flight firmware.

Every five seconds, a compact status report shows telemetry enable, TX accepted/error
counts, SD row/error counts, last received/successful command IDs, latest command
success, calibration state, stored-reference availability, and XBee diagnostics.
USART3 commands `status`, `stream off`, `stream on` control these summaries.
Command events remain visible with streaming off. TX acceptance is not RF delivery;
verify the received 151-byte packets at the ground station as well.

The bench context stays LAUNCH_PAD and supplies the command handler's persistence
policy without calling StateMachine_Init/Update. No camera, HDRM or buzzer actions
run. An empty mission journal gets an initial launch-pad record; existing mission
records are preserved, including previously airborne/touchdown states. Invalid
mission storage rejects calibration. This is a ground bench test, not a full
mission-state recovery test.

Calibration uses actual sensors and appends real calibration records to sector 11.
Keep the board stationary at the intended ground reference. Startup checks the
stored mission: LAUNCH_PAD calibrates ground pressure and gyro, waiting for flash
save before starting normal tasks. Airborne/touchdown boots restore the saved pair.
Command 3 does nothing. Success means the pair was saved/restored; see
[Calibration.md](Calibration.md).

For a reboot check: complete calibration, note the displayed ground pressure, reset
the MCU without erasing sector 11. Launch-pad boots now measure a new pair;
airborne boots retain the same reference. Telemetry starts OFF again and logging
creates a new CSV file. The
restored gyro bias is applied inside BMI270_Read. Preserve sector 11 when reflashing.

Host verification: `Tests/host/run_communication_tests.ps1` exercises this harness
with the real command parser and mocked peripherals: split/batched commands,
ON/OFF scheduling, continued logging, calibration events and completion failures.
Radio, sensor calibration quality, SD media and flash retention need board testing.
# Diagnosing missing commands

Raw RX hex output is enabled by default. `rx off` disables it; `rx on` restores it.
`stream off` disables only the periodic report, leaving raw RX and command events
visible. Send `status` for a snapshot. Raw output shows up to 32 bytes per chunk
before command parsing; chunks can split a command or contain several commands.

For a binary telemetry-on command, expect `XBee RAW RX (3 bytes): A5 01 5A`
(possibly split across lines), followed by a successful command event. Sending
the text `A5 01 5A` instead produces ASCII bytes `41 35 20 30 31 20 35 41`, which
are visible but intentionally do not execute a command. Use the sender's hex/binary
send mode. Raw RX can be disabled for sustained traffic to reduce console overhead.

`status=3` is `XBEE_READY`, not an error and not proof of radio association or RF
delivery. `receiving=1` means the driver believes DMA reception is armed.
`RX bytes` counts bytes observed by the UART/DMA RX callback, regardless of command
validity; `queued` is the number left after the main loop drains the queue and can
normally be zero. `lastError` is the HAL UART error bitmask.

If RX bytes remains zero, check the signal path before changing the parser:

1. Verify radio UART baud matches MCU UART4 (currently 115200, 8N1). Check sender
   destination/network configuration and that the radio supplies the intended raw
   payload to the MCU UART; API-framed output is not decoded by this command parser.
2. Probe the PCB XBee DOUT and MCU **PA1 / UART4_RX** while transmitting. The MCU
   TX is **PA0 / UART4_TX**. A logic analyzer distinguishes no signal from wrong
   baud/framing. The driver cannot itself confirm an over-the-air reception.
3. In the debugger inspect `huart4.hdmarx->Instance->NDTR` before/after a small
   burst. The circular buffer starts at 256. If it changes but RX bytes stays zero,
   inspect `UART4_IRQHandler`, `HAL_UARTEx_RxEventCallback`, and
   `XBee_RxEventCallback`. A short command normally relies on the UART IDLE event;
   it does not fill half the DMA buffer. Avoid reading UART DR in the debugger,
   which can disturb reception.
4. If NDTR does not move, inspect UART RX wiring/configuration and DMA enable state.
   If RX bytes increases but Commands stays zero, use the raw dump to inspect framing.

Garbled characters in the USART3 console report are a separate observation; they
are not raw XBee bytes. Verify that console is 115200 8N1 and inspect its wiring and
signal quality. `saved=1 ground=0` also merits checking sensor startup errors:
saved calibration availability does not guarantee the BMP driver initialized and
successfully restored its ground reference.

