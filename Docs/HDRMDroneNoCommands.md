# HDRM drone test without telecommands

Use `Tests/HDRM_drone_no_commands_test.c`, based on the acquisition/logging/telemetry
harness. The earlier `HDRM_drone_test.c` remains a separate telecommand variant.

```powershell
cmake --preset HDRMDroneNoCommands
cmake --build --preset HDRMDroneNoCommands
```

Flash `build/HDRMDroneNoCommands/HDRM_drone_no_commands_test.elf` and open USART3
at 115200 baud, 8N1. Keep the CanSat stationary until calibration SUCCESS appears.

Both servos are initialized, commanded to 90 degrees and PWM-enabled before
blocking SD/sensor startup. They hold that command throughout calibration, HDRM
firing and afterward, including after console `stop`. Telemetry/CSV contain both
90-degree commands. The console reports initialization/PWM-start failure if one
occurs. Servo supply power must remain connected; enabling PWM supplies the control
signal, not electrical power to the servos.

Startup explicitly calls `SensorAcquisition_StartCalibration`: reinitialize sensors,
calculate ground pressure, and start stationary gyro sampling. Acquisition completes
the gyro sampling and commits ground pressure/biases to flash. This test deliberately
recalibrates on every boot, even if saved calibration exists. A blank mission journal
gets its initial launch-pad record; existing mission records are preserved. Flight
state transitions, camera actions and command processing are not run.

After successful calibration, the HDRM countdown begins:

- `HDRM_FIRE`: default 30000 ms after calibration completion.
- `HDRM_ON`: default 2000 ms measured from actual HDRM activation.

Both macros are at the top of the test file. HDRM stays off throughout initialization
and calibration. Calibration/setup failure disables firing while acquisition,
logging and enabled telemetry continue. The pulse occurs once; acquisition/logging/
telemetry continue afterward. Pulse completion sets the telemetry parafoil-status
field to true, indicating commanded completion rather than physical feedback.

Telemetry is enabled automatically and scheduled every 500 ms; logging runs every
1000 ms and acquisition every 10 ms, using the existing task macros. Incoming XBee
bytes are drained and discarded. Neither `Communication_Update` nor the ground
command parser is called or linked into this firmware. No ground command is needed
to enable telemetry, and received ON/OFF/CALIBRATE packets have no effect.

The console shows calibration results, HDRM ON/OFF events and the harness's four
average task-time reports. `status`, `stream on`, `stream off` control timing reports.
Local console `stop` stops acquisition/logging/transmission and cancels HDRM, including
an active pulse; reset starts a fresh test. Stream off does not hide HDRM events.

The software timer is checked before and after each loop. Blocking sensor/SD work
can delay firing or extend the pulse; the console reports actual ON duration.
Reboot starts calibration again and then arms a new one-shot, so the stationary-
startup assumption applies to every reboot of this test firmware.

Host tests cover explicit startup calibration, calibration-success countdown gating,
calibration failure, unconditional scheduled TX, discarded OFF packets, pulse
boundaries, tick rollover and console stop. Firmware builds are checked; physical
radio, SD, calibration and HDRM timing still require board verification.
