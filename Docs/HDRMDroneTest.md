# HDRM drone test

For the separate always-on telemetry variant with automatic startup calibration
and **no telecommands**, use [HDRMDroneNoCommands.md](HDRMDroneNoCommands.md).

`Tests/HDRM_drone_test.c` reuses the complete telecommand bench harness and adds a
one-shot HDRM pulse. Tune the macros at the top of that file, in milliseconds:

- `HDRM_FIRE`: delay after initialization finishes, default 30000 ms.
- `HDRM_ON`: duration measured from actual HDRM activation, default 2000 ms.

```powershell
cmake --preset HDRMDroneTest
cmake --build --preset HDRMDroneTest
```

Flash `build/HDRMDroneTest/HDRM_drone_test.elf`. The timer starts automatically
after startup/ready messages. HDRM is held low during SD/sensor initialization,
then switches on once after HDRM_FIRE, and off after HDRM_ON. A reset restarts
the entire timer and permits another pulse; this one-shot is RAM-only.

Acquisition, CSV logging, telemetry and the three XBee ground commands run as in
[TelecommandTest.md](TelecommandTest.md). Telemetry initially stays OFF until
`A5 01 5A`; `A5 02 5A` turns telemetry off without affecting the HDRM timer or SD
logging. `A5 03 5A` is a no-op. Startup performs ground/gyro calibration and flash
save on LAUNCH_PAD, or restores the existing pair on airborne/touchdown recovery.
Calibration failure prevents arming the HDRM timer.
The test keeps the bench mission context at LAUNCH_PAD and does not run flight
transitions, camera actions or guidance. Perform calibration while stationary.

USART3 at 115200 baud reports the armed delay, actual HDRM ON and OFF times,
actual pulse duration, all ground-command events and normal task status. The
existing console commands are `status`, `stream on`, `stream off`. HDRM events
remain visible with streaming off. `hdrm_drone_phase`, `hdrm_drone_started_ms`,
`hdrm_drone_fired_ms` and `hdrm_drone_off_ms` are available in the debugger.

After the pulse, `actuators.parafoilDeployed` becomes true in telemetry/CSV to
indicate commanded pulse completion, not physical deployment feedback. Sampling,
logging and enabled telemetry continue after firing; the pulse does not repeat.

Timing uses wrap-safe HAL tick checks before and after each loop. It is **nominal
software timing**, not a hardware-enforced pulse limit: synchronous sensor/SD
operations or console writes can delay firing or extend ON time. Calibration
finishes before arming. The console reports actual ON duration for verification.
