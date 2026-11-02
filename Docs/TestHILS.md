# TestHILS: pressure-driven mission testing

## Alternate instance: telemetry ON by default

Use **TestHILSTelemetryOn** to run HILS without needing any ground commands:

```powershell
cmake --preset TestHILSTelemetryOn
cmake --build --preset TestHILSTelemetryOn
```

Flash `build/TestHILSTelemetryOn/testHILS_telemetry_on.elf`.
Its entry point is `Tests/testHILS_telemetry_on.c`; it reuses the same HILS harness
and Python GUI. Telemetry starts enabled after XBee initialization and is scheduled
every 500 ms. ON/OFF commands still work: the default is set only once per boot,
so OFF stays effective until ON or an MCU reset. Command 3 remains a received,
reported no-op and never starts calibration or writes flash.

Stream constant 0 m pressure with the real board stationary. Startup calibration
automatically begins when the stable pressure window is available, saving ground
pressure and gyro biases together. No calibration command is needed. Before a
valid pressure/reference exists, telemetry uses the usual unavailable-value encoding.
Blocking startup calibration can briefly pause transmission. Airborne/touchdown
recovery retains the saved calibration. LED-on and servo-at-0 startup are unchanged.

The original **TestHILS** target remains available with telemetry OFF by default.
In the instructions below, skip the Telemetry ON step for TestHILSTelemetryOn;
after reset this alternate target again defaults to ON. Both use the same GUI.

`Tests/testHILS.c` directly includes the current `Core/Src/apps/FlightApp.c`, rather
than maintaining a separate copy of its startup and loop. Both HILS variants inherit
the LED-on startup, enabled 5-degree servos, calibration/recovery policy, failure
checks and mission processing. Launch-pad calibration is deferred only until the PC
supplies a stable pressure window; it then calls the same startup calibration and
flash-save implementation. Airborne recovery uses that implementation immediately.

`Tests/testHILS.c` runs the existing **FlightApp loop**, including sensor acquisition,
ground commands, the state machine, camera control, telemetry and SD logging.
The PC supplies BMP pressure and temperature over the existing USART3 console.
Mission decisions are made on the MCU. The GUI does not predict or force states.

**This firmware operates real camera, HDRM and buzzer outputs and writes real
mission/calibration flash records.** Use an indicator/test load for the initial
HDRM run. A saved DESCENT_2 state can activate HDRM immediately on startup.
The LED turns on and both servos are PWM-enabled at 5 degrees during startup,
as in the other combined tests. Estimation/control is not implemented by this test.

## What is simulated

Only `BMP280_Init`, `BMP280_Read` and `BMP280_CalibrateGroundPressure` are replaced
by test-only linker wrappers in `Tests/hils/HilsBMP.c`. Production driver files
and FlightApp are unchanged. The wrapper accepts physical pressure in **Pa**, not
raw BMP ADC register bytes; it calculates relative height using the same formula
as BMP.c. It therefore tests the pressure-to-height/state integration, **not** the
BMP's I2C transport, factory compensation, internal filtering or physical response.

BMI270, IIS2MDC, AHT20, battery and GNSS remain real. SD and XBee remain real.
Connect the working sensors for calibration: the combined calibration still
requires BMP/AHT/BMI/MAG initialization and real stationary BMI gyro samples.
Missing GNSS fixes remain invalid; missing devices are not populated with fabricated data.

The flight scheduler remains acquisition every `ACQUISITION_PERIOD_MS` (10 ms),
telemetry every `COMMUNICATION_TELEMETRY_PERIOD_MS` (500 ms, when enabled), and
logging every `LOGGING_PERIOD_MS` (1000 ms). The PC supplies new pressure at 10 Hz,
close to the current BMP normal-mode rate. HILS reports status about every 200 ms.
Synchronous sensor/SD/console work can extend these periods. This is a functional
test, not a measurement of production loop performance.

## Build and program

From the project root in a shell with CMake, Ninja and the STM32 GNU toolchain:

```powershell
cmake --preset TestHILS
cmake --build --preset TestHILS
```

Program **`build/TestHILS/testHILS.elf`** using the usual ST-Link workflow. Select
the TestHILS preset/binary in the IDE; `main.c` already calls `App_Init` and
`App_RunOnce`, so it needs no editing. Resume execution if the debugger stops at main.
For recovery tests, reset the MCU without reflashing or erasing sector 11.
Mass erase destroys both mission state and calibration records.

## PC setup and connections

Use Python 3.10+ with Tkinter and pyserial:

```powershell
python -m pip install -r Tools/hils/requirements.txt
python Tools/hils/hils_gui.py
```

If `python` selects the unrelated GTKWave runtime on this PC, use the installed
Python 3.12 explicitly:

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe" Tools/hils/hils_gui.py
```

1. Connect the same USB/UART adapter normally used for the USART3 console,
   with the existing board pin wiring and common ground. Use **115200, 8N1**.
   Close the IDE serial monitor so the GUI can open that port exclusively.
2. Select the console COM port. Select **Console injection** for commands, or
   select a **different COM port** connected to the ground-side XBee at 115200.
   Select routes before connecting; reconnect to change ports.
3. Click Connect. Optionally choose a `.jsonl` file to capture all MCU status,
   command events and sent pressure samples with PC timestamps. Cancel skips capture.
4. Wait for MCU status. Initialization can take several seconds. A failed sensor
   does not stop other acquisition; `sensor_init=0` identifies partial startup.

Console injection passes the exact bytes `A5 ID 5A` into the existing
`Communication_ProcessCommand` parser in the MCU main loop. It exercises command
execution, bookkeeping and calibration, but bypasses XBee transport. With a
ground-side XBee selected, the GUI sends those three bytes over that radio; MCU
UART4 DMA reception and the real `Communication_Update` path are exercised too.
The GUI drains ground-radio telemetry without decoding it: its display comes from
the MCU's separate console status. Use your ground-station parser separately to
verify radio packet contents, without sharing a COM port between applications.

## First complete run

1. Check the displayed recovered state. For a new scenario click **New mission**.
   This explicitly appends LAUNCH_PAD, clears mission timers/peak, switches outputs
   off and starts a new SD log. The next stable ground stream starts calibration
   and replaces the saved pair on success. It is a test-console
   operation, not an additional flight telecommand. A running calibration blocks it.
2. Set Height = **0**, Noise = **0**, and ground pressure = **101325 Pa** (or your
   chosen reference). Click **Stream / resume**. Wait at least **2 seconds**.
   If calibration exists, the GUI automatically adopts the MCU's recovered ground
   pressure on connection and whenever that value changes. You can edit it afterward.
3. Keep the real board stationary. Calibration starts automatically when the stable
   ground window is available. Wait for `startup_calibration` with `ok=1`, calibration
   SUCCESS and `saved=1`. Command 3 is temporarily a no-op (received but not executed).
   Ground pressure is averaged from the last 16 valid simulated samples, spanning
   at least 1.4 s with no gap over 1 s and pressure spread no more than 50 Pa.
   This replaces the physical BMP's settling/read loop. BMI calibration and the
   atomic flash save/rollback are the existing application implementation.
4. Click **1: Telemetry ON**. Telemetry starts disabled at boot, as in FlightApp.
   Confirm TX count increments and SD row count increments. An accepted command
   is distinct from successful execution: watch both latest command and last
   successful command, especially for asynchronous calibration.
5. Click **Run profile** to execute the included 100-second nominal flight.
   The GUI plots requested height in cyan and fresh MCU height in orange.
   The profile finishes by continuously streaming 0 m.

Expected sequence with the current state-machine macros:

| State change | Condition / expected activity |
|---|---|
| LAUNCH_PAD → ASCENT | 5 distinct fresh samples above 30 m; camera recording already started on LAUNCH_PAD after startup calibration succeeded |
| ASCENT → DESCENT_1 | Observed peak above 700 m and fall greater than 20 m, confirmed by 5 samples |
| DESCENT_1 → DESCENT_2 | 5 samples below 600 m; HDRM on for 10 s; deployment completion saved |
| DESCENT_2 → TOUCHDOWN | 5 samples below 20 m; camera stops, buzzer starts, SD logging stops |

Values are controlled by `Core/Inc/apps/StateMachine.h`. Peak increases only in
ASCENT. Camera timing macros are in `CameraControl.h`. Ground-command processing
continues in every state. Automatic startup calibration runs only on LAUNCH_PAD
with healthy mission persistence; telemetry ON/OFF work independently of state.

## Noise, dropout and manual scenarios

Edit Height while streaming to hold any level. **Hold current height** exits the
profile and holds the last supplied value. **Pause input** stops pressure packets;
the MCU continues command processing, camera/HDRM timers, logging and telemetry.
After 1 s the height becomes stale and cannot advance a transition. Resume does
not replay missed samples. The GUI's status-stale warning instead means console
reports have stopped; it must not be interpreted as a valid current actuator state.

Add bounded uniform height noise using Noise ±m. For repeatable spikes or gate
checks, load a CSV with `time_s,height_m`, at least two rows, first time 0 and
strictly increasing times. Values are linearly interpolated; the last height is
held. Use the supplied `Tools/hils/profiles/nominal.csv` as a starting point.
Time is wall-clock time: no acceleration that would distort camera/HDRM timing.

Suggested checks:

- Hold 25 m; inject one or a few samples above 30 m, then return to 25 m. No transition.
- Establish a peak below 700 m and drop more than 20 m. Remain in ASCENT.
- Establish 850 m, then hold 820 m. Enter DESCENT_1 after confirmation.
- In DESCENT_1 alternate 590/610 m to reset confirmation; then hold 590 m to deploy.
- Pause during a partial gate confirmation; after staleness, require a new full count.
- Toggle telemetry OFF and ON during the flight. TX count stops/restarts; logging
  and command reception continue. TX count means UART acceptance, not RF delivery.
- Request calibration after leaving LAUNCH_PAD. Expect a received command with
  failed execution and unchanged previously successful command ID.

## MCU reset and flash recovery

**MCU reset** issues `NVIC_SystemReset()` with no simulated restart or graceful
application shutdown. The GUI holds the current requested height during restart
and resumes streaming when MCU status returns. You can also press the hardware
reset button. Use Hold/Pause before an external reset if you want the profile clock
to stop advancing. Reconnect the GUI if your USB adapter itself disconnects.

- Reset in ASCENT: state and ground/gyro calibration recover. Camera startup is
  replayed. **Peak height is RAM-only** in the existing implementation; after a
  reset it must observe a new qualifying peak above 700 m. This HILS does not mask
  that limitation or persist additional mission variables.
- Reset during the DESCENT_2 HDRM pulse: recovery repeats the full pulse, as
  specified by the existing state machine. After deployment completion is saved,
  resetting in DESCENT_2 must not repeat the pulse.
  During this 10-second window, the hardware reset button is easier than the
  GUI confirmation dialog; use an indicator load while testing pulse replay.
- Reset in TOUCHDOWN: touchdown actions are restored; logging stays stopped.
- `recovered=1`, `flash=1`, unchanged ground pressure and `saved=1` show recovery
  status. New SD filenames are expected on each boot; telemetry/command counters
  and telemetry enabled state are RAM values and reset. Send TELEMETRY_ON again.
- The append-only mission/calibration journal is shared with flight firmware in
  sector 11 at `0x080E0000`. New mission does not erase or free it. A full/corrupt
  journal is reported as a failure and blocks transitions/calibration as usual.

Before returning to a real flight, explicitly start a new mission and calibrate
using the real BMP in flight firmware. HILS can leave simulated pressure and an
in-flight state in flash; merely switching firmware does not clear them.

## Protocol and status details

PC → console, ASCII, newline terminated:

```text
H1,P,<uint32 sequence>,<pressure Pa>,<temperature C>*<CRC16 hex>\n
H1,C,<1|2|3>*<CRC16 hex>\n
H1,STATUS*<CRC16 hex>\n
H1,NEW*<CRC16 hex>\n
H1,RESET*<CRC16 hex>\n
```

CRC is CCITT-FALSE (polynomial 0x1021, initial 0xffff, no final XOR) over every
ASCII byte before `*`. `Tools/hils/protocol.py:frame()` constructs packets.
Only pressure 30000–110000 Pa and temperature −40–85 °C are accepted. Duplicate
or backward pressure sequence numbers are rejected; uint32 wrap is supported.
Only the latest pending pressure sample is consumed, once, with the MCU receipt
timestamp retained. Floods are not replayed as a fast burst of confirmations.
The console's interrupt RX ring is drained in the main loop; parsing has a bounded
budget. CRC/range/framing failures are counted. The reset and new-mission controls
are intentionally explicit test actions and never generated by the height profile.

Height → pressure on the PC:

```text
pressurePa = groundPa * (1 - heightM / 44330) ** (1 / 0.1903)
```

MCU → console is newline-delimited JSON. `status` includes input age/sequence,
pressure, reference, barometric height/freshness, mission state, peak/debounce count,
camera phase, HDRM/deployment/buzzer, logging filename/rows/errors, TX/RX counters,
command results, calibration, recovered/persistence flags and serial diagnostics.
`command` events use event 0=RECEIVED, 1=PENDING, 2=FINISHED. `ok` is meaningful on
FINISHED. GPIO/camera status describes commanded output latches/software state,
not physical confirmation that a camera recorded or a release mechanism deployed.
Inspect the actual SD CSV and radio reception to verify those external effects.

## Automated checks

```powershell
powershell -ExecutionPolicy Bypass -File Tests/host/run_hils_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe
python Tools/hils/test_protocol.py
python Tools/hils/test_gui.py
```

The C test covers corrupted/range-invalid/replayed input, sample freshness,
pressure-derived transitions with debounce, camera timing, HDRM replay/completion,
touchdown, and mixed mission/calibration flash recovery with mocked hardware.
The Python tests cover pressure conversion, profiles, CRC, and a hidden-window
GUI exercise with fake serial ports. Existing communication, calibration,
acquisition, state and logging suites cover their corresponding application paths.
Physical timing, GPIO/load behavior, radio delivery and power-loss recovery still
need the board run described above.
