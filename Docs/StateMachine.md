# Persistent mission transitions

`StateMachine_Update()` runs on each flight-loop pass after acquisition and command
service. Thresholds and durations are macros in `Core/Inc/apps/StateMachine.h`.
It uses `navigation.altitude` in metres above launch ground, `altitudeValid`, and
`altitudeTimestampMs`. Estimates older than 1000 ms, invalid values, and NaN/Inf
cannot advance the mission. The barometric altitude operation supplies these
fields before state updates. Sensor acquisition continues to copy driver units.

| Transition | Gate |
| --- | --- |
| Launch pad -> Ascent | Height >30 m |
| Ascent -> Descent 1 | Height has exceeded 800 m, then drops >2 m over a five-second observation window |
| Descent 1 -> Descent 2 | Height <600 m |
| Descent 2 -> Touchdown | Height <20 m |

The ascent test compares the endpoints of successive five-second windows after
arming. It does not require every intermediate sample to decrease. Equality at
any threshold does not trigger a transition. Invalid/stale altitude resets the
observation window. A reboot preserves the 800 m latch but starts a fresh window
because the MCU clock and observation history were lost. At most one state
transition is performed per loop pass.

## Recovery and flash

Sector 11, `0x080E0000` through `0x080FFFFF`, is reserved for a 128 KiB journal.
The linker limits firmware to the preceding 896 KiB. Each 16-byte record contains
a versioned magic, state/flags/ground-pressure reference, their bitwise complement, and a commit word written
last. Startup scans for the last valid committed record, skipping torn records.
State changes and the altitude/deployment latches are committed before dependent
state actions. No sector erase occurs in flight. There are 8192 record slots;
a full journal or failed write inhibits further transitions and exposes
`StateMachine_GetStatus()->persistenceOK == false`.

Erased flash means a fresh launch pad mission. Nonempty flash with no valid record
means a recovery error: mission state remains `CANSAT_STATE_UNINITIALIZED` (255),
not a fabricated launch pad state. `flight_state_init_ok` exposes initialization
success. A committed touchdown is recovered as touchdown on the next boot.

`StateMachine_BeginNewMission()` is an explicit ground-only reset to launch pad,
appended to the journal; it is never called automatically. Reboot afterward to
start a new logging session. The future ground-command parser can invoke this
operation when the command is defined. Preserve sector 11 when reflashing if
recovery is desired; programmer mass erase destroys the journal. Bench erase of
sector 11 resets the journal and must not be done during a mission.

## Entry actions

- Ascent powers DVR, raises Capture and timestamps it. After 1000 ms, Capture is
  lowered and `camera.isRecording` becomes true. Recovery in ascent or descent
  repeats this startup sequence because GPIO outputs were reset.
- Descent 1 sets `reactionWheelRequested` and `actuators.reactionWheel.enabled`.
  Both remain true in Descent 2 and clear on touchdown or a new mission.
  This is software enable only. PB5 is the intended ESC PWM signal; timer setup
  and PWM commands will be implemented with the later control algorithm.
  The PWM value/valid flag are not fabricated.
- Descent 2 raises HDRM for 2000 ms, then lowers it and commits deployment complete.
  The confirmed recovery policy repeats a full pulse if completion was not committed.
  Completed deployment is not pulsed again. `parafoilDeployed` indicates commanded
  pulse completion, not physical feedback. Even if saving completion fails, HDRM
  is turned off; another restart will repeat the pulse.
- Touchdown lowers HDRM, starts the buzzer, stops/closes logging, raises Capture,
  clears recording, then lowers DVR and clears camera power. Failed log closes
  are retried. Telemetry and acquisition continue. No mission state gates attitude
  estimation; calibrated estimation/control algorithms are still unimplemented.

Camera and HDRM timers use wrap-safe elapsed-time checks without delays. Blocking
sensor/SD operations can extend pulses past their nominal duration; hard maximum
pulse timing would need hardware timers or interrupt-driven output shutdown.

## Camera test and verification

Configure/build `TestCamera` and flash `build/TestCamera/test_camera.elf`.
It powers DVR, pulses Capture high for one second, records for ten seconds measured
from the falling edge, then raises Capture and switches DVR off. It runs once per
boot. `test_camera`, `test_camera_complete`, and USART3 console output expose its
progress. This test does not write the mission journal. Verify the actual saved
video on the camera; flags represent commands, not recording feedback.

`Tests/host/run_state_tests.ps1` checks threshold boundaries, descent windows,
recovery, interrupted flash writes, full journal, stale altitude, camera timer
rollover, HDRM recovery/shutdown, and touchdown outputs. Existing acquisition,
communication and logging suites remain applicable. Power interruption and flash
retention on the board still require hardware validation.


## Barometric launch-relative altitude

`BarometricAltitude_Update()` runs before state gates on every loop pass. It uses
BMP pressure in Pa and the standard-atmosphere pressure-ratio approximation:

`altitude_m = 44330 * (1 - (pressure_pa / ground_pressure_pa)^0.19029495)`

The first fresh valid BMP sample while on the launch pad establishes ground
pressure, rounded to whole Pa and committed to the mission journal before the
altitude becomes valid. Start a new mission while stationary at the launch site.
The reference stays fixed; it is not continuously adjusted while waiting or flying.
No extra filtering is applied beyond the existing BMP driver configuration.
Atmospheric temperature/weather variation and pressure disturbances affect accuracy.

Ground pressure occupies bits 10..26 of the journal payload; zero means unset.
Mission transitions preserve it, and an explicit new mission clears it. A recovered
mid-flight state without a reference (including older firmware records) cannot
automatically calibrate at its current height: altitude stays invalid. A recovered
launch-pad state with no reference can establish it from a fresh sample.

Pressure must be finite and within 30000..110000 Pa. Samples older than 1000 ms
are invalidated, retaining their last numerical altitude but not allowing state
transitions. The altitude timestamp is the sensor timestamp, never the loop time.
Negative relative altitude is preserved; the existing unsigned telemetry field
encodes such values as its reserved unavailable sentinel. The reference persists
at 1 Pa resolution (roughly 0.08 m near sea level).

Host tests also check a 500 m pressure example, negative height, stale/invalid
pressure, reference-write failure, recovery without re-zeroing, and a legacy
mid-flight record with no reference.
