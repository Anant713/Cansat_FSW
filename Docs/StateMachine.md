# Persistent mission transitions

`StateMachine_Update()` runs on each flight-loop pass after acquisition and command
service. Thresholds and durations are macros in `Core/Inc/apps/StateMachine.h`.
All height gates use `bmp.relativeHeightM` in metres, with `bmp.valid`,
`bmp.heightValid`, and `bmp.timestamp_ms`. Samples older than 1000 ms,
invalid values, and NaN/Inf cannot update the peak or advance the mission.
`navigation.altitude` is not used by any transition gate.

| Transition | Gate |
| --- | --- |
| Launch pad -> Ascent | Height >30 m |
| Ascent -> Descent 1 | Peak > STATE_DESCENT_MIN_PEAK_M (700 m) AND peak - current height > STATE_DESCENT_DROP_M (20 m), debounced |
| Descent 1 -> Descent 2 | Height <600 m |
| Descent 2 -> Touchdown | Height <20 m |

Only checks that execute the **ASCENT** state update `maxHeightM`, when the current
fresh, valid BMP height exceeds the previous peak. A lower reading retains the peak. The ASCENT gate
checks the drop on that same pass; exactly 20 m does not trigger it. There is no
800 m arming threshold or five-second observation window. The new peak requirement
is strictly above 700 m; exactly 700 m does not qualify. Both conditions must hold
for every counted sample in the debounce sequence. Invalid/stale samples
leave the peak unchanged. At most one transition occurs per loop pass.

All four height gates require `STATE_HEIGHT_CONFIRM_SAMPLES` consecutive
qualifying samples (default **5**, configurable in StateMachine.h; minimum 1).
The counter saturates at the threshold. A false condition, invalid/nonfinite/stale
sample, or a gap greater than STATE_ALTITUDE_MAX_AGE_MS resets the count. State
entry and initialization reset it too; it is not persisted. Only a new BMP
timestamp counts: repeated main-loop calls with the retained sample cannot
confirm a transition. Timed camera/HDRM actions still run on every call.
These are distinct driver sample timestamps, not a guarantee of distinct physical
conversions; debounce duration therefore depends on successful BMP read cadence.

The raw maximum still updates only in ASCENT, before checking the drop. A new
maximum cancels a pending descent count. Debouncing does not filter high spikes
out of the peak: one false high followed by five lower samples may still trigger
descent. Peak/outlier filtering would be a separate change.

The peak is RAM-only and resets on startup or BeginNewMission; the first valid
ASCENT check seeds it. The launch-pad check that transitions into ASCENT does not
seed the peak; the next ASCENT check does. The peak remains unchanged in launch
pad, DESCENT_1, DESCENT_2 and touchdown. Mission state is still recovered from flash. Legacy flash bit 8
(the old 800 m latch) is accepted but ignored and no longer written.

The BMP driver establishes a new relative-height zero during each initialization.
Consequently, after a mid-flight restart these gates use height relative to the
restart location, and the pre-reset peak is unavailable. The separately persisted
navigation reference does not correct `bmp.relativeHeightM`. Preserving launch-
relative gates and the peak through a reboot would require a separate recovery
change to the BMP reference/peak storage.

## Recovery and flash

Sector 11, `0x080E0000` through `0x080FFFFF`, is reserved for a 128 KiB journal.
The linker limits firmware to the preceding 896 KiB. Each 16-byte record contains
a versioned magic, state/flags/ground-pressure reference, their bitwise complement, and a commit word written
last. Startup scans for the last valid committed record, skipping torn records.
State changes, ground reference and deployment status are committed before dependent
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
- Descent 1 keeps camera recording active; it adds no actuator command.
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
Send `record` on the USART3 console to power DVR and pulse Capture high for one
second; send `stop` to raise Capture and switch DVR off. `status` reports camera
and console diagnostics. There is no automatic ten-second stop.
`test_camera`, `test_camera_complete`, and console output expose its progress.
This test does not write the mission journal. Verify the actual saved
video on the camera; flags represent commands, not recording feedback.

`Tests/host/run_state_tests.ps1` checks threshold boundaries, ASCENT-only peak tracking,
recovery, interrupted flash writes, full journal, stale altitude, camera timer
rollover, HDRM recovery/shutdown, and touchdown outputs. Existing acquisition,
communication and logging suites remain applicable. Power interruption and flash
retention on the board still require hardware validation.


## Auxiliary navigation altitude (not used by state gates)

`BarometricAltitude_Update()` still populates navigation altitude independently;
state gates now read the BMP height directly. It uses
BMP pressure in Pa and the standard-atmosphere pressure-ratio approximation:

`altitude_m = 44330 * (1 - (pressure_pa / ground_pressure_pa)^0.1903)`

The first fresh valid BMP sample while on the launch pad supplies the driver's
startup-averaged `bmp.groundPressurePa`, rounded to whole Pa and committed to the mission journal before the
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
invalidate auxiliary navigation altitude while retaining its last numerical value.
This does not control the state gates: they separately check BMP height validity
and freshness. The altitude timestamp is the sensor timestamp, never the loop time.
Negative relative altitude is preserved. Telemetry ALTITUDE uses BMP relative
height, not navigation altitude; negative BMP height encodes as the unsigned
field's unavailable sentinel. The auxiliary reference persists
at 1 Pa resolution (roughly 0.08 m near sea level).

Host tests also check a 500 m pressure example, negative height, stale/invalid
pressure, reference-write failure, recovery without re-zeroing, and a legacy
mid-flight record with no reference.

