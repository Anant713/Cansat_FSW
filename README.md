# DCS firmware core

The first portable C modules implement target-centred horizontal navigation,
barometric vertical position, and the CDR's straight-flight, GNSS-based wind
measurement with an exponential moving average. A compact payload attitude
estimator uses x-io Fusion to combine gyro, gated accelerometer, and calibrated
magnetometer data.
These modules have no MCU or driver
dependencies and use caller-owned, fixed-size state.

[SENSOR_AXIS_CONVENTIONS.md](SENSOR_AXIS_CONVENTIONS.md) records the installed
physical-Body, BMI270, IIS2MDC and payload frames. `dcs_sensor_transform` maps
calibrated native sensor vectors into the payload frame and provides the
optional BMI-to-cylinder-axis rotational acceleration correction.

This repository is the portable DCS flight core. Fusion is integrated into the
attitude module. Desktop checks establish software behavior, not aerodynamic
accuracy; the frozen flight parameters remain assumptions accepted because no
further identification time is available.

[FLIGHT_TEST_VALIDATION.md](FLIGHT_TEST_VALIDATION.md) is the separate register
of assumptions, required drone-drop data, identification tasks, and validation
limits. Update it as new design assumptions are introduced and link results to
specific runs; approved design choices are not automatically flight-validated.

[Test2_3010 damaged-drop analysis](Flight_Data/Analysis_Test2_3010/REPORT.md)
records raw-log integrity, flight phases, offline calibration candidates,
GNSS/barometer disagreement and velocity-circle fits. It supplies no validated
healthy trim or control settings. Reproduce derived data/plots with
`python tools/analyze_flight_data.py` (NumPy, pandas, SciPy and Matplotlib).

Architecture, algorithms, interfaces, thresholds, and recovery behavior must
be discussed with the project owner before implementation. Routine drivers
and low-level implementation can proceed within the agreed design.

[dcs_config.h](dcs_config.h) is the frozen flight-value register, including the
target at 26.717535 N, 84.301543 E. [dcs_flight_config.h](dcs_flight_config.h)
and dcs_flight_config.c assemble those constants into one `DcsFlightConfig` for
the main FSW. They were promoted from unvalidated placeholders by programme
decision and must be archived with the flight log.

[dcs_planner.h](dcs_planner.h) and dcs_planner.c implement bounded six-family
candidate planning: select final course, connect to the energy point, append
the combined straight through final entry to target, evaluate wind/time/height,
and rank candidates. Outputs contain four segments and path-distance markers
for both energy management and final entry. It also supports a post-energy
connection directly to the final-approach entry.

The agreed runtime uses one persistent guidance state and one bounded update
for path progress, direct energy/final-entry/target decisions, reassessment and
LOS guidance. No event queue, acknowledgment interface or obsolete-plan event
history is required. Marker handling is internal; logging does not gate control.
This is implemented by dcs_guidance.h/.c. It includes full/partial tangent
loiter, reconnection to qA, direct-target fallback, final-line continuation,
wind/height/path reassessment and flare latching; see GUIDANCE_DESIGN.md.

[CIRCLE_WIND_ESTIMATION_DESIGN.md](CIRCLE_WIND_ESTIMATION_DESIGN.md) records the
requested GPS velocity-circle method for wind/airspeed identification, primary
references, and why the damaged circling drop does not establish healthy trim
speed. dcs_circle_fit.h/.c implements the bounded velocity-circle fit, and
dcs_runtime.h/.c commands the configurable identification turn followed by a
settled trim-speed sampling stage. The same manoeuvre estimates turn rate from
the ordered wind-corrected velocity vectors and sets the planner turn radius to
airspeed divided by turn rate. The damaged flight still supplies no healthy
flight value, so the configured 20 m radius remains the fallback.

dcs_control.h/.c implements the configurable heading-error/rate-feedback
controller, single-sided brake allocation, symmetric flare allocation and
independent calibrated left/right servo pulse mapping. Hardware PWM generation
remains in the main FSW driver.

[dcs_dubins.h](dcs_dubins.h) and dcs_dubins.c implement the six Dubins geometry
families in North/East coordinates, returning straight/arc segments and length/
turn metrics. This is a building block, not complete planning or guidance.
The normalized equations adapt Andrew Walker's MIT-licensed Dubins-Curves;
the copyright and permission notice are retained in the source.

`dcs_nav_step` corrects North/East position and velocity from GNSS, holds fresh
GNSS ground velocity between fixes, and takes target-relative height from the
barometer. Its horizontal propagation does not require a canopy-heading
estimate.

The project owner selected retaining this GNSS velocity hold as the V1 baseline,
including during turns. Freshness limits and measured turn accuracy still need
validation. No turn-aware propagation is added.

`dcs_wind_update` accepts processed GNSS ground velocity, a recent payload
heading estimate, and the current normalized brake command. The thresholds and
EMA coefficient are configuration values, not flight-calibrated defaults.
Its estimate is **provisional** because payload heading is not a direct canopy
heading measurement. Do not enable wind-based guidance solely on the basis of
these desktop tests.
The project owner agreed to retain this wind model for V1 with configurable
airspeed, gate thresholds, and EMA strength. `last_update_ms` records the GNSS
measurement timestamp of the last accepted wind update. Rejected samples leave
it unchanged. Check `has_estimate` before computing its age; a timestamp of zero
can be valid. Retain accepted wind without expiry; age alone does not request
reassessment. Before any estimate exists, planning explicitly assumes zero wind.

`dcs_attitude_update` accepts calibrated gyro, accelerometer, and magnetometer
vectors from flight software, which owns calibration. The interface requires
payload forward/right/down axes; who performs that axis mapping must be confirmed
at integration. Units are rad/s for gyro, m/s² for accelerometer specific force
(including gravity at rest), and the configured field-strength units for
magnetometer. The module performs no sensor calibration. It estimates a
payload body-to-NED quaternion, geographic heading, and heading rate. The caller
allows accelerometer correction only under suitable flight conditions.
Accelerometer magnitude alone cannot prove that its direction is gravity.

Initialization takes the known launch-pad roll, pitch, geographic heading
(radians in `DcsAttitudeConfig`) and the pose timestamp in milliseconds passed
to `dcs_attitude_init`. It does not derive the starting pose from sensor samples.
Run attitude updates throughout the flight, including before secondary DCS.
The known pose supplies the initial heading reference; accepted magnetometer
corrections refresh it. `max_heading_reference_age_ms` retains the existing
heading-reference timeout policy. Repeated timestamps are ignored without
changing state. An excessive time gap still requires explicit reinitialization
from a known current pose; the launch-pad pose must not be reused in flight.
The implementation and approved initial bench behavior are documented in
[ATTITUDE_FUSION_DESIGN.md](ATTITUDE_FUSION_DESIGN.md). Physical and replay checks are in
[ATTITUDE_BENCH_TESTS.md](ATTITUDE_BENCH_TESTS.md).

The approved first bench settings are gain 0.5, accel/mag angular rejection
10 degrees, recovery timeout 5 s, and an acceleration band of 0.8-1.2 g. These
are caller-supplied test settings, not flight-validated defaults. See
`bench_config()` in tests/test_dcs_attitude.c for a complete example. Sample rate
and gyro range must match acquisition. Pose, magnetic field strength/tolerance,
maximum step, and heading-reference-age limit also require explicit configuration.
Keep config unchanged after initialization; Fusion caches its settings at init.

Accelerometer and magnetometer corrections require caller permission, provider
validity, and finite values. Acceleration outside the configured band is withheld
even during recovery. The prior configurable magnetic total-field magnitude
check is retained; the former 10% horizontal-field cutoff is removed. A zero
horizontal magnetic vector is withheld to avoid normalizing zero. Fusion then
applies its angular rejection and upstream recovery logic to admitted references.

Internally Fusion uses magnetic NED. The wrapper initializes magnetic heading as
true heading minus east-positive declination, and rotates the output quaternion
back into true NED. Heading rate is computed from body gyro and orientation, not
assumed to equal gyro Z. The earlier geometry policy is retained: heading is held
and marked invalid when squared horizontal projection of the unit forward axis
is below 0.04; quaternion updates continue. Reference-age and large-gap policies
remain provisional. Numerical validity is not an attitude accuracy guarantee;
inspect recovery/overrange flags too.

The IMU timestamp must identify a new measurement. The caller provides a recent
magnetic sample and clears `mag_valid` when stale. This interface does not have a
separate magnetic timestamp. Zero vectors represent references withheld from
Fusion; they do not stop gyro integration. No calibration or separate gyro-bias
estimator is used here.

On Windows, compile every portable DCS and Fusion source into one static
library, then run all desktop checks:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_all.ps1 -Target Host
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
python tools/vendor_fusion.py --verify
```

The host library is `.build/host/all/libdcs.a`. To build the same sources for
the STM32F405 Cortex-M4F using the GNU Arm toolchain:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_all.ps1 -Target Arm
```

This produces `.build/arm/all/libdcs.a`. A GNU Makefile provides equivalent
`make host`, `make arm`, and `make tests` targets where GNU Make is installed.

The flight-software integration supplies monotonic millisecond timestamps,
fresh valid GNSS solutions, calibrated attitude sensor vectors, and brake commands.
For FSW integration, compile dcs_attitude.c and the pinned FusionAhrs.c with
`FUSION_USE_NORMAL_SQRT` consistently defined, and retain the Fusion headers and
MIT license under third_party/Fusion. This uses upstream's standard square-root
option without changing upstream sources. No additional loop or scheduler is
required: keep one DcsAttitudeState and update it for each new gyro sample.

Hardware bench calibration remains required. The wind model and full guidance
phase sequence are implemented. [GUIDANCE_DESIGN.md](GUIDANCE_DESIGN.md) records
the runtime, fallbacks and parameters that still need physical calibration.
The integrated main FSW calls dcs_runtime_start only after its own state machine
reports secondary descent and completed parafoil deployment. The runtime then
enforces a further 5 s canopy-settling interval before identification. It calls
dcs_runtime_update from the cooperative super-loop, does not infer deployment
or duplicate the mission state machine, and returns pulse widths without
accessing PWM hardware.
See [DCS_RUNTIME_INTEGRATION.md](DCS_RUNTIME_INTEGRATION.md) for the exact call
boundary, input/output wiring and mandatory pre-flight calibration values.

The main FSW is evolving. Its integration model is one-time initialization,
bounded updates called from a cooperative super-loop, persistent module state,
and timestamped inputs. Existing FSW app names, shared fields, scheduling, units,
and ADCS/DCS interfaces are preliminary examples rather than fixed requirements.
