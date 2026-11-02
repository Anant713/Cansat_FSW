# DCS Firmware Architecture

## 1. Purpose

This document defines the initial architecture of the CAN-7USAT Descent Control System firmware.

The firmware is intended to:

- run ultimately on an STM32F4-class flight computer,
- remain portable across ESP32, Arduino, Raspberry Pi, and desktop test environments,
- keep GNC algorithms independent of board-specific drivers and APIs,
- support simulation, HIL testing, and flight-test parameter identification.

The current CAN-7USAT flight-software architecture uses a cooperative super-loop, with hardware-specific operations isolated in the driver layer.

---

## 2. Architecture Status
FLIGHT_TEST_VALIDATION.md separately tracks assumptions requiring drone-drop,
bench, or replay evidence. Maintain it as the design evolves. An architectural
decision or a host test does not close its flight-validation entry.

Current checkpoint: portable navigation, wind, Fusion attitude, planning,
guidance/loiter, circle identification, heading control and servo mapping exist
with desktop tests. Hardware bench and representative drop-test results are pending. The
application proposal and test procedure are in ATTITUDE_FUSION_DESIGN.md and
ATTITUDE_BENCH_TESTS.md.

The status labels below record the evolving design, not permission to make new
architectural decisions without discussion. Algorithms, interfaces, thresholds,
and recovery policies must be agreed with the project owner before changes.
Existing ADCS/DCS FSW interfaces remain preliminary. Integration must support
one-time initialization, bounded super-loop updates, persistent estimator state,
and measurement timestamps without imposing a separate scheduler.

The architecture is divided into three categories.

### Frozen

These decisions are considered fixed for the first firmware implementation unless testing reveals a fundamental issue.

### TBD

These items are part of the intended architecture but still require design or tuning.

### Future

These are deliberately excluded from V1 and may be added after simulation, HIL, or flight testing.

---

# 3. High-Level DCS Architecture

**Status: Frozen**

The DCS core is hardware-independent.

The intended information flow is:

```text
GNSS + ADCS
    |
    v
Navigation
    |
    +----> Wind Estimation
    |
    v
Trajectory Planning
    |
    v
Path Management
    |
    v
LOS Guidance
    |
    v
Heading Control
    |
    v
Brake Allocation
    |
    v
Servo Mapping
    |
    v
Hardware PWM Driver
```

This follows the existing GNC formulation:

```text
GNSS + attitude solution
-> target-centred navigation state
-> wind estimation
-> wind-aware trajectory planning
-> LOS guidance
-> heading control
-> single-sided brake actuation
```



The DCS core shall not directly contain:

- STM32 HAL calls,
- Arduino APIs,
- ESP-IDF APIs,
- Linux system calls,
- PWM peripheral configuration,
- direct RTC access,
- UART parsing,
- GNSS protocol parsing.

---

# 4. Implementation Language and Portability

**Status: Frozen**

The DCS core will be implemented in plain C.

The implementation should use:

- fixed-size data structures,
- explicit state ownership,
- no dynamic memory allocation,
- no dependency on RTOS services,
- no dependency on platform-specific timers.

Board-specific code will provide:

- sensor measurements,
- system time,
- communication interfaces,
- actuator output.

---

# 5. Time Architecture

## 5.1 System Time

**Status: Frozen**

Elapsed time is supplied to DCS by the main flight software as a monotonic
millisecond counter. RTC calendar time may be used for mission timestamps but
must not be used to calculate propagation or control time steps.

System time is used for:

- propagation timestep calculation,
- controller timestep calculation,
- guidance scheduling,
- planner/replanner timing,
- internal state timestamps,
- wind-update timing,
- execution timing.

DCS modules shall not read MCU timers directly.

---

## 5.2 GNSS Time

**Status: Frozen**

GNSS time is used only to identify whether a received GNSS solution is new.

Conceptually:

```text
if current_gnss_time != previous_gnss_time:
    process GNSS fix
```

The previously processed GNSS time is stored internally.

GNSS time is not currently used for:

- propagation timing,
- controller timing,
- delayed-measurement reconstruction,
- ADCS/GNSS epoch synchronization.

---

# 6. Coordinate Frames

## 6.1 Target-Centred Local Frame

**Status: Frozen**

The GNC system uses a target-centred local North-East-Down frame.

Axes:

```text
+x : North
+y : East
+z : Down
```

The target is therefore:

$$
\mathbf{p}_T =
\begin{bmatrix}
0 \\
0 \\
0
\end{bmatrix}
$$

Incoming GNSS geodetic coordinates are converted into this local Cartesian frame before being used by GNC.

The planner and guidance algorithms operate only in local Cartesian coordinates.

---

## 6.2 Geographic Heading

**Status: Frozen**

The geographic parafoil heading is denoted by $\psi$.

It is measured clockwise from North.

Thus:

$$
\psi = 0
$$

corresponds to North, and

$$
\psi = \frac{\pi}{2}
$$

corresponds to East.

---

# 7. GNSS Interface

**Status: Frozen**

The DCS does not parse NMEA or other GNSS protocols.

GNSS parsing belongs to the GNSS driver or flight-software layer.

The DCS receives processed quantities including:

- latitude,
- longitude,
- altitude,
- ground speed,
- course over ground,
- optional GNSS Down velocity for comparison with barometric vertical motion,
- GNSS time.

Latitude and longitude should use sufficient precision for geodetic conversion.

After conversion into the local frame, single-precision floating-point values are expected to be sufficient.

---

# 8. Attitude Support Interface

**Status: Provisional pending parafoil-payload flight data**

Fusion is integrated as the attitude filter using unchanged upstream revision
a8d7224f36a0ec82345ef49a3db50e65f8d3bab8. The user approved configurable initial
bench settings: acceleration band 0.8-1.2 g, gain 0.5, accel/mag angular rejection
10 degrees, recovery timeout 5 s, and upstream recovery for admitted references.
These settings are not validated flight parameters. ATTITUDE_FUSION_DESIGN.md
describes the wrapper, retained provisional policies, and diagnostics.

The reaction wheel has been descoped. Attitude estimation exists to support
DCS, regardless of which flight-software module owns its implementation.
Attitude estimation operates throughout the flight, beginning on the launch
pad, rather than starting only when secondary DCS becomes active. Activating
secondary DCS must not itself reset the attitude estimate. The launch-pad
attitude is supplied as initial roll, pitch, and geographic heading with its
timestamp. The estimator starts directly from that pose without reconstructing
it from accelerometer or magnetometer measurements. The known pose provides the
initial heading reference; accepted magnetometer corrections refresh it.
Repeated timestamps are ignored without resetting the estimate. The existing
heading-reference timeout and excessive-gap recovery policy remain provisional.
Flight software owns sensor calibration, including any prelaunch calibration
command, and supplies calibrated gyro, accelerometer, and magnetometer vectors
to the attitude module. The attitude module applies no calibration matrices or
bias corrections. Its current interface requires payload forward/right/down
axes; responsibility for sensor-to-body axis mapping must be confirmed at
integration.

The calibration owner must check bias repeatability across power cycles for
the gyro, accelerometer, and magnetometer. Record each bias estimate, sensor
temperature, time since power-on, and the hardware/software offset settings
under comparable stationary test conditions. For the magnetometer, keep the
orientation and magnetic environment controlled. Use these results to establish
whether stored calibration remains valid after restart and how much settling
time is needed. Prelaunch calibration behavior belongs to flight software.

The BMI270 gyroscope and accelerometer, together with the IIS2MDC
magnetometer, can estimate the payload's orientation. They do not directly
measure parafoil-canopy orientation. The attitude interface supplies:

- estimated payload geographic heading and heading rate,
- a monotonic measurement timestamp,
- validity/quality information.

The heading used in the parafoil model is the air-relative canopy-forward
direction. Payload heading is only a candidate proxy for this quantity.
Oscillation, twisting, mean offset, and turn-response lag will be studied in
flight tests. A raw payload body-axis gyro channel is not assumed to equal
geographic heading rate.

An attitude estimator may maintain a quaternion internally to combine gyro,
accelerometer, and magnetometer data; DCS guidance needs only the resulting
horizontal direction and rate. Accelerometer tilt corrections must be reduced
or rejected during strong non-gravitational acceleration.

No heading sample is used without checking its validity and age. The mapping
from payload heading to canopy heading remains an explicit model assumption,
not an established sensor measurement.

---

# 9. Navigation Architecture

## 9.1 Navigation Philosophy

**Status: GNSS velocity hold selected for V1**

GNSS updates more slowly than the desired navigation and control execution rates.

A translational propagator is therefore used between GNSS fixes.

The project owner selected retaining the current baseline: replace horizontal
position and velocity on each accepted GNSS fix, then hold that velocity while
the fix remains fresh to propagate position between fixes. No turn-aware or
attitude-derived horizontal velocity propagation is added for V1. Numerical
freshness limits and actual turn accuracy still require validation.

Only translational quantities are propagated in V1.

Heading and heading rate are payload estimates until flight data establishes
whether they can represent canopy motion. GNSS course over ground is retained
as a distinct measurement of motion over the ground.

---

## 9.2 Navigation State

**Status: Frozen**

The navigation state conceptually contains:

```text
Position:
    x_north
    y_east
    z_down

Estimated ground velocity:
    V_north
    V_east
    V_down

Latest attitude support data:
    payload_heading
    payload_heading_rate
    validity and age
```

The navigation state must not derive velocity by differentiating corrected position.

---

# 10. GNSS Navigation Update

**Status: Provisional**

Whenever a fresh GNSS fix is received:

1. Validate the fix and convert GNSS latitude and longitude to local North/East.
2. Override the propagated horizontal position with GNSS position.
3. Store the measured GNSS ground velocity.
4. Offer the sample to the gated wind estimator with a recent, valid payload
   heading; its output remains provisional while canopy-heading alignment is
   unverified.
5. Override the current velocity estimate with the trusted GNSS velocity.
6. Continue propagation from the corrected state.

GNSS supplies horizontal position and velocity. Barometric measurements supply
the operational vertical position relative to the target ground level.

---

# 11. Horizontal Velocity Propagation

**Status: GNSS velocity hold selected for V1; parafoil model is provisional**

The implemented V1 propagator uses the last accepted GNSS North/East ground
velocity until its configured age limit. The equations below describe the
alternative parafoil model; they are not the selected V1 navigation propagator.

The reduced parafoil model estimates horizontal velocity from:

- calibrated horizontal airspeed,
- canopy heading or a validated proxy,
- current wind estimate.

The horizontal model is:

$$
V_{N,\mathrm{est}}
=
V_{ah}\cos\psi + \hat W_N
$$

$$
V_{E,\mathrm{est}}
=
V_{ah}\sin\psi + \hat W_E
$$

where:

- $V_{ah}$ is calibrated horizontal airspeed,
- $\psi$ is geographic canopy heading in the model,
- $\hat W_N$ is estimated North wind component,
- $\hat W_E$ is estimated East wind component.

The corresponding position propagation is:

$$
x_{k+1}
=
x_k + V_{N,\mathrm{est}}\Delta t
$$

$$
y_{k+1}
=
y_k + V_{E,\mathrm{est}}\Delta t
$$

These relations follow the reduced parafoil kinematic model. They must not be
used as the sole operational propagator until payload heading is shown to be a
usable proxy. A bounded hold of the latest valid GNSS ground velocity provides
a simple initial between-fix propagation path.

---

# 12. Vertical Propagation

## 12.1 Barometric Vertical Navigation

**Status: Selected for V1; calibration and thresholds TBD**

The BMP280 pressure measurement is the primary source for vertical navigation,
deployment, and terminal-height decisions. The flight software converts it to
height above the target ground reference and supplies validity and a monotonic
timestamp. A launch-site GNSS altitude calibration alone does not establish
the target-ground elevation if the two sites differ.

Between new barometric samples, a bounded propagation may use an estimated
Down velocity:

$$
z_{k+1}
=
z_k + V_D\Delta t
$$

where $V_D$ is a validated sink-rate estimate. The filter setting and sensor
installation delay must be checked for deployment and flare timing.

---

## 12.2 GNSS Vertical Velocity

**Status: Optional cross-check**

GNSS Down velocity may be logged and compared with barometric vertical motion,
but is not assumed to be the V1 operational vertical source.

---

# 13. Accelerometer-Based Velocity Propagation

## 13.1 Operational Use

**Status: Future**

Accelerometer-based inertial propagation will not be used as the primary navigation method in V1.

A full inertial solution would require:

- body-to-navigation frame transformation,
- gravity compensation,
- accelerometer bias handling,
- rotational acceleration handling,
- treatment of payload pendulum motion.

This is especially relevant because the IMU is mounted on the suspended CAN-7USAT payload rather than directly on the parafoil.

---

## 13.2 Parallel Test Propagator

**Status: Planned for testing**

During tests, an accelerometer-based velocity propagator should run in parallel with the nominal wind-model-based propagator.

At each new GNSS velocity fix, compare:

$$
\mathbf{V}_{\mathrm{wind-model}}
$$

and

$$
\mathbf{V}_{\mathrm{IMU}}
$$

against:

$$
\mathbf{V}_{\mathrm{GNSS}}
$$

The prediction errors can be logged and compared.

The better-performing approach may later be incorporated into the operational estimator.

---

# 14. Wind Estimation

## 14.1 Wind Measurement

**Status: Frozen**

The project owner agreed to retain the existing model for V1: configured
constant horizontal airspeed, payload heading as the provisional canopy proxy,
straight-flight gating, and EMA filtering. Airspeed, gate thresholds, and EMA
coefficient remain configurable; their numerical flight values are not selected.
The approved wind freshness interface is implemented: last_update_ms records
the GNSS measurement timestamp of the last accepted wind update. Rejected
measurements do not refresh it. Check has_estimate before computing age; a zero
timestamp or zero wind vector can be valid. Retain the last accepted wind without
expiry or age-only reassessment. Keep the timestamp for logging and validation;
GNSS validity remains separate.

Wind is calculated only when a fresh GNSS ground-velocity measurement is available.

The horizontal wind measurement is:

$$
W_N
=
V_{g,N}^{GNSS} - V_{ah}\cos\psi
$$

$$
W_E
=
V_{g,E}^{GNSS} - V_{ah}\sin\psi
$$

The latest recent, valid payload heading is used as a provisional canopy proxy.
This measurement is not independently verified by the wind EMA.

The wind estimator must never use propagated velocity as its measurement input.

The formulation is consistent with the existing DCS model.

---

# 15. Wind-Update Gates

**Status: Frozen concept, thresholds TBD**

A wind measurement is accepted only during approximately straight flight.

All gates must pass.

The initial conditions are:

$$
|r| < r_{\mathrm{gate}}
$$

AND

$$
|u| < u_{\mathrm{gate}}
$$

AND

$$
|\dot{\chi}_g| < \dot{\chi}_{\mathrm{gate}}
$$

where:

- $r$ is heading rate,
- $u$ is normalized steering command,
- $\chi_g$ is GNSS course over ground,
- $\dot{\chi}_g$ is GNSS course rate.

The numerical thresholds are TBD and will be tuned experimentally.

---

# 16. GNSS Course Rate

**Status: Frozen**

Course rate is updated only when a fresh GNSS solution arrives.

Because course is angular:

$$
\Delta\chi
=
\operatorname{wrap}
\left(
\chi_k - \chi_{k-1}
\right)
$$

The course rate is then:

$$
\dot{\chi}_g
=
\frac{\Delta\chi}{\Delta t}
$$

where $\Delta t$ is calculated using system time between the two fresh GNSS fixes.

The wind estimator therefore retains internally:

```text
previous GNSS course
previous GNSS system time
```

---

# 17. Wind Filtering

## 17.1 Initial Implementation

**Status: Selected for V1; coefficient TBD**

The CDR filters accepted wind measurements to reduce the effect of payload
oscillation on the wind estimate:

$$
\hat{\mathbf W}_k
=
\alpha\mathbf W_{\mathrm{measurement},k}
+(1-\alpha)\hat{\mathbf W}_{k-1}
$$

The estimate starts at zero wind and the first accepted measurement is also
EMA-weighted. A single scalar
$\alpha$ is used for both horizontal components and remains configurable.
Rejected samples leave the estimate unchanged.

This filter does not remove an unknown persistent payload-to-canopy heading
offset and does not provide a filtered heading for the control loop.

---

# 18. Initial Wind State

**Status: Provisional**

Before the first accepted wind measurement:

$$
\hat W_N = 0
$$

$$
\hat W_E = 0
$$

Thus the initial navigation/planning assumption is zero wind.

---

# 19. Wind Estimation During Loiter

**Status: Future**

Wind estimation during turning is excluded from the initial straight-flight estimator.

However, circular-flight wind-estimation methods may be useful during the energy-management loiter phase.

When loiter guidance is designed, investigate whether the circular trajectory can be used simultaneously for:

- altitude dissipation,
- wind identification.

This is intentionally deferred until loiter design.

---

# 20. Data Ownership

## 20.1 `DcsConfig`

**Status: Frozen concept**

Contains fixed or tunable parameters such as:

```text
Vah

guidance parameters
planning radii
controller gains

wind gates

servo calibration
```

The current DCS mathematical formulation already identifies the main quantities requiring calibration or tuning.

---

## 20.2 Navigation Runtime Context

**Status: Frozen concept**

Private navigation bookkeeping may include:

```text
previous GNSS time
previous GNSS course
previous GNSS system time
last propagation system time
```

These should remain internal and should not be part of the public navigation state.

---

## 20.3 Navigation State

**Status: Frozen concept**

Contains the current GNC-relevant navigation quantities:

```text
target-relative Cartesian position
estimated ground velocity
latest heading
latest heading rate
```

---

## 20.4 Wind State

**Status: Frozen concept**

Contains:

```text
estimated North wind component
estimated East wind component
```

---

# 21. V1 Navigation Flow

**Status: Frozen**

The nominal navigation flow is:

```text
GNSS
  |
  v
Check GNSS time for new fix
  |
  v
Convert position to local NED
  |
  +--> Override propagated position
  |
  +--> Store trusted GNSS velocity
  |
  +--> Offer sample to gated wind EMA
  |
  +--> Override estimated velocity
```

Between GNSS updates:

```text
Latest valid GNSS ground velocity
        |
        v
Bounded horizontal propagation
        |
        +
Barometric height / validated sink estimate
        |
        v
Propagate Cartesian position
```

---

# 22. Explicitly Deferred Features

The following are deliberately excluded from the first implementation.

## Future

- full inertial navigation,
- Kalman filtering,
- canopy-mounted operational attitude sensing,
- full rotational dynamics,
- GNSS/ADCS delayed-measurement synchronization,
- backward propagation to measurement epochs,
- detailed degraded-mode optimization beyond basic stale-data checks,
- turn-based wind estimation,
- IMU-assisted operational velocity estimation.

These should be added only when simulation or test data demonstrates a need.

---

# 23. Next Architecture Topic

The project owner approved the full descent sequence: deployed/stabilized
readiness, path generation, transit to the energy-management point, height
evaluation, optional loiter and reconnection, final approach, insufficient-height
direct-to-target fallback, and terminal flare. GUIDANCE_DESIGN.md records that
decision and the application details still requiring discussion. No planner or
guidance implementation is implied by this phase-level approval.

The fallback final-approach bearing defaults to the current position-to-target
direction at plan generation, held for that plan; a manual bearing override is
available as a design option. An available strong estimate selects an into-wind
approach; an available weak estimate uses fallback with measured compensation.
Retain accepted wind without expiry. Before any estimate exists, use fallback
with an explicit zero-wind planning assumption.
GUIDANCE_DESIGN.md records these approved decisions; thresholds and
remaining transition rules are still to be agreed.

Path representation, segment/LOS progress, candidate ranking and the initial
20% turning height-loss allowance are recorded in GUIDANCE_DESIGN.md. A request
first reassesses the remaining current path; replacement generation is needed
only if that path no longer works. Horizontal distance from the active path
greater than X metres or remaining-height shortage greater than Y metres
requests this check. These are configurable thresholds with no selected values.
The height trigger uses the remaining height budget, not a prescribed 3D track.

**Next: wind triggers, reassessment thresholds and remaining feasibility details.**

Approved configurable initial test timing uses separate 1-second continuous
persistence timers for horizontal error and height shortage, reset independently
when their conditions clear, and a 2-second minimum ordinary reassessment
interval. Target passage requests immediate reassessment once per plan and
bypasses that interval. Active-path tracking continues during assessment.
Wind changes use 30% vector deviation when both estimates are strong and
weak/strong transitions otherwise, skipping percentage checks when both are
weak. Use a separate 1-second persistence timer and the ordinary 2-second
interval. Compare against wind at the last completed assessment (initially
plan generation), refreshing that reference even when the path is retained.
The first accepted wind estimate requests reassessment. Off-path recovery-cost
assessment remains pending. dcs_config.h now holds the frozen flight parameters,
and dcs_flight_config assembles the navigation, attitude and runtime configuration
used by the FSW. Many physical values remain unvalidated assumptions.
dcs_dubins.h/.c now implements the six Dubins geometry families with fixed
three-segment output and length/turn metrics. The planner interface selects
final approach internally; insufficient height is a no-feasible-approach failure
reason. dcs_planner_generate now selects final course, evaluates six routes to
qE plus the combined target straight, predicts time/height, and ranks candidates.
Guidance and reassessment are implemented in dcs_guidance.h/.c. See
GUIDANCE_DESIGN.md for scope and host validation.

The agreed height-tolerance percentage applies to predicted remaining loss,
excluding reserve. Numerical turn/length equality tolerances are separate.
Arc-time integration uses 32 Simpson intervals per arc, with numerical tie
tolerances frozen in dcs_config.h. Starting near the target does not automatically shorten
final approach. If the energy marker would precede the path start, adjust the
route to reach a valid energy-management point instead of clamping the marker.
The approved construction connects Dubins geometry to qE = -(final_length +
energy_upstream) * final_course_unit_vector, arriving along that course, then
continues straight through final entry to target. This replaces placing qE
upstream along a previously generated Dubins route. The two collinear legs
share one segment; planner output contains both energy and final-entry markers.
At exactly the target without an override or strong wind, use the into-wind
course for any nonzero estimate, otherwise current usable GNSS ground course.
These selection policies are implemented by the planner, not the geometry-only
layer. Guidance handles path progress and marker decisions internally.

The owner approved one persistent guidance state and one bounded update for
progress, direct marker decisions, modes, reassessment and LOS guidance. The
planner remains a calculation helper; heading/brake control remains separate.
This supersedes the proposed event queue/acknowledgment and old-plan handoffs.
Handle detected crossings internally and remember handling through mode/progress.
Valid replacements activate immediately and obsolete markers cease to apply;
failed replacement preserves the current route. Logging does not gate control.
See GUIDANCE_DESIGN.md; this runtime is implemented without an event queue.

The main FSW mission state explicitly starts DCS when secondary descent begins;
DCS does not infer deployment or duplicate that state machine. Once started,
DCS owns its settling/identification/guidance sequence, flare and degraded-
navigation actuator behavior. Navigation continues a held-velocity propagated
state after GNSS becomes stale and reports freshness separately. This simple
propagator cannot reproduce turning motion during a long outage.
Use barometric measurements to correct available height. Controller feedback
can come directly from attitude heading/rate; guidance supplies the reference
(proposed wiring). Phase-aware replacements go to qE before energy management
and qA after it; both planner modes are implemented.
See CIRCLE_WIND_ESTIMATION_DESIGN.md for the requested identification research:
GNSS velocity-circle centre estimates wind and radius estimates maneuver
horizontal airspeed. Turning/damaged-flight speed is not automatically healthy
trim. A subsequent settled trim leg estimates |Vground-W|. The bounded estimator
and commanded sequence are implemented with configurable gates; their flight
parameters remain unidentified.
