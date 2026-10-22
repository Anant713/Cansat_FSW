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

System time is the primary timebase used by DCS.

It is supplied by the main flight software using the system RTC/timekeeping infrastructure.

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
- Down velocity, assuming it can be obtained reliably,
- GNSS time.

Latitude and longitude should use sufficient precision for geodetic conversion.

After conversion into the local frame, single-precision floating-point values are expected to be sufficient.

---

# 8. ADCS Interface

**Status: Frozen**

ADCS owns attitude estimation.

DCS consumes only the quantities required for guidance and control:

- geographic heading $\psi$,
- heading rate $r$.

The DCS will not initially:

- propagate attitude independently,
- run full rotational dynamics,
- estimate attitude from raw IMU measurements.

The latest available ADCS heading is used whenever required by the DCS.

---

# 9. Navigation Architecture

## 9.1 Navigation Philosophy

**Status: Frozen**

GNSS updates more slowly than the desired navigation and control execution rates.

A translational propagator is therefore used between GNSS fixes.

Only translational quantities are propagated in V1.

Heading and heading rate remain ADCS-derived quantities.

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

Latest ADCS data:
    heading
    heading_rate
```

The navigation state must not derive velocity by differentiating corrected position.

---

# 10. GNSS Navigation Update

**Status: Frozen**

Whenever a fresh GNSS fix is received:

1. Convert GNSS latitude, longitude, and altitude to local Cartesian coordinates.
2. Override the propagated position with GNSS position.
3. Store the measured GNSS ground velocity.
4. Update the wind estimator using GNSS ground velocity and the latest available heading.
5. Override the current velocity estimate with the trusted GNSS velocity.
6. Continue propagation from the corrected state.

GNSS therefore acts as the absolute measurement source for position and velocity.

---

# 11. Horizontal Velocity Propagation

**Status: Frozen**

Between GNSS updates, horizontal velocity is estimated from:

- calibrated horizontal airspeed,
- latest ADCS heading,
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
- $\psi$ is latest geographic heading,
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

These relations follow the reduced parafoil kinematic model.

---

# 12. Vertical Propagation

## 12.1 Initial Approach

**Status: Frozen for V1 assumption**

For the initial firmware implementation, it is assumed that a usable Down-velocity measurement can be obtained from the GNSS receiver.

Vertical propagation is therefore:

$$
z_{k+1}
=
z_k + V_D\Delta t
$$

where $V_D$ is the latest measured Down velocity.

---

## 12.2 Barometric Vertical Navigation

**Status: Future**

Barometric altitude and/or vertical-velocity estimation may later be incorporated after:

- sensor testing,
- literature review,
- characterization of GNSS vertical performance.

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

The latest available ADCS heading is used.

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

**Status: TBD**

The initial implementation may use direct hard-gated updates:

$$
\hat{\mathbf W}
=
\mathbf W_{\mathrm{measurement}}
$$

whenever all gates pass.

This avoids adding filtering before test data demonstrates that filtering is required.

---

## 17.2 Optional EMA

**Status: Future / TBD after testing**

If GNSS-derived wind proves noisy, an exponential moving average may be added:

$$
\hat{\mathbf W}_k
=
\alpha \mathbf W_k
+
(1-\alpha)\hat{\mathbf W}_{k-1}
$$

A single scalar $\alpha$ will initially be used for both horizontal components.

---

# 18. Initial Wind State

**Status: Frozen**

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
  +--> Update wind estimate
  |
  +--> Override estimated velocity
```

Between GNSS updates:

```text
Latest ADCS heading
        +
Calibrated Vah
        +
Current wind estimate
        |
        v
Estimate horizontal ground velocity
        |
        +
Latest measured Down velocity
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
- independent heading propagation,
- full rotational dynamics,
- GNSS/ADCS delayed-measurement synchronization,
- backward propagation to measurement epochs,
- barometric sensor fusion,
- detailed degraded-mode handling,
- turn-based wind estimation,
- IMU-assisted operational velocity estimation.

These should be added only when simulation or test data demonstrates a need.

---

# 23. Next Architecture Topic

**Next: trajectory representation and path-manager design.**

This includes:

- Dubins arc representation,
- straight-segment representation,
- final-approach segment,
- energy-management point,
- loiter representation,
- active-segment tracking,
- interaction between the planner and LOS guidance.