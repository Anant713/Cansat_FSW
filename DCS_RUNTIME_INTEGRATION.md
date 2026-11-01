# DCS runtime integration

## Call ownership

The main flight-software state machine owns the mission transition into
secondary descent. On that transition it calls `dcs_runtime_start` once. It then
calls `dcs_runtime_update` from the cooperative super-loop while secondary DCS
is active. DCS does not infer HDRM deployment and does not create a second
mission state machine.

The runtime sequence is:

```text
CANOPY_SETTLING
  -> IDENTIFICATION_TURN
  -> TRIM_SETTLING
  -> TRIM_SAMPLING
  -> GUIDANCE
  -> FLARE (latched)
```

Invalid or insufficient barometric height and identification timeouts abort the
manoeuvre. Identification failure is not a blocking state. The runtime raises
`identification_failed`, loads the configured fallback airspeed/wind, and
continues into guidance.

## Input wiring

For every update, populate `DcsRuntimeInput` from one coherent FSW snapshot:

- monotonic `now_ms`;
- target-centred navigation position, ground velocity and barometric height;
- separate horizontal/vertical validity and propagated-position status;
- fresh-GNSS flag, GNSS measurement epoch and monotonic acquisition timestamp;
- attitude heading, geographic heading rate and heading validity.

Before attitude fusion, apply calibration in each sensor's native axes and then
use `dcs_sensor_transform` to map BMI270 and IIS2MDC vectors into the payload
frame. The magnetometer calibration must precede its left-handed-native to
payload mapping. See `SENSOR_AXIS_CONVENTIONS.md`.

Set `has_fresh_gnss_velocity` only for a new receiver velocity measurement.
Never mark held or propagated velocity as fresh. Circle fitting and trim-speed
sampling accept unique fresh epochs only.

## Output wiring

`DcsRuntimeOutput` contains the mode, identification status, wind/airspeed,
guidance diagnostics, normalized steering/brake demands, and independently
mapped left/right pulse widths. The FSW PWM driver applies those pulse widths;
the portable module contains no timer or HAL access.

Invalid heading feedback commands neutral brakes immediately. A flare request
latches the runtime in flare and applies the configured equal brake demand;
normal heading control does not resume.

## Controller basis

The controller is based on the DCS note's reduced first-order heading-rate
response, rather than copied solely from the CDR:

```text
u = Kp * wrap(desired_heading - measured_heading)
    - Kd * measured_heading_rate
```

The saturated command is rate limited and allocated to one brake. Positive
command pulls the right brake; negative command pulls the left. No integral or
unidentified heading-rate feedforward is used.

## Servo calibration

Each installed servo has four pulse settings:

- neutral pulse;
- maximum safe pull pulse;
- absolute minimum pulse;
- absolute maximum pulse.

Endpoint order represents direction, so a mirrored servo can have a maximum
pull pulse below its neutral pulse. The mapping is linear only between the two
measured installed endpoints:

```text
pulse = neutral + normalized_pull * (maximum_pull - neutral)
```

The frozen assumption is 500 us neutral and 1000 us at the 45-degree pull
endpoint on both MG996R channels, with an absolute 500-2500 us bound. This has
not been established from installed-linkage measurement. Before arming, the
crew must at minimum verify neutral direction, pull direction, and absence of
binding on both channels.

## Frozen flight configuration

`dcs_flight_config_load` supplies the single configuration selected for this
flight. It includes:

- horizontal trim speed, sink rate and turn/planning radii;
- reserve, tolerance, qE/qA distances and LOS look-ahead;
- identification brake command, settling/timeout/height gates and fit gates;
- controller gains, maximum command and command-rate limit;
- each servo's installed neutral and maximum-pull pulse;
- flare height and symmetric brake demand;
- navigation freshness, attitude validity and wind-estimator gates.

The target is 26.717535 degrees North, 84.301543 degrees East. Save the complete
configuration and Git revision with the flight logs so every planning and
actuator decision can be reconstructed.

The attitude configuration applies +0.4720 degrees east magnetic declination
from WMMHR-2025 for 2026-11-02 at the target and 60 m GPS elevation. The
configured 25.25 magnetic-magnitude gate remains tied to the calibration
pipeline's output units; the WMM physical total field of 48.4249 microtesla is
not substituted unless the FSW confirms it supplies calibrated vectors in
microtesla.
