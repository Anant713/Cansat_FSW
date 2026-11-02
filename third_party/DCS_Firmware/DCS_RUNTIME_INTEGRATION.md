# DCS runtime integration

## Call ownership

The main flight-software state machine owns secondary descent and HDRM timing.
It calls `dcs_runtime_start` once only when the state is secondary descent and
`parafoilDeployed` is true. The runtime's 5 s `CANOPY_SETTLING` timer therefore
starts after the deployment pulse completes, not on entry to secondary descent.
With the FSW's 10 s HDRM pulse and this 5 s settling timer, the identification
turn cannot begin earlier than 15 s after entry to secondary descent.
It then calls `dcs_runtime_update` from the cooperative super-loop through
secondary descent and touchdown so a latched flare remains commanded. DCS does
not infer deployment and does not create a second mission state machine.

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
identified planning turn radius,
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

The frozen assumption is 556 us (approximately 5 degrees) neutral and 1000 us
at the 45-degree pull endpoint on both MG996R channels. The absolute software
bounds are also 556-1000 us, so DCS cannot request an angle outside the approved
5-45 degree range. This has
not been established from installed-linkage measurement. Before arming, the
crew must at minimum verify neutral direction, pull direction, and absence of
binding on both channels.

## Frozen flight configuration

`dcs_flight_config_load` supplies the single configuration selected for this
flight. It includes:

- horizontal trim speed, sink rate and fallback turn/planning radius;
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
IIS2MDC driver supplies calibrated vectors in microtesla, so the configured
magnetic-magnitude reference is the WMM physical total field of 48.4249
microtesla. The known launch pose is upright with physical Body +X facing true
North; payload/parafoil +X is initialized at +15 degrees.

During the 30-degree identification pull, the circle fit first estimates wind
and manoeuvre airspeed in velocity space. It then unwraps the ordered
wind-corrected velocity direction, divides angular sweep by elapsed GNSS time,
and calculates `turn_radius = airspeed / abs(turn_rate)`. Direction consistency,
sweep, fit residual and radius gates must all pass. A valid identified radius
replaces the planner radius for the current flight and lower-bounds the loiter
radius; otherwise the 20 m fallback remains.
