# Drone-drop validation register

Created: 2026-10-27. Current integration target: final mission launch near
1000 m, primary descent, and secondary parafoil/DCS operation from about 600 m.

This is the separate record of assumptions to check using collected data. Add
new assumptions here as design discussions continue. Design approval and host
test success do not constitute flight validation. On 2026-11-01 the owner
reported a drop with a broken servo mount, substantial unintended brake-line
pull and continuous small circles. The subsequently supplied Test2_3010 raw
logs have now been analyzed; see the linked result below. This is mechanical-
failure evidence, not healthy-flight validation. All entries below remain open.

Related documents:

- [Guidance decisions and pending design](GUIDANCE_DESIGN.md)
- [Fusion application behavior](ATTITUDE_FUSION_DESIGN.md)
- [Bench procedures](ATTITUDE_BENCH_TESTS.md)
- [Overall architecture](ARCHITECTURE.md)
- [Circle-based identification research](CIRCLE_WIND_ESTIMATION_DESIGN.md)

## Status and evidence rules

- **Baseline:** chosen model or behavior to implement/use; accuracy is unproven.
- **Initial setting:** chosen numerical starting value to tune from data.
- **Proposed:** discussed but not yet approved; recording it here is not approval.
- Close an entry only with identified logs, analysis, and the conditions under
  which the conclusion holds. Otherwise record supported, contradicted, or not
  observed, and keep the remaining uncertainty explicit.
- Do not infer validation of a whole flight regime from one successful drop.
  Record vehicle configuration, height, wind information, and manoeuvres observed.
- Architectural changes resulting from these findings still require discussion
  with the project owner before implementation.

## Minimum useful dataset

### Analyzed run: Test2_3010, damaged mount

See [detailed analysis](Flight_Data/Analysis_Test2_3010/REPORT.md) and its source
hashes, rejection audit, offline calibration candidates, sensitivity checks and
plots. The analysis does not change flight code or approve flight parameters.

- A01/A02: CDR BMI270/IIS2MDC models and logger-raw units are recorded. Gyro
  zero-rate candidates are estimated from quiet blocks; absolute axis/heading
  and payload/canopy relationship remain unverified. Representative full accel
  affine calibration is not identifiable from the available poses.
- A03: 84.6% of dense damaged-descent samples pass the 0.8-1.2 g magnitude band.
  This is not evidence that every passing vector is a valid gravity reference.
- N01/I01: accepted GNSS and sensor cadence/gaps are quantified. A 2 s constant-
  velocity prediction has about 9.27 m 95th-percentile disagreement with later
  positions from the same receiver; this is not independent position truth.
- N03: ground pressure-height scatter is about 0.10 m, but the pressure/GNSS
  relative-height maxima are about 93 m versus 41 m with different timing. The
  owner independently confirmed that the drone reached 100 m; that supports a
  roughly 7 m pressure-height shortfall in this run but does not explain the GNSS
  timing/amplitude discrepancy or identify one correction model.
- W01-W04: velocity-circle fits provide provisional wind near 4.5 m/s, while
  maneuver radii vary substantially between windows. No healthy trim is inferred.
- V01-V05: about 4.43 m/s damaged-descent sink and large body rates are observed;
  healthy sink, turn loss, radius, actuator mapping and gains remain open.
- G08: no settled touchdown/impact record or known flare actuation exists.

290/3327 GNSS rows are quarantined, including five malformed-coordinate rows
that pass reconstructed checksums. This supports schema/range/continuity checks
in addition to checksums, without establishing the original corruption cause.
The supplied different-PCB mag calibration fails direct norm comparison;
an offline hard-iron candidate improves consistency but validates no true yaw.

Log source measurements and estimator inputs, not only final telemetry outputs.
Rates and the telemetry/logging format remain integration decisions with the FSW
developer. These requirements do not imply a logger has been implemented here.

| Group | Data to preserve |
| --- | --- |
| Run identity | Flight/run ID, Git revision, full configuration, calibration identity, payload/canopy configuration, mass, release height, target-ground reference |
| Time and freshness | Measurement timestamps on a common monotonic clock; GNSS epoch; receive time where available; sensor validity, sequence/overrun indicators, estimator update intervals, processing/logging gaps |
| Sensors | Calibrated accel/gyro/mag vectors with units and axes; pressure, barometer temperature and derived height; IMU temperature if available; original measurements/calibration metadata where FSW makes them available |
| GNSS | Position, North/East velocity or speed/course, per-field validity, quality/accuracy indicators if available, and actual achieved update rate |
| Attitude | Quaternion, heading, heading rate, availability/validity, reference-used flags, angular residuals, Fusion startup/recovery/overrange flags |
| Wind | Raw wind measurement, EMA result, acceptance/rejection reason, estimate timestamp/age, configured airspeed and EMA/gate settings |
| Vehicle events | Release, deployment, observed stabilization, mission/guidance mode changes, flare command and touchdown; timestamped video event markers where available |
| Actuation | Requested and applied left/right brake commands, saturation/rate limiting, servo position/feedback if available, event timing |
| Planning, once implemented | Selected path geometry/ID, active segment and progress, LOS point, commanded course/heading, wind used/assumed, predicted remaining time/height, reserve/tolerance, replan reasons and outcomes |
| Independent observations | Synchronized video showing canopy and payload where possible; measured release/landing positions, surveyed/known heights or other independent references, and ground wind observations with location/time |

Ground wind is contextual evidence, not automatically the wind at canopy height.
Keep source timestamps: aligning inputs by log-row time alone can hide delay.

## First-drop priorities

The most useful early results are: stable straight-glide sink rate, turn-related
sink increase, turn response/radius, payload-to-canopy motion, sensor/estimator
continuity through deployment, and actual GNSS/update timing. Extract these from
the manoeuvres actually observed; this document is not a flight-command script.

## Attitude and sensor assumptions

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| A01 | Baseline: calibrated measurements use correct units/axes and remain useful after startup. Calibration is owned by FSW. | Bench known-pose/rotation checks; record calibration, temperature and time since power-on. Calibration owner checks bias changes across power cycles. Relate preflight baseline to drop observations. | Calibration validity, mounting mapping, startup settling and recalibration needs; no duplicate calibration in attitude code. |
| A02 | Baseline: payload heading is a useful proxy for mean canopy direction despite oscillation, twist and lag. | Video with visible payload/canopy orientation where possible, synchronized gyro/mag/attitude and brake commands. Estimate oscillation frequencies, mean offset, lag and changes during turns. | Whether proxy is adequate for wind/control; whether additional heading treatment or mounting changes need discussion. GNSS course alone is not canopy-heading truth. |
| A03 | Initial Fusion settings: gain 0.5, accel band 0.8-1.2 g, accel/mag angular rejection 10 degrees, recovery 5 s. | Inspect correction availability/use and residuals through release, deployment, glide and turns. Replay identical input streams with alternatives; compare against independent pose evidence where available. | Gains, bands, recovery timeout and caller veto use. Stable output alone does not prove correct orientation. |
| A04 | Baseline: gyro propagation can bridge rejected/missing accel/mag references for useful durations. | Real stationary gyro-coast bench tests plus flight rejection durations, temperature, vibration and gyro range usage. Compare attitude before/after reference return; quantify error only where independent reference exists. | Acceptable reference outages and whether current recovery/validity policies are adequate. Synthetic gyro tests do not characterize hardware drift. |
| A05 | Provisional: current magnetic magnitude/heading-geometry checks and recovery behavior flag important loss of heading information. | Magnetic field magnitude/direction and electrical activity; reference flags and video around large tilts. Examine gyro clipping and the startup ramp reentered after overrange, when Fusion angular rejection is bypassed. | Magnetic gates, heading validity, gyro range and overrange handling. Existing numerical cutoffs are not established flight limits. |
| A06 | Baseline: known-pose initialization followed by continuous estimation survives the actual pre-release and deployment history. | Record initial reference, carriage to release, continuity, missing samples and deployment events. Inspect resets and heading-reference expiry. Absolute accuracy needs an external orientation reference. | Initialization timing, continuous operation, maximum gap and reference-age policies. Never silently reload launch-pad pose during flight. |

## Navigation and altitude assumptions

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| N01 | Baseline: holding fresh GNSS velocity between fixes is adequate, including turns. | Actual update intervals, velocity/position quality and latency, turn intervals and synchronized position reference where available. Replay the hold and measure prediction residuals at subsequent fixes. Such residuals also include GNSS noise and are not independent truth. | Acceptable between-fix error, age limits and whether another propagator is justified. A requested 5 Hz rate is not proof of achieved rate or accuracy. |
| N02 | Baseline: GNSS ground-track direction is usable for initial Dubins orientation and course-rate gates. | Speed/course scatter, velocity quality and delay in straight flight and turns, especially low speed. Compare position trends over time; use external trajectory evidence when available. | Minimum usable speed, course-rate gates and temporal alignment with attitude. |
| N03 | Baseline: BMP280-derived target-relative height is useful with sufficiently small noise, offset and delay. | Pressure, temperature, derived height, GNSS altitude as a cross-check and independent known-height/video events where available. Examine deployment/turn pressure transients and venting delay. Verify launch-versus-target ground elevation reference. | Barometer filtering/reference, freshness, height tolerance and flare/energy timing. Repeated barometer samples are not independent altitude truth. |

## Wind and airspeed assumptions

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| W01 | Baseline: one configured horizontal airspeed represents ordinary glide adequately. | Ground velocity over different headings, payload/canopy direction evidence, brake state and independent wind/airspeed reference if available. Ground velocity equals air velocity plus wind; one heading alone cannot separate both unknowns. | Nominal airspeed and whether its variation is small enough for the model. Fitting wind with the same assumed airspeed is not independent validation. |
| W02 | Baseline: straight-flight gates reject misleading wind samples and still permit enough updates. | Raw/filtered wind, each gate result, brake transients, payload oscillation and GNSS course. Check acceptance rate, rejection duration, and estimate age over time. | Heading/course-rate, brake and speed thresholds; whether useful wind is obtained before energy-management decisions. |
| W03 | Baseline: an EMA reduces varying errors sufficiently; a single horizontal wind vector remains useful over the planning interval. | Wind residual spectra/lag, raw and filtered estimates over height/time/headings, and independent observations where possible. Separate persistent heading/airspeed bias from fluctuating estimates. | EMA coefficient, retained-wind error over update gaps and reassessment threshold. EMA cannot remove a persistent model bias. |
| W04 | Baseline: weak/no-estimate fallback and retaining accepted wind without expiry remain useful. | Log which wind state selected each plan, assumed versus measured wind, observed drift and approach/touchdown errors. Compare planning predictions with outcomes. | Weak-wind threshold, retained-wind errors and suitability of zero-wind assumption before the first estimate plus position-to-target fallback bearing. These are assumptions, not measured calm conditions. |

## Glide, turns and actuators

### Requested circle identification and reported damaged drop

Extend W01-W04/V01-V05 evidence to the GPS velocity-circle method described in
CIRCLE_WIND_ESTIMATION_DESIGN.md. Preserve fresh GNSS North/East velocity and
measurement/receive times, quality, pressure/height, attitude, commanded and
actual brake configuration where observable, and failure timing/video. Do not
fit propagated or duplicated velocities as independent observations.

The bounded onboard fit now checks unique fresh GNSS epochs, direction coverage,
velocity spread, radius bounds and radial RMS. Continue inspecting conditioning,
sensitivity and agreement across consecutive windows offline. A visually
circling payload is not itself evidence of a valid velocity circle. Host tests
cover ideal circles and degenerate line data; noisy, sparse and changing-speed
replay remains required. A small residual does not establish independent wind
truth or healthy trim equivalence.

To identify healthy horizontal trim speed, the proposed additional evidence is
a settled, normal brake-setting leg after wind identification, using |Vground-W|.
Wind must remain representative between stages. The damaged flight alone cannot
establish healthy trim, sink/turn settings or command-to-brake response; the
broken mount means actual line pull differed from commanded actuation. No such
healthy trim leg is established here. The runtime now uses configurable
settling, height and timeout limits and a flagged fallback when identification
fails. Their numerical flight values remain unvalidated.

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| V01 | Working model: nominal straight-glide sink rate V_D can be treated as constant. | Measured height loss divided by elapsed time over settled straight intervals; report spread and dependence on load, wind, speed and brake state. Cross-check barometric transients before interpreting variation as sink change. | V_D input and prediction uncertainty. Earlier preference for descent below 20 m/s is not a measured result or a flare-performance specification. |
| V02 | Initial setting: turns consume 20% more height per unit time than straight glide. | Compare comparable settled straight and turning intervals, including left/right turns and brake demand. Estimate m_turn = sink_turn / sink_straight - 1; also compare predicted versus observed cumulative height loss. Separate deployment and flare. | Retain, increase or reduce 0.20; assess whether one percentage covers turn radius/brake variation. The percentage is not yet flight-validated. |
| V03 | Baseline: a conservative fixed planning radius covers real turn capability and transient response. | GNSS ground-track curvature, airspeed/wind information, commands, and canopy/payload response through turn entry/exit. Measure both directions and lag. Ground-path radius in wind is not obtained directly from payload gyro Z. | Minimum sustainable radius, R_plan, loiter radius and whether curvature transitions need larger margins. |
| V04 | Implemented control model: heading/brake response can be represented simply with bounded commands and rate limits. | Time-aligned desired heading, payload/canopy motion, heading rate, requested/applied brakes and saturation. Observe delays, overshoot, asymmetric response and command reversals. | Controller gains, actuator mapping, command-rate limits and whether single-sided steering is adequate. |
| V05 | Baseline approximation: steering-induced airspeed changes and turn-entry/exit sink transients are adequately covered by the simple model and margin. | Speed/height-loss histories around commands, not only steady portions. Compare predicted integrated loss with actual loss for whole manoeuvres. | Whether constant airspeed plus the turn multiplier is sufficient; changes require discussion rather than silently adding a more complex model. |

## Planning, guidance and terminal assumptions

These components are implemented and host-tested. Their physical behavior and
parameters remain open until simulation, bench calibration and flight evidence.

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| G01 | Baseline: wind-aware path timing and straight/turn sink accounting predict remaining height usefully. | At each plan/replan and energy check, save remaining segment geometry, wind, time/height prediction and measured subsequent consumption. Compare cumulative prediction error by segment type. | Height classification and feasibility model. Include 20% turn loss consistently in loiter, reconnection and final-path budgets. |
| G02 | Baseline: the remaining-route budget includes a separate reserve; percentage classification tolerance applies to predicted remaining loss excluding reserve. | Distribution of height-prediction errors, sensing delay, turn/reconnection transients and remaining approach consumption. Test decisions in replay. | Owner-suggested ranges are 10-20 m reserve and 1.5-2% tolerance; exact flight values are unset. Keep the metre-based Y reassessment threshold distinct. Avoid treating the turn-loss allowance as the same reserve twice. |
| G03 | Baseline: LOS over lines/arcs is trackable with a suitable look-ahead distance. | Active geometry, reference/LOS points, cross-track error, desired course/heading, actual trajectory, brake state and response lag. Measure arc cutting, segment transitions and sustained error. | Look-ahead, radius margin and replan thresholds; a planned arc does not guarantee the flown radius. |
| G04 | Baseline: end-plane and directed angular-progress rules identify segment/marker crossings reliably. | Timestamped progress, segment changes, GNSS gaps/jumps and video/trajectory evidence. Check +/-pi crossings and backtracking; use deterministic replay for cases not flown. | Progress tolerances, event persistence and handling of poor position data. No advancement should be inferred solely from heading rotation. |
| G05 | Baseline: late loiter and exit/reconnection preserve enough height for final approach. | Predicted/actual full and partial orbit loss, exit configuration, replacement-path cost, reserve and subsequent approach. Reevaluate exit/reconnection cost when its geometry changes. | Loiter entry/exit logic, radius, energy-point placement and reserve over the final 600 m parafoil phase. |
| G06 | Baseline: final entry distance and into-wind/fallback bearing allow a useful stabilized final approach. | Final course/heading, wind state, entry position/height, tracking error, touchdown position and independently observed landing reference where possible. | Final length, energy-point distance and weak-wind fallback suitability. Four-slot geometry is a software property; actual trackability needs flight evidence. |
| G07 | Baseline: target-overrun replanning can identify a feasible return from current remaining height. | Save overrun event, candidate costs, feasibility outcome and actual height consumption if a return occurs. Use replay/simulation when not observed; distinguish from the low-height continue-final fallback. | Return feasibility/reserve and replan scheduling. Do not mark it verified without observing or separately testing the relevant conditions. |
| G08 | Planned terminal flare has an appropriate trigger, brake level and response. | Trigger-height source/delay, commanded/applied symmetric brakes, sink/ground speed, video and touchdown outcome. | Flare authority, trigger and demand. Values remain undecided; crossing the horizontal target alone is not a flare trigger. |

### Near-target and short-connection scenarios

Extend G01/G05/G06 software and simulation coverage to initial high-altitude
starts near the target, short connections with an upstream energy marker before
path start, and initial exact-target starts with weak/no wind and no override.
The approved construction generates a Dubins connection to qE, arriving along
the final course, then a straight through qA to target, rather than clamping an
upstream marker to the start. Proximity alone must not shorten final approach.
Validate its full wind/height budget including qE-to-qA against flight evidence.
At the exact target, after override/strong-wind handling, use any nonzero wind
direction even if weak; otherwise use usable GNSS ground course. Test the
sensitivity to weak-wind direction noise and GNSS course uncertainty. This is
distinct from existing final-line overrun. Candidate-planner host tests now
exercise these software cases; no corresponding flight behavior is validated.

For bounded arc-time integration, check numerical convergence against a finer
offline reference, including near wind-feasibility boundaries. Verify numerical
ranking ties independently of physical tolerance. Sample count and equality
tolerances are frozen flight assumptions: 32 Simpson intervals per
arc, 1e-5 rad turn equality, and 0.001 m length equality. Selected host-test
routes through wind/airspeed ratio 0.999 agree within 0.2% in time/loss with an
8192-midpoint-per-arc double reference. Analytical straight times and ranking
priorities are also checked. This does not bound all possible routes, establish
STM32 execution time, or validate the physical airspeed/sink/wind model. Broader
simulation and G01 flight-prediction validation remain open.

### Direct marker handling in one guidance update

The agreed simplified runtime supersedes event queues, external acknowledgment
and old-plan notification handoffs. Extend G04 host/replay coverage when guidance
is implemented: detected energy/final-entry/target crossings must directly
invoke the corresponding decision/transition, recorded in persistent mode and
progress. Test coincident markers, repeated calls without repeated handling,
invalid position, failed replacement preserving the route, and successful
replacement taking effect immediately without old markers acting on it.
Verify energy decisions precede later approach transitions. Logging may record
transitions but must not gate guidance. Position sampling still limits detection;
no arbitrary-GNSS-gap or power-loss guarantee is implied. No runtime marker
handling implementation has been tested yet.

### Wind reassessment threshold

The approved threshold is 30% vector deviation relative to reference wind
magnitude when both usable estimates are strong. Skip the percentage check
while both are weak; crossing between weak and strong requests reassessment.
The weak-wind threshold is unset. Validate these rules using W02-W04 evidence:
log both vectors, classifications, relative change where defined, and assessment
outcomes. Examine repeated weak/strong crossings due to noise and changes in
direction as well as speed. The approved reference is wind at the last completed
assessment (initially plan generation), refreshed even if the path is retained.
Use a separate 1-second persistence timer and the ordinary 2-second interval.
Retain the last accepted estimate without expiry; age alone causes no request.
Record age and rejected-update intervals to assess errors from retaining old
wind. The first accepted estimate requests reassessment of the initial zero-wind
plan. This is not validated.

The common dcs_config.h header contains frozen DCS_FLIGHT_* values. Many were
formerly arbitrary simulation values and remain unvalidated physical
assumptions, but are accepted for this flight by programme decision. Record the
full configuration and Git revision with each run. Acceptance for flight does
not constitute validation.

### Path reassessment thresholds

The agreed trigger structure requests reassessment when shortest horizontal
distance to the active path exceeds X metres or remaining-height shortage
exceeds Y metres. Current-path feasibility is checked before replacement.
The frozen X/Y values are 10 m and 5 m. The implementation uses separate
1-second continuous persistence timers for horizontal error and height shortage,
each reset when its condition clears, and at least 2 seconds between ordinary
reassessments. Target passage requests immediate reassessment once per plan,
bypassing that interval. These timing settings are not flight-validated;
wind changes use the same timing with a separate persistence timer.

Using the G01-G04 tracking and prediction evidence, replay threshold crossings
and log horizontal error, height shortage, trigger reasons, persistence state
if implemented, and reassessment/retain/replace outcomes. Check nuisance requests
from GNSS/barometer noise and delayed detection of sustained deviations. Assess
recovery travel from the actual off-path position rather than assuming immediate
return to the path. None of these thresholds or recovery estimates is validated.

## Integration and operating limits

| ID | Assumption or setting | Evidence and analysis needed | Decision informed |
| --- | --- | --- | --- |
| I01 | Baseline: the super-loop services acquisition, estimation and control frequently enough under real logging/telemetry load. | Source/update timestamps, gaps, sensor overruns, execution time, logging/telemetry activity and validity changes through the drop. | Actual update rate, timeout limits and whether bounded processing meets the target MCU budget. Host runtime does not establish STM32 timing. |
| I02 | Baseline: deployment/stabilization can be identified well enough to enable descent guidance. | Release/deployment/stabilization markers, video, acceleration/rate settling and acquisition continuity. | Readiness input, settling criterion and startup of descent GNC. Continuous attitude estimation is independent of this enable condition. |

## What these drops cannot establish alone

- Without independent attitude evidence, absolute attitude accuracy remains open.
  Agreement between Fusion and a heading-derived wind estimate is circular evidence.
- Without independent wind/airspeed information or justified multi-heading
  identification assumptions, separate airspeed and wind accuracy remains open.
- Short 100 m drops may not exercise full/partial loiter, high-altitude wind
  variation, long reference outages, temperature range, overrun returns, or all
  flare conditions. Use bench tests and replay where appropriate, and retain
  unobserved regimes as open rather than claiming general validation.
- Barometric height used both for prediction and outcome can hide common offset
  or delay errors; independent references are required for absolute accuracy.

## Per-run result record

For each run, add a dated result with:

1. Run ID, log/video paths, software revision, configuration and calibration.
2. Register IDs exercised and actual operating conditions/coverage.
3. Measured values, spread, reference used, timestamp alignment and uncertainty.
4. Supported/contradicted/not-observed conclusion for each exercised item.
5. Proposed parameter/model changes and project-owner decision before changes.

Agree numerical acceptance targets before grading performance. Keep raw logs
and configurations so alternative settings can be compared on the same data.
No acceptance thresholds or new flight algorithms are approved by this register.
