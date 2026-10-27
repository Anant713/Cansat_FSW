# Fusion attitude integration review

Status: implemented and verified by host tests. The user approved the initial
bench settings and recovery behavior below. Hardware performance remains untested.

## Scope and implementation

The main FSW is being developed independently. Its app boundaries and shared
structures are preliminary, not fixed requirements. Preserve compatibility with
one-time initialization and repeated bounded calls from a cooperative super-loop;
retain estimator state between calls and advance only on new gyro measurements.
Actual sample timestamps, rates, units, and mounting axes must be agreed at the
integration boundary. A nominal acquisition period is not a timing guarantee.

Flight software owns acquisition and calibration. The attitude module receives
calibrated gyro, accelerometer specific force, and magnetic vectors in payload
axes. It runs continuously from the known launch-pad pose throughout flight.

The upstream Fusion AHRS core is retained unchanged at revision
`a8d7224f36a0ec82345ef49a3db50e65f8d3bab8` under third_party/Fusion, with its MIT
license and SHA-256 manifest. Application rules live in dcs_attitude. The separate
Fusion calibration and gyro-bias modules are not used. The core computes payload
orientation, not canopy orientation or attitude uncertainty.

## Core logic

1. Use the previous quaternion to predict reference directions in body axes.
2. Compare normalized acceleration with the predicted accelerometer reference.
3. Compare horizontal magnetic direction with its predicted direction.
4. Reject feedback with excessive angular disagreement, subject to upstream
   startup and recovery behavior.
5. Add accepted vector feedback multiplied by a gain to the gyro rate.
6. Integrate over the sample interval and normalize the quaternion.

## Implemented application flow

```text
Known launch-pad pose and timestamp
    -> configure NED convention
    -> set quaternion and skip startup convergence
    -> run continuously at gyro sample cadence

Calibrated vectors and timestamp
    -> ignore repeated timestamps
    -> check provider validity and finite values
    -> determine usable accel/mag references
    -> convert gyro rad/s to degrees/s and accel m/s^2 to g
    -> use zero vectors for withheld references
    -> Fusion update with the measured, approximately nominal sample interval
    -> output quaternion, geographic heading, heading rate, and diagnostic flags
```

Known geographic heading and output geographic heading require consistent
magnetic-declination conversion around Fusion's magnetic reference. Initialization
and output frames must match; declination must not be applied twice.

## Approved initial bench behavior and retained policies

- Keep a configurable magnitude band around 1 g before angular rejection.
  Near-zero acceleration cannot supply a gravity direction. The band also
  rejects strong acceleration, but passing it does not prove a gravity reference.
  The approved initial band is 0.8-1.2 g and remains configurable.
- Keep provider validity and existing caller veto flags as input permissions.
  Withholding a reference must not stop gyro propagation.
- Approved first bench-test settings based on upstream recommendations: gain 0.5,
  accel rejection 10 degrees, mag rejection 10 degrees, recovery timeout
  5 seconds. These are starting test values, not validated flight settings.
  Sample rate and gyro range must match acquisition configuration.
- Allow upstream recovery only on references admitted by application checks.
  Recovery reaccepts disagreeing data after prolonged rejection to limit gyro
  dependence. That data can still be misleading during sustained acceleration.
  An externally withheld zero vector stays unused even during recovery.
- Review the existing heading-reference timeout and large-gap recovery policy.
  Neither timeout establishes a quantified attitude accuracy bound. Never
  restore the launch-pad pose automatically during flight.

The prior magnetic total-field magnitude check remains configurable. It is an
earlier provisional policy, not an accuracy guarantee. The former 10%-of-field
horizontal threshold is removed; a zero cross product is withheld to prevent
normalizing zero. The prior heading geometry cutoff (horizontal projection
squared below 0.04) is retained. Neither is a new Fusion threshold.

Fusion settings are cached at initialization; keep the caller config unchanged
thereafter. Supply actual sample rate and gyro range. This adapter requires a
positive gain/recovery timeout and angular rejection limits in (0, 90] degrees
so accidental zeros do not disable the agreed rejection behavior. Recovery
sample counts must fit the upstream signed counter. Per-update dt comes from
the gyro timestamp. Upstream recovery counts updates at the configured nominal
rate; large scheduling deviations are not made harmless by supplying measured dt.

The caller supplies sufficiently fresh magnetic values and clears mag_valid
when stale. The sample API has one IMU timestamp, not independent mag-age logic.
Keep acquisition and scheduling details in the main FSW; no extra task is created.

Diagnostics distinguish reference availability from actual correction use, and
expose residual errors, accel/mag recovery, gyro-overrange recovery, and startup.
An unavailable reference has zero reported error qualified by its availability
flag. Upstream recovery flags can remain set while a reference is withheld;
correction-used flags identify what actually influenced that update. Numerical
orientation validity does not guarantee accuracy following gyro saturation.

Known-pose initialization skips startup once. Upstream overrange recovery can
subsequently reenter its startup gain ramp, during which angular rejection is
bypassed. Application magnitude/validity vetoes still apply. This is retained
upstream behavior, exposed through startup and overrange-recovery flags.

Compile both the wrapper and Fusion with FUSION_USE_NORMAL_SQRT. This selects
an upstream-supported option; no source modification is required.

The reviewed upstream initialized defaults disable angular rejection. Setting
recovery timeout to zero also disables angular rejection; it does not provide
indefinite rejection. Configure settings explicitly.

## Verification and drop data

Detailed proposed procedure: [ATTITUDE_BENCH_TESTS.md](ATTITUDE_BENCH_TESTS.md).
No hardware bench or drone-drop results have yet been collected in this work.
Host tests cover known tilted pose with nonzero declination, positive and
negative rotations about all axes, measured dt and timestamp wrap/duplicates,
invalid/missing references, near-free-fall noise, magnitude and angular rejection,
both prolonged-rejection recovery paths, withheld references during recovery,
gyro overrange, and invalid configuration. Navigation and wind host checks pass.

Before drops, check known-pose startup, stationary stability, rotation signs,
duplicate timestamps, missing references, low acceleration, and gyro overrange.
Log calibrated vectors and their timestamps, quaternion, heading, correction-used
and recovery/overrange flags alongside GNSS and brake commands. Preserve data for
replay with different settings. Drop data without independent attitude truth
supports consistency checks but cannot establish absolute attitude accuracy.

## Sources

- https://github.com/xioTechnologies/Fusion
- https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionAhrs.c
- https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionAhrs.h

The integrated source revision and per-file hashes are recorded in
third_party/Fusion/UPSTREAM.json. Verify offline with
`python tools/vendor_fusion.py --verify`.
