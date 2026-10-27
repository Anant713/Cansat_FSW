# Fusion attitude integration review

Status: Fusion selected by the user. Application behavior below is proposed
for agreement before replacing the current custom estimator.

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

Proposal: retain the upstream Fusion AHRS core with its license and a fixed
revision, and put application rules in dcs_attitude. Do not use Fusion's separate
calibration or gyro-bias module. The core computes payload orientation, not
canopy orientation or attitude uncertainty.

## Core logic

1. Use the previous quaternion to predict reference directions in body axes.
2. Compare normalized acceleration with the predicted accelerometer reference.
3. Compare horizontal magnetic direction with its predicted direction.
4. Reject feedback with excessive angular disagreement, subject to upstream
   startup and recovery behavior.
5. Add accepted vector feedback multiplied by a gain to the gyro rate.
6. Integrate over the sample interval and normalize the quaternion.

## Proposed application flow

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

## Decisions to agree before integration

- Keep a configurable magnitude band around 1 g before angular rejection.
  Near-zero acceleration cannot supply a gravity direction. The band also
  rejects strong acceleration, but passing it does not prove a gravity reference.
  Its numerical width remains to be agreed.
- Keep provider validity and existing caller veto flags as input permissions.
  Withholding a reference must not stop gyro propagation.
- Proposed first bench-test settings from upstream recommendations: gain 0.5,
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

The reviewed upstream initialized defaults disable angular rejection. Setting
recovery timeout to zero also disables angular rejection; it does not provide
indefinite rejection. Configure settings explicitly.

## Verification and drop data

Detailed proposed procedure: [ATTITUDE_BENCH_TESTS.md](ATTITUDE_BENCH_TESTS.md).
No hardware bench or drone-drop results have yet been collected in this work.

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

The implementation revision will be fixed at integration. These mutable links
identify source locations, not a pinned dependency.
