# Attitude bench tests

Status: the Fusion application is integrated and its deterministic host checks
pass. Physical bench tests, replay of actual flight logs, and drone drops remain
pending. The user approved initial bench values: gain 0.5, accel/mag rejection
10 degrees, recovery timeout 5 s, and accel magnitude band 0.8-1.2 g. These are
configurable test settings, not validated flight settings.

Run the integrated deterministic checks with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
```

The script builds/runs all three modules. The attitude cases implement the
synthetic-input checks below against the actual Fusion adapter. No bench-test
data logger or CSV replay application is claimed by these host checks.

## Equipment and recording

Use the assembled electronics, a laptop/logger, a marked sheet with headings
90 degrees apart, and a protractor or known-angle support. A video recording can
help establish event timing. Mark the payload axes used by the estimator.

Record one row per estimator update containing:

- sensor timestamps, update interval, and provider validity;
- calibrated gyro, accelerometer, and magnetometer vectors with explicit units;
- quaternion, roll, pitch, heading, and output validity;
- correction-used, acceleration/magnetic recovery, and gyro-overrange flags;
- configuration and software revision, recorded once per run.

Run with logging enabled to expose its effect on update timing. Preserve the
input stream for replay. Known poses establish static references; hand motion
and drop data without an independent reference do not establish precise dynamic
attitude accuracy.

## 1. Stationary stability: two minutes

1. Place the payload in the known launch-pad pose and initialize from that pose.
2. Leave it untouched for two minutes while logging.
3. Confirm acceleration magnitude is near 1 g and gyro readings near zero.
4. Inspect orientation stability, timing gaps, and correction/recovery flags.
5. Report mean orientation, variation about the mean, and start-to-end drift.

Absolute heading error requires an independent heading reference. A stable
heading alone does not prove it is correct.

## 2. Known-angle changes: approximately five minutes

1. With the chosen forward axis horizontal, mark a starting orientation.
2. Rotate +90 degrees, hold ten seconds, then rotate another +90 degrees and hold.
3. Return to the initial pose and hold. Repeat three times in both directions.
4. Impose known roll and pitch changes, for example +/-30 degrees, using a support.
5. Record held-angle error, settling time, and return-to-start error.

Check rotation signs, swapped axes, unit conversion, and excessive lag. Use
marked endpoints as references rather than assuming the hand trajectory is exact.

## 3. Gyro-only propagation: one minute per run

1. Use a test switch to withhold both accel and mag corrections, retaining real
   gyro updates. Keep the payload stationary.
2. Record orientation drift after 10, 30, and 60 seconds.
3. Restore reference inputs and record recovery behavior.
4. Repeat with a controlled 90-degree rotation during the interval.

Retain numeric orientation even when a reference-age policy marks its heading
invalid; log validity separately. This measures baseline drift, not a guaranteed
flight bound under vibration or changing temperature.

## 4. Deterministic input tests

Run the actual integrated estimator on a laptop with a known starting pose,
zero gyro rate unless specified, coherent reference directions, and controlled
timestamps. Reset to the known pose between independent cases.

| Case | Expected observation |
| --- | --- |
| Duplicate timestamp | No extra integration or state reset |
| Near-zero acceleration with small noise | Magnitude check withholds tilt correction |
| Acceleration outside the agreed magnitude band | Tilt correction withheld; gyro propagation continues |
| 1 g vector tilted 30 degrees from the expected reference, briefly applied | Magnitude check passes; angular rejection initially withholds correction when configured below 30 degrees |
| Same inconsistent vector held past the recovery timeout | Recovery eventually permits correction; record orientation error and recovery timing |
| Magnetic direction abruptly rotated beyond its angular threshold at unchanged magnitude | Magnetic rejection initially activates |
| Withheld accel/mag during recovery | Withheld references stay unused |
| Known constant gyro rotation | Integrated angle agrees with the supplied rate and duration |
| Gyro above configured measurement range | Overrange is reported and documented recovery behavior is exercised |

Angular rejection tests assume known-pose startup with normal startup convergence
skipped. Keep brief disturbances shorter than the configured recovery timeout.
Recovery time depends on the sequence of accepted/rejected updates. Simulated
gyro overrange checks software handling, not reconstruction of physically clipped
motion. Repeat at the intended sample rate and exercise representative timing gaps.

## 5. Representative electronics activity

Hold the payload fixed and operate the radio, logging, and servos through
representative commands. Record heading changes, magnetic measurements,
rejection flags, and update timing. Keep the fixture fixed to separate physical
movement from electrical/magnetic effects. If useful, apply and remove a brief
nearby steel-object disturbance, recording its timing; not every disturbance is
guaranteed to trigger angular rejection.

## Review before drone drops

Wrong rotation signs, nonfinite outputs, duplicate integration, unexpected
resets, and failure to propagate with missing references are implementation
failures. Fix those before proceeding.

Record measured drift, pose errors, settling times, recovery effects, and timing
distribution. Agree acceptable values against guidance needs rather than
inventing accuracy limits after seeing results. Preserve logs and the exact
configuration for the planned drone drops from 100 m and subsequent replay.

## Algorithm references

- [Fusion documentation](https://github.com/xioTechnologies/Fusion)
- [Fusion update and recovery implementation](https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionAhrs.c)

The integrated revision is pinned in third_party/Fusion/UPSTREAM.json.
