# DCS firmware core

The first portable C modules implement target-centred horizontal navigation,
barometric vertical position, and the CDR's straight-flight, GNSS-based wind
measurement with an exponential moving average. A compact payload attitude
estimator uses x-io Fusion to combine gyro, gated accelerometer, and calibrated
magnetometer data.
These modules have no MCU or driver
dependencies and use caller-owned, fixed-size state.

This is a development checkpoint, not complete flight firmware. Fusion is
integrated into the portable attitude module. Desktop checks establish software
behavior, not flight accuracy. Hardware bench and drone-drop testing have not
been performed here.

Architecture, algorithms, interfaces, thresholds, and recovery behavior must
be discussed with the project owner before implementation. Routine drivers
and low-level implementation can proceed within the agreed design.

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
can be valid. Expiry and guidance fallback behavior remain to be agreed.

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

On Windows, build and run all desktop checks with GCC:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
python tools/vendor_fusion.py --verify
```

Executables are written to the ignored .build/host directory. Equivalent C11
compiler commands (use platform-appropriate executable suffixes):

```sh
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. dcs_nav.c tests/test_dcs_nav.c -lm -o dcs_nav_test
./dcs_nav_test
gcc -std=c11 -Wall -Wextra -Werror -pedantic -DFUSION_USE_NORMAL_SQRT -I. dcs_attitude.c third_party/Fusion/Fusion/FusionAhrs.c tests/test_dcs_attitude.c -lm -o dcs_attitude_test
./dcs_attitude_test
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. dcs_wind.c tests/test_dcs_wind.c -lm -o dcs_wind_test
./dcs_wind_test
```

The flight-software integration supplies monotonic millisecond timestamps,
fresh valid GNSS solutions, calibrated attitude sensor vectors, and brake commands.
For FSW integration, compile dcs_attitude.c and the pinned FusionAhrs.c with
`FUSION_USE_NORMAL_SQRT` consistently defined, and retain the Fusion headers and
MIT license under third_party/Fusion. This uses upstream's standard square-root
option without changing upstream sources. No additional loop or scheduler is
required: keep one DcsAttitudeState and update it for each new gyro sample.

Hardware bench testing is deferred for now. The wind model is retained; discuss
wind freshness and its use by guidance next. Guidance, control, brake
allocation, and servo integration remain work
to do; none of these modules drives servos.

The main FSW is evolving. Its integration model is one-time initialization,
bounded updates called from a cooperative super-loop, persistent module state,
and timestamped inputs. Existing FSW app names, shared fields, scheduling, units,
and ADCS/DCS interfaces are preliminary examples rather than fixed requirements.
