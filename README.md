# DCS firmware core

The first portable C modules implement target-centred horizontal navigation,
barometric vertical position, and the CDR's straight-flight, GNSS-based wind
measurement with an exponential moving average. A compact payload attitude
estimator fuses gyro, gated accelerometer, and calibrated magnetometer data.
These modules have no MCU or driver
dependencies and use caller-owned, fixed-size state.

This is a development checkpoint, not complete flight firmware. The attitude
implementation is still the provisional custom filter; Fusion is selected but
has not been integrated. Desktop checks establish basic software behavior,
not flight accuracy. Bench and drone-drop testing have not been performed here.

Architecture, algorithms, interfaces, thresholds, and recovery behavior must
be discussed with the project owner before implementation. Routine drivers
and low-level implementation can proceed within the agreed design.

`dcs_nav_step` corrects North/East position and velocity from GNSS, holds fresh
GNSS ground velocity between fixes, and takes target-relative height from the
barometer. Its horizontal propagation does not require a canopy-heading
estimate.

`dcs_wind_update` accepts processed GNSS ground velocity, a recent payload
heading estimate, and the current normalized brake command. The thresholds and
EMA coefficient are configuration values, not flight-calibrated defaults.
Its estimate is **provisional** because payload heading is not a direct canopy
heading measurement. Do not enable wind-based guidance solely on the basis of
these desktop tests.

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
Fusion has been selected to replace the current custom filter. Application
integration is proposed in [ATTITUDE_FUSION_DESIGN.md](ATTITUDE_FUSION_DESIGN.md);
rejection thresholds and recovery policy remain to be agreed before replacement.
The proposed physical and replay checks are documented in
[ATTITUDE_BENCH_TESTS.md](ATTITUDE_BENCH_TESTS.md).

The current custom filter uses proportional vector feedback with separate accel
and mag gains. Accel correction requires a configured magnitude band around 1 g;
mag correction requires a configured total-field band and a horizontal component
at least 10% of the expected field. Both require caller permission and valid,
finite inputs. Heading is held and marked invalid when the squared horizontal
projection of the unit forward axis is below 0.04; quaternion updates continue.
These checks and numerical cutoffs are provisional earlier implementation choices,
not agreed flight criteria or Fusion settings.

Build and run the desktop check with a C11 compiler:

```sh
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. dcs_nav.c tests/test_dcs_nav.c -lm -o dcs_nav_test
./dcs_nav_test
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. dcs_attitude.c tests/test_dcs_attitude.c -lm -o dcs_attitude_test
./dcs_attitude_test
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. dcs_wind.c tests/test_dcs_wind.c -lm -o dcs_wind_test
./dcs_wind_test
```

The flight-software integration supplies monotonic millisecond timestamps,
fresh valid GNSS solutions, calibrated attitude sensor vectors, and brake commands.
The next step is agreeing on Fusion application behavior, integrating it, and
running the bench checks. Navigation propagation during turns remains under
review. Guidance, control, brake allocation, and servo integration remain work
to do; none of these modules drives servos.

The main FSW is evolving. Its integration model is one-time initialization,
bounded updates called from a cooperative super-loop, persistent module state,
and timestamped inputs. Existing FSW app names, shared fields, scheduling, units,
and ADCS/DCS interfaces are preliminary examples rather than fixed requirements.
