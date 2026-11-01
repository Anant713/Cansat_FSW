# Circle-based wind and airspeed identification

Status: the bounded velocity-circle estimator and runtime identification
sequence are implemented in dcs_circle_fit.h/.c and dcs_runtime.h/.c. Physical
commands, gates and timing remain configurable and require validation.

## Primary references

- Jeffrey C. Gilkey, Sandia National Laboratories, *Precision Guided Parachute
  LDRD Final Report*, SAND96-1622, 1996, printed pp. 12-13 (PDF pp. 20-21),
  [report](https://digital.library.unt.edu/ark:/67531/metadc671463/m2/1/high_res_d/270670.pdf).
  Describes in-flight least-squares circle fitting of North/East GPS velocities
  to estimate wind components and airspeed during steady turning.
- W. M. Hollister, E. R. Bradford and J. D. Welch, *Using Aircraft Radar Tracks
  to Estimate Winds Aloft*, Lincoln Laboratory Journal 2(3), 1989, theory and
  error discussion,
  [paper](https://www.ll.mit.edu/sites/default/files/publication/doc/using-aircraft-radar-tracks-estimate-hollister-ja-6375.pdf).
  Explains the velocity-circle geometry without known heading/airspeed, and
  errors when airspeed varies during a turn. Its radar-specific weighting and
  iterative estimator are not proposed as our implementation.
- Edward J. Scheuermann, Georgia Institute of Technology, *Autonomous Control
  of Parafoil and Payload Systems Using Upper Surface Canopy Spoilers*, 2015,
  section 4.1.1,
  [thesis](https://repository.gatech.edu/bitstreams/a7f5ab6d-4220-4f63-b57e-4be5482c00cd/download).
  Describes an initialization turn with constant asymmetric actuation and
  regression of GPS velocity to estimate wind and vehicle airspeed. Spoiler
  hardware and maneuver parameters are not transferred to this CanSat.

## What is fitted

Let g = (vN, vE) be measured GNSS horizontal ground velocity, W horizontal wind,
and Vh the approximately constant horizontal airspeed over the fitting window:

    g = W + air_velocity
    (vN - WN)^2 + (vE - WE)^2 = Vh^2

Plot vN against vE, not North/East positions. The points lie approximately on a
circle whose centre is (WN, WE) and whose radius is Vh in m/s. This avoids
needing a known airspeed or payload-to-canopy heading relationship to obtain
the initial wind estimate. It does not make position propagation independent
of wind or make the resulting wind true without validation.

For a small implementation, expand that equation:

    vN^2 + vE^2 = 2*WN*vN + 2*WE*vE + C
    C = Vh^2 - WN^2 - WE^2

Fit the three unknowns (WN, WE, C) using least squares, then recover
Vh = sqrt(C + WN^2 + WE^2). This algebraic three-unknown fit is our proposed
implementation of the circle geometry, not a claim that it duplicates the
references' solvers. A fixed-size sample window and a bounded small solve are
compatible with the MCU/super-loop design. Centering/scaling and fit-quality
handling must be specified before coding; normal-equation conditioning and
noise bias must be tested rather than assuming any three samples suffice.

The method needs useful spread of velocity directions and approximately
constant wind and airspeed over that window. A well-covered turn helps; heading
rotation alone is not proof that the GNSS velocity samples constrain a circle.
GNSS receiver update rate, latency, velocity noise and actual turn duration
matter, especially in small fast circles. Duplicate or propagated velocities
are not independent measurements and must not populate the fitting window.

An identification turn is different from a ground-fixed loiter orbit. Its
ground trajectory can drift with wind. Do not estimate wind from ground-circle
centre or use displacement per turn without the additional assumptions that
air-relative motion closes. Circle fitting in velocity space does not require
a uniform turn rate, but constant differential brake does not itself guarantee
constant airspeed, particularly after damage or during entry transients.

## Turning airspeed versus trim speed

The fitted radius is horizontal airspeed during the sampled maneuver. It is
not automatically the undamaged, steady, brakes-released trim speed. Persistent
asymmetric brake, bank/sink changes and a damaged rig invalidate that automatic
identification. A low fit residual does not prove those states are equivalent.

Proposed two-stage identification:

1. After deployment has settled and sufficient height/time is available, use
   a controlled identification turn to estimate W and turning Vh.
2. Release steering/braking to the intended normal trim setting, let the
   transition settle, and estimate horizontal trim speed from several fresh
   GNSS measurements: Vtrim_h = magnitude(g - W). Wind must remain sufficiently
   representative over this short subsequent interval.

The second stage estimates magnitude without payload heading. Barometric
height history can separately estimate ground-relative sink in settled straight
flight. Pressure alone cannot separate horizontal wind and airspeed. Our planner
uses horizontal airspeed; do not silently substitute total 3D speed or treat
barometric descent as vertical air-relative speed in unknown vertical wind.

This initial identification can bootstrap the existing straight-flight wind
estimator, which currently requires configured horizontal airspeed. Whether it
initializes or replaces that estimator, how accepted trim updates are shared
with planning, and whether later turns refresh the fit require approval.
Do not run conflicting estimates silently or promote simulation dummy values
to measured performance if identification fails.

## Reported damaged flight

On 2026-11-01 the owner reported a test in which the servo mount broke, pulling
one brake line significantly and causing continuous small circles during descent.
At the initial method review, the flight date, sample rate, height, failure
time and raw dataset had not been established. The owner subsequently supplied
the raw Test2_3010 logs and magnetic calibration from a different PCB.
[Offline analysis](Flight_Data/Analysis_Test2_3010/REPORT.md) now includes
algebraic/geometric velocity-circle fits, window sensitivity and provisional
wind. Early/late fitted maneuver speeds differ (about 3.14 vs 1.57 m/s), so no
healthy trim value is accepted. The fitter is offline analysis tooling, not an
onboard estimator or an implemented identification maneuver.

The flight may contain useful windows for an offline wind/turning-speed fit if
the velocity data have sufficient spread and a stable fit across windows.
Constant-speed consistency fails over the full descent, and independent wind
accuracy is not established. Tag the data as
mechanically damaged. Do not identify healthy trim, healthy sink/turn radius,
or commanded-brake response from this flight alone; actual brake displacement
was no longer determined by the commanded servo value. Subsequent steady
brakes-released data cannot be assumed to exist in that flight.

## Implemented policy and remaining flight decisions

- Main FSW explicitly starts DCS in secondary descent. After a configured canopy
  settling time, DCS commands one configured single-sided identification input.
- Invalid/insufficient barometric height or configured timeout limits abort
  identification. Failure uses the
  configured fallback airspeed/wind and raises a status flag.
- The fixed-size fit requires unique fresh GNSS epochs, sample count, velocity
  spread, angular coverage, radius bounds and radial-RMS acceptance.
- After neutral settling, trim speed is averaged from |Vground-W| over fresh
  GNSS samples. The accepted result seeds the existing straight-flight wind EMA.
  Numerical flight settings remain to be selected.
- GNSS-outage propagation and its interaction with the new estimates. Continue
  from a propagated state as requested, but do not feed propagated data back
  into this measurement-based identification as if they were fresh GNSS fixes.

Record required data and eventual evidence in
[FLIGHT_TEST_VALIDATION.md](FLIGHT_TEST_VALIDATION.md). Implementation and host
tests do not make the manoeuvre flight-approved.
