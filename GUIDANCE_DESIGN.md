# Trajectory planning and guidance discussion

Status: the project owner approved the full phase sequence below. Transition
conditions, numerical thresholds and remaining fallback policies require
discussion before implementation. Dubins geometry is implemented; complete
planning, path management and guidance remain unimplemented.

Record flight-dependent assumptions and required evidence in
[FLIGHT_TEST_VALIDATION.md](FLIGHT_TEST_VALIDATION.md). In particular, the 20%
turn-loss allowance, height prediction, radius/LOS tracking, loiter exit budget,
and pending reserve/tolerance values require data. Registration is not approval
of still-proposed policies.

## Agreed sequence

1. Wait for parafoil deployment and stabilization before starting descent GNC.
   Attitude estimation continues independently throughout flight.
2. Generate a path toward the final-approach entry configuration.
3. Follow the path to its energy-management point without early deliberate loiter.
4. Evaluate available height against the predicted remaining path requirement.
5. If excess height exists, loiter as needed, then reconnect to the approach.
6. If height is suitable, proceed along the remaining path into straight final
   approach. If insufficient, use best-effort direct-to-target guidance.
7. Enter terminal flare when its agreed trigger is asserted; normal heading
   tracking gives way to symmetric brake demand.

The detailed reference is References/Note_on_DCS_to_INSPACe.pdf, sections 7-9.
The sequence is retained; numerical parameter values in documents or tests are
not automatically approved flight values.

## Responsibilities under discussion

- Planner: generate path geometry and predict traversal time/height consumption.
- Path manager: track the active segment, mode transitions, and replan requests.
- Guidance: select the LOS point, derive desired ground track, and convert it to
  desired heading using wind compensation.
- Controller/allocation: track heading and generate normalized brake demands.

These responsibilities do not impose a task scheduler or final FSW app layout.
Dubins geometry uses GNSS ground-track direction. Heading control uses payload
heading as the provisional canopy-heading proxy.

## Agreed wind handling and fallback approach

Apply these decisions when generating a plan, rather than changing approach
direction on every guidance update:

- An available strong wind estimate: final approach points into the wind, and
  measured wind is used for compensation and traversal-time prediction.
- An available weak wind estimate: use the fallback approach bearing while retaining the
  measured vector for compensation and traversal-time prediction.
- No estimate obtained yet: use the fallback bearing and an explicitly identified
  zero-wind planning assumption. This assumption is not a measured calm wind.

The default fallback bearing is from the current position to the target at
plan generation: atan2(-east_m, -north_m). Hold it for that plan. A manually
specified approach-bearing override may be supplied if the landing site requires
one. The bearing at the exact target is undefined and needs a terminal-case rule.

This selects the reserved final straight segment's direction, not an instruction
to discard turn constraints. Connect the current ground-track configuration to
the final-approach entry using the chosen feasible path. If height is insufficient,
the agreed best-effort direct-to-target mode applies. The weak-wind threshold
remains unset. Retain the last accepted estimate without automatic expiry;
age alone does not change the fallback mode or request reassessment. Keep the
update timestamp for analysis. GNSS validity remains a separate concern.

## Agreed path representation

- Straight segment: start and end North/East positions.
- Circular arc: North/East centre, radius, start angle, and signed sweep.
- Normal path: fixed storage for up to four segments, covering a three-segment
  Dubins connection followed by the reserved final straight. No heap allocation.
- Energy-management point: a distance marker along the path, not another segment
  type or an additional split in the segment array.
- Loiter: reuse arc geometry; repetitions are separate manager state. Reconnect
  by generating a replacement remaining path rather than growing the array.

## Agreed segment progress and LOS rules

Use North/East position, not payload heading, to locate progress on the active
ground-path segment. Search the active segment rather than jumping to whichever
segment is globally closest. Completed segments do not reopen within the same
plan; accepting a replacement plan resets path-manager progress, not attitude.

For a straight from A to B, define unit tangent t = (B-A)/length and progress
s = (position-A) dot t. Advance at the end plane, s >= length, with any agreed
numerical tolerance. Position need not coincide with B. Cross-track monitoring
and replanning separately address excessive deviation from the reference path.

For an arc, use theta = atan2(east-centre_east, north-centre_north). Increasing
theta is a right turn in North/East coordinates. Accumulate wrapped successive
angle differences in the intended direction to retain continuity across +/-pi.
Negative increments must not be discarded: otherwise back-and-forth motion can
be counted as a completed orbit. Complete when directed progress reaches the
planned sweep magnitude. Loiter repetition uses the same angular progress rule.
This assumes usable position samples resolve motion between updates; the circle
centre itself has no defined angular position.

Project onto the active segment for the LOS reference, then walk the configured
look-ahead distance along the path. Continue into subsequent segments when the
distance crosses a boundary. Arc travel distance equals radius times swept angle
in radians. Process crossed energy-management markers before normal advancement.

On the final straight, continue its projected reference beyond the target rather
than clamping it to the endpoint, and put the LOS point ahead of that reference.
Passing the end reports target overrun and requests replanning, retaining
final-line tracking while the request is evaluated. The replacement route must
be generated from current position, ground-track direction, remaining height,
wind, and parafoil parameters, not by reversing the previous heading command.
If flare is already active, continue flare. Otherwise accept a return only if
its route and terminal approach fit the remaining height with the applicable
reserve/tolerances. If a return is infeasible, retain controlled final-line
tracking and its separate flare trigger rather than blindly commanding a
low-height direct-to-target reversal. Exact overrun trigger and replan scheduling
details remain to be agreed. No flare is triggered solely by horizontal overrun.

The completion and look-ahead rules are approved. Numerical tolerances,
look-ahead distance, data-quality handling, and replanning policy remain to be
agreed; no code has yet been written for these components.

## Agreed candidate ranking

First exclude candidates with infeasible wind traversal. Rank the remaining
height-feasible candidates lexicographically, without weighted cost terms:

1. Prefer nominal-height candidates; if none exist, consider excess-height
   candidates. Insufficient-height candidates do not form a normal approach.
2. Minimize the sum of absolute Dubins turn angles, not the number of turns.
3. When turning effort is effectively equal, minimize geometric path length.
4. When both are effectively equal, retain the first candidate in a fixed
   evaluation order. Numerical equality tolerances remain to be chosen.

Containing a straight segment does not itself give a candidate priority. A fully
straight candidate has zero turn effort but still obeys the height-class priority.
The turn metric covers the candidate Dubins connection, not future loiter turns.
If no normal approach fits, use the agreed insufficient-height behavior, including
the final-overrun exception requiring a feasible return before a return command.

## Turn-related height-loss margin

The project owner approved a 20% initial allowance for extra height loss during
turns, configurable until further data is collected. Define V_D as nominal straight-glide sink speed and
turn_loss_fraction as the extra fraction applied to arc traversal time:

    required_height = V_D * (straight_time + (1 + turn_loss_fraction) * turn_time)

The same accounting must be used for initial candidate evaluation, replanning,
energy-management checks, full/partial loiter orbits, and reconnection. Evaluate
the actual remaining straight and turning portions. This avoids treating all
remaining flight as straight glide when deciding whether further loiter fits.

Use turn_loss_fraction = 0.20 for the initial implementation. This is a simple
planning allowance, not a detailed aerodynamic turn model or a validated flight
value. It is separate from the terminal height reserve. The model and initial
percentage are agreed; the planner itself remains unimplemented.

## Remaining-height accounting

Initial planning counts the entire Dubins connection and final straight. At the
energy-management point, use the measured current height and count only future
travel; height already lost reaching that point is not charged again.

For a proposed loiter and exit, the agreed budget structure is:

    H_loiter + H_exit_to_approach + H_final + H_reserve <= H_current

The energy-management point may precede the end of the initial Dubins connection.
H_exit_to_approach therefore includes all remaining travel from the proposed
loiter exit to the final-approach entry, including any reconnection. It is not
limited to a short joining manoeuvre. H_final covers the final straight, and
every remaining turn receives the agreed 20% turn-loss allowance. Reserve and
height-classification tolerance settings still require discussion. The project
owner suggested a 10-20 m reserve and 1.5-2% classification tolerance. Exact
values and the percentage basis remain unset; 15 m and 2% of predicted remaining
loss were proposals, not approved defaults. Straight-glide sink speed is also
unknown and remains a required configuration input to determine later.

## Reassessment before path replacement

A trigger requests reassessment rather than unconditional path generation.
Evaluate the remaining current route using updated position, ground course,
height and wind. Include the travel needed to regain the route; do not evaluate
the original route from its original starting point or assume instantaneous
return to it.

Retain the current plan if its remaining route is feasible and acceptable.
Generate replacement candidates only when it fails that assessment, rather
than searching repeatedly for a marginally better path. A wind-direction change
alone does not force replacement: assess forward progress, remaining height
budget, ability to regain the route and final-approach requirements first.
No separate reconnection planner, tracking corridor or correction-height allowance
has been approved for ordinary path deviation. How reassessment accounts for
off-path recovery travel remains unresolved; the simple trigger rules below do
not imply that recovery costs zero height.

## Agreed horizontal and height-shortage triggers

Keep the trigger interface simple, with two configurable distances in metres:

- Horizontal error greater than X metres requests reassessment. Measure the
  shortest North/East distance to the active straight or arc, not to the moving
  LOS point and not to whichever segment is closest across the entire plan.
  Retain the existing final-line continuation beyond the target; target passage
  is handled separately by the agreed overrun event.
- Height shortage greater than Y metres requests reassessment. This is the
  shortfall of current target-relative height against predicted remaining height
  loss plus reserve, rather than geometric distance from a prescribed 3D path.
  In symbols, shortage = predicted_remaining_loss + reserve - current_height.
  Positive shortage exceeding Y triggers the check; excess height alone does
  not trigger it and remains handled at the energy-management point.

X and Y have no selected numerical defaults. Y is a reassessment threshold in
metres, distinct from the percentage tolerance used for height classification.
Reassessment retains the current path if it still works; a threshold crossing
does not itself force replacement or immediately command direct-to-target.

## Agreed reassessment timing

The following configurable initial test settings are approved:

- Horizontal error must exceed X continuously for 1 second before requesting
  reassessment.
- Height shortage must exceed Y continuously for 1 second, using a separate
  persistence timer.
- Reset each persistence timer when its own condition clears. One condition
  cannot inherit elapsed persistence from the other.
- Allow at least 2 seconds between ordinary reassessments.
- Target passage requests reassessment immediately, once per plan, bypassing
  the ordinary interval. Retain the existing flare/overrun behavior.
- Continue tracking the active path during assessment.

These settings require flight/replay validation. X/Y values remain unset.
Wind triggers use the timing below. These are design notes only; no planner,
reassessment logic or timers have been implemented.

## Wind-change threshold

The project owner selected a 30% deviation threshold instead of an absolute
wind-change threshold in m/s. For a nonzero reference vector, the discussed
comparison is:

    relative_change = magnitude(latest_wind - reference_wind) / magnitude(reference_wind)

Request reassessment when relative_change > 0.30. This compares vectors, so
direction changes as well as speed changes contribute. It requests assessment
of current-path feasibility rather than unconditional replacement.

The approved reference is the wind used at the last completed reassessment
(initially the plan-generation wind), refreshed even if the path is retained.
Use a separate 1-second continuous wind persistence timer, reset when the
condition clears, and the ordinary 2-second reassessment interval.
Percentage change is undefined for zero reference wind and sensitive for weak
reference wind. The approved rule uses the existing weak-wind classification:
skip percentage checks while both reference and latest usable estimates are
weak; request reassessment when crossing between weak and strong wind. Apply
the 30% vector comparison when both are strong. The weak-wind threshold remains
unset. Do not invent a denominator floor or treat assumed zero wind as measured
wind. Retain the last accepted estimate without expiry. Rejected measurements
can occur with usable GNSS, so wind rejection is not equivalent to lost GNSS.
Age alone does not request reassessment. Resumed accepted updates use the same
percentage/classification rules. The first accepted wind estimate requests
reassessment because it replaces the initial zero-wind assumption with measured
information. No wind-trigger code exists.

## Common simulation configuration

The owner approved arbitrary dummy values for later simulation tests rather
than selecting physical flight parameters now. dcs_config.h is the common
configuration header: DCS_SIM_DUMMY_* names and comments explicitly identify
placeholder radius, airspeed, sink rate, final length, energy-point offset, LOS
look-ahead, X/Y thresholds and weak-wind threshold. Reserve/tolerance placeholders
are also labelled dummy; the percentage basis still needs agreement before use.
Agreed numerical test settings use DCS_TEST_* names separately. These constants
are not wired into existing estimators and do not implement a planner. Keep
planning and wind-estimator airspeed consistent when simulation is integrated.
Dummy values are not evidence of physical feasibility, including whether a
100 m drop can accommodate the configured route and any loiter.

## Planner interface

dcs_planner.h declares the agreed planner boundary; dcs_planner_generate has no
implementation yet. DcsPlannerInput supplies current target-centred position,
GNSS course, height and wind/availability. The planner selects the final course
internally using the agreed override/strong-wind/fallback policy.
DcsPlannerConfig supplies geometric and height-model parameters, the weak-wind
threshold and optional manual course override. Dummy macros
in dcs_config.h are not automatically applied to this structure.

DcsPlannerOutput holds at most four tagged straight/arc segments, a path-distance
energy marker and predicted time/height loss. Height loss includes the turn
allowance but excludes the separately budgeted reserve. DcsPlannerResult
distinguishes nominal approach, excess height, no feasible approach and invalid
input. Insufficient height is a failure reason within no feasible approach,
alongside wind infeasibility; it is not a separate feasibility category.
Only nominal/excess results supply a normal route;
the manager decides fallback/overrun behavior and retains its active plan until
a replacement is accepted. Call into separate candidate storage.

The percentage classification basis, numerical ranking tolerances, finite
path-integration resolution and the short-Dubins case where the requested
energy point lies before the path start still need agreement before the
corresponding calculations are implemented. The interface accepts an absolute
classification tolerance without silently selecting a percentage basis.

## Implemented Dubins geometry

dcs_dubins.h/.c constructs one requested family from start/goal position and
ground course with a positive turn radius: LSL, RSR, LSR, RSL, RLR or LRL. This
is also the fixed evaluation order for future tie-breaking. Each successful
call returns three segment slots (zero-length segments can occur), geometric
length and total absolute turning. Failure leaves output unchanged. The caller
will evaluate all six; this layer does not select a candidate.

Normalized equations are adapted from the primary
[Andrew Walker Dubins-Curves implementation](https://github.com/AndrewWalker/Dubins-Curves/blob/master/src/dubins.c).
Its MIT copyright and permission notice are preserved in dcs_dubins.c. The code
reflects East for the standard mathematical equations, then emits native
North/East geometry with positive right-turn sweeps. Float roundoff guards near
touching/coincident circles are numerical handling, not physical tolerances.
The implementation uses float math, fixed storage and a three-segment loop;
there is no sampling loop dependent on route length and no dynamic allocation.

Host checks cover straight and quarter-circle known lengths, coincident poses,
touching/overlapping circles, impossible three-arc candidates, all-family pose
closure, stored-geometry continuity, left/right mirror symmetry and invalid
input/output preservation. Endpoint checking independently integrates the
motion in double precision. These tests establish software geometry only,
not a parafoil's achievable ground-track curvature or STM32 execution time.

No final line, wind/height integration, candidate ranking, energy marker,
reassessment, path manager or heading commands are implemented by this layer.

## Details to discuss next

- Replace simulation-only weak-wind and other physical placeholders using data.
  Preserve the chosen final bearing while following the current plan.
- X/Y values and off-path recovery-cost assessment. Expensive candidate
  generation should not run on
  every guidance/controller update.
- Loiter exit: budget the exit/reconnection path and final approach before
  committing to further height loss. Partial-orbit reconnection can change the
  remaining height requirement.
- Height reserve/tolerance and loiter exit budget, sink-rate input, deployment/stabilization readiness,
  progress tolerances and look-ahead distance, height reserve, and terminal flare
  authority/trigger.

The phase sequence, wind/fallback behavior, path representation, segment/LOS
rules, overrun replanning, and candidate ranking above are approved. The remaining
discussion items are not implemented policies.
