# Trajectory planning and guidance discussion

Status: the project owner approved the full phase sequence below. Transition
conditions, numerical thresholds, exact APIs, and remaining fallback policies
require discussion before implementation. No planning or guidance code exists.

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

- Fresh, sufficiently strong wind: final approach points into the wind, and
  measured wind is used for compensation and traversal-time prediction.
- Fresh but weak wind: use the fallback approach bearing while retaining the
  measured vector for compensation and traversal-time prediction.
- Missing or stale wind: use the fallback bearing and an explicitly identified
  zero-wind planning assumption. This assumption is not a measured calm wind.

The default fallback bearing is from the current position to the target at
plan generation: atan2(-east_m, -north_m). Hold it for that plan. A manually
specified approach-bearing override may be supplied if the landing site requires
one. The bearing at the exact target is undefined and needs a terminal-case rule.

This selects the reserved final straight segment's direction, not an instruction
to discard turn constraints. Connect the current ground-track configuration to
the final-approach entry using the chosen feasible path. If height is insufficient,
the agreed best-effort direct-to-target mode applies. Wind-age and weak-wind
thresholds remain configurable values to agree; the estimator itself does not
expire its output.

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
height-classification tolerance settings still require discussion.

## Details to discuss next

- Wind-age and weak-wind thresholds, and when wind/approach changes warrant a
  replan. Preserve the chosen final bearing while following the current plan.
- Replanning triggers, request persistence, and minimum interval. Expensive
  candidate generation should not run on every guidance/controller update.
- Loiter exit: budget the exit/reconnection path and final approach before
  committing to further height loss. Partial-orbit reconnection can change the
  remaining height requirement.
- Height reserve/tolerance and loiter exit budget, sink-rate input, deployment/stabilization readiness,
  progress tolerances and look-ahead distance, height reserve, and terminal flare
  authority/trigger.

The phase sequence, wind/fallback behavior, path representation, segment/LOS
rules, overrun replanning, and candidate ranking above are approved. The remaining
discussion items are not implemented policies.
