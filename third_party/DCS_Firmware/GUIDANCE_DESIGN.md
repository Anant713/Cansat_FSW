# Trajectory planning and guidance discussion

Status: the approved sequence is implemented in portable C. Numerical flight
parameters, actuator calibration and physical performance still require bench,
simulation and flight evidence.

Record flight-dependent assumptions and required evidence in
[FLIGHT_TEST_VALIDATION.md](FLIGHT_TEST_VALIDATION.md). In particular, the 20%
turn-loss allowance, height prediction, radius/LOS tracking, loiter exit budget,
and pending reserve/tolerance values require data. Registration is not approval
of still-proposed policies.

## Agreed sequence

1. Wait for parafoil deployment and stabilization before starting descent GNC.
   Attitude estimation continues independently throughout flight.
2. Generate a Dubins connection to the energy-management configuration, then a
   straight through the final-approach entry to the target.
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

## Agreed simple runtime structure

- Planner: generate path geometry and predict traversal time/height consumption.
- One guidance state/update: track the active segment and markers, handle height
  decisions and mode transitions, reassess/replan when needed, select the LOS
  point and convert desired ground track to heading with wind compensation.
- Controller/allocation: track heading and generate normalized brake demands.

These responsibilities do not impose a task scheduler or final FSW app layout.
Use one persistent guidance state and one bounded update called by the existing
super-loop. Planner and geometry functions remain calculation helpers. A path
manager may describe an internal responsibility, but it is not a separate
event-producing subsystem requiring caller acknowledgments.
Dubins geometry uses GNSS ground-track direction. Heading control uses payload
heading as the provisional canopy-heading proxy.

## Runtime ownership and latest integration decisions

The main FSW state machine explicitly calls dcs_runtime_start when secondary
descent begins. DCS does not infer HDRM deployment or duplicate the mission
state machine. Once started, DCS owns settling, identification, guidance,
terminal flare and degraded-navigation actuator behavior. Drivers and sensor
calibration remain external.

The controller needs measured heading and heading rate. The existing attitude
state already supplies payload_heading_rad and payload_heading_rate_rad_s.
The controller receives guidance's desired heading plus
attitude feedback directly; no need to duplicate measured feedback in guidance
output. Measured rate is not the same quantity as an optional desired-rate
feedforward. The implemented V1 law is configurable heading-error proportional
feedback plus measured-heading-rate damping; no unmeasured Kr-based feedforward
is introduced.

During GNSS loss, continue using propagated position as requested; do not stop
solely because the GNSS measurement is unavailable. Separate measured-data
freshness from availability of the propagated state. dcs_nav_step now continues
the held-velocity state after measurement freshness expires and reports both
conditions separately. The current baseline holds GNSS velocity
and cannot reproduce a turn through a long outage. A pressure measurement
corrects target-relative height, not horizontal position or wind.

The following runtime choices were accepted in the consolidated discussion:

- Persistent horizontal deviation > X makes the active route unacceptable;
  try replacement. If unavailable GNSS prevents usable planning, continue on
  the propagated state rather than discard the route.
- Estimate remaining-route height with the existing model and reserve; correct
  available height with barometer measurements. Explicit recovery travel is
  not modeled in this simplified runtime and needs validation against reserve.
- With strong wind, a tailwind component on the existing final approach makes
  it unacceptable. Otherwise retain it when traversal and height remain feasible.
- Before the energy decision, replacements connect to qE. After that decision,
  including loiter exit/overrun return, connect to qA without repeating energy
  management. Both qE and qA planner routes are implemented.
- Use a tangent loiter with configured side/radius and allow partial exits.
  Evaluate loiter, exit connection, final straight and reserve together.
- Retry no-approach planning at the ordinary interval with usable inputs. Retain
  the previously agreed direct-target/final-overrun fallback distinction.
- If a requested LOS course cannot be wind-compensated, request reassessment,
  hold the last valid heading demand while waiting and flag unachievability;
  report unavailable if no prior demand exists.

Low GNSS ground speed was considered unlikely; no additional low-speed holding
policy was approved in that exchange. Unknown heading must not be invented.

The owner requested circle-based wind/airspeed identification because healthy
test data are insufficient. See
[CIRCLE_WIND_ESTIMATION_DESIGN.md](CIRCLE_WIND_ESTIMATION_DESIGN.md) for primary
sources, velocity-space circle fitting, the turning-versus-trim distinction,
the reported mount failure and proposed integration. This does not yet change
the existing straight-flight wind estimator or automatically command a circle.

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
one. At the exact target, use the exception documented below: nonzero wind
first, otherwise the usable current GNSS ground course.

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
  Dubins connection to the energy point followed by one straight through the
  final-approach entry and target. No heap allocation. The final-approach entry
  is a marker within that straight, not an additional segment.
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
   evaluation order. Use small numerical tolerances to decide whether turning
   or length are effectively equal; their values remain to be chosen. These
   handle rounding, not physical tracking or height errors.

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
percentage are agreed and used by candidate planning; flight validation is pending.

## Remaining-height accounting

Initial planning counts the entire Dubins connection, energy-to-entry straight
and final straight. At the
energy-management point, use the measured current height and count only future
travel; height already lost reaching that point is not charged again.

For a proposed loiter and exit, the agreed budget structure is:

    H_loiter + H_exit_to_approach + H_final + H_reserve <= H_current

The energy-management point is now the endpoint of the initial Dubins connection.
H_exit_to_approach includes all remaining travel from the proposed
loiter exit to the final-approach entry, including any reconnection. It is not
limited to a short joining manoeuvre. H_final covers the final straight, and
every remaining turn receives the agreed 20% turn-loss allowance. The project
owner suggested a 10-20 m reserve and 1.5-2% classification tolerance. The approved
percentage basis is predicted remaining height loss, excluding reserve:

    height_tolerance_m = tolerance_fraction * predicted_remaining_loss_m

Exact flight values remain unset. The 15 m reserve and 2% fraction in the common
configuration are simulation-only placeholders. Straight-glide sink speed is also
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
For wind/height requests, reassessment retains the current path if it still
works; a request does not immediately command direct-to-target. In the latest
runtime decision, sustained horizontal error beyond X itself makes the path
unacceptable and requests replacement, while propagated tracking continues if
usable planning inputs are unavailable.

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
Wind triggers use the timing below. These are runtime design notes only;
candidate planning is implemented, but reassessment logic and timers are not.

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

## Frozen flight configuration

The formerly arbitrary simulation values are now frozen under DCS_FLIGHT_* by
programme decision because no further identification time is available. They
remain unvalidated physical assumptions. Apply the height-tolerance fraction to
predicted remaining loss excluding reserve in candidate height evaluation.
`dcs_flight_config_load` applies the consistent planner, estimator, controller,
actuator, attitude and navigation values. Flight acceptance is not evidence that
the approximately 600 m parafoil phase can safely execute the configured route,
identification and loiter.

## Planner interface

dcs_planner.h declares the planner boundary, implemented by dcs_planner.c.
DcsPlannerInput supplies current target-centred position,
GNSS course, height and wind/availability. The planner selects the final course
internally using the agreed override/strong-wind/fallback policy.
DcsPlannerConfig supplies geometric and height-model parameters, the weak-wind
threshold and optional manual course override. Dummy macros
in dcs_config.h are not automatically applied to this structure.

DcsPlannerOutput holds four tagged straight/arc segment slots, path-distance
energy and final-entry markers, and predicted time/height loss. Height loss includes the turn
allowance but excludes the separately budgeted reserve. DcsPlannerResult
distinguishes nominal approach, excess height, no feasible approach and invalid
input. Insufficient height is a failure reason within no feasible approach,
alongside wind infeasibility; it is not a separate feasibility category.
Only nominal/excess results supply a normal route;
the manager decides fallback/overrun behavior and retains its active plan until
a replacement is accepted. Call into separate candidate storage.

The header accepts height_tolerance_fraction; the evaluator computes an absolute
band for each candidate using the agreed basis. The common configuration labels
32 Simpson intervals per arc (33 nodes), 1e-5 rad turn equality and 0.001 m length
equality as simulation-only numerical settings. They require convergence and
integration evidence before flight use; no physical defaults are applied by
the planner. INVALID_INPUT preserves candidate output. NO_FEASIBLE_APPROACH
clears candidate output and reports insufficient height if at least one
wind-traversable candidate exists, otherwise wind infeasibility. Callers must
keep candidate storage separate from the active plan.

## Near-target starts and energy-point placement

Horizontal distance less than the configured final-approach length is not a
separate mode trigger and does not automatically shorten final approach. For
example, starting secondary descent at 600 m AGL and 10 m horizontally from the
target still requires height management. Retain the configured final length,
generate a route from actual position/course, reach the energy-management point,
and use the remaining-height decision to loiter or continue. Initial travel away
from the target can be appropriate; proximity alone does not justify landing.

The approved construction replaces the earlier marker-on-Dubins design: place
qA = -final_length * t, where t is the chosen final-course unit vector, and place
qE = qA - energy_point_upstream * t. Generate the Dubins connection from actual
position/course to qE with arrival course t. From qE, continue straight through
qA to the target. The energy marker is at the end of that connection. Budget
the whole route, including qE-to-qA, and retain the agreed candidate ranking.
No marker is clamped to the start, no minimum Dubins length is required to fit
an upstream marker, and no extra arbitrary loop is appended. For an into-wind
approach, qE lies downwind while the arrival course faces into wind.

The two collinear straight legs share one segment, retaining four-slot storage.
The planner now emits qE and qA distance markers. The future manager must process
them in order; guidance now handles marker crossing and loiter execution.

## Exact-target bearing edge case

Initial planning at exactly the target's horizontal position has no defined
position-to-target bearing. The approved direction-selection order is:

1. Use a manual override when enabled.
2. Otherwise use an into-wind direction for strong wind.
3. Otherwise use position-to-target bearing when displacement is nonzero.
4. At zero displacement, use into-wind direction if any nonzero wind estimate
   exists, even weak wind. This is an exception to the usual weak-wind rule.
5. With zero displacement and no estimate or exactly zero wind, use the current
   usable GNSS ground course as the final-approach direction.

The last fallback selects the orientation of the approach; it does not imply a
straight flight from the current pose to qE. Generate the Dubins connection as
usual. Starting course also comes from GNSS, so usable GNSS ground course is
already a prerequisite. This is distinct from target overrun on an existing
approach, where final-line guidance continues during reassessment and flare
remains separately triggered. No compass bearing is invented.

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

## Implemented candidate evaluation

dcs_planner_generate evaluates families in the fixed order above. Each route
contains the Dubins connection to qE followed by the combined qE-to-target
straight. Zero-length geometry slots remain present and contribute no time.
For each traversed course chi:

    W_parallel = W dot [cos(chi), sin(chi)]
    W_perpendicular = W dot [-sin(chi), cos(chi)]
    ground_speed = W_parallel + sqrt(airspeed^2 - W_perpendicular^2)

Reject a direction if crosswind exceeds airspeed or forward ground speed is
nonpositive. Straight time is length/ground_speed. Arc time uses fixed-count
Simpson integration of reciprocal ground speed. Upwind arithmetic uses a
conjugate form to avoid subtraction cancellation near wind equal to airspeed.
When wind magnitude is at least airspeed, feasible courses form a downwind
cone no wider than pi: arcs longer than pi are rejected, and endpoint/sample
checks on shorter arcs prevent accepting an unsampled infeasible excursion.
Wind feasibility is not inferred from time samples alone.

Compute straight and turning time separately and apply the agreed 20% modifier
only to turning time. With predicted loss H, reserve R, fraction f and current
height z, classify error z-(H+R): nominal within +/-f*H, excess above that band,
and insufficient below it. Thus the tolerance can admit a small reserve
shortfall; reserve is not a separate absolute rejection after classification.
Rank nominal ahead of excess, then turning, length and fixed family order.
All routes are evaluated from current position/course; this function does not
reassess the active path or implement a fallback maneuver.

Host tests cover analytical calm/tailwind/headwind/crosswind straight times,
reserve/tolerance boundaries, nominal priority over lower-turn excess routes,
equal-turn shortest length and exact fixed-order ties, wind infeasibility,
near-target high starts, exact-target bearing cases and coincident markers.
Selected arc routes through wind/airspeed ratios up to 0.999 agree within 0.2%
with a separate double-precision 8192-midpoint-per-arc reference. This is limited
numerical evidence, not a global error guarantee or validation of the physical
model. Invalid-input preservation and no-route failure reasons are also checked.

## Agreed direct marker handling

The owner approved simplifying runtime coordination to one guidance state/update.
This supersedes the earlier pending-event/acknowledgment and old-plan handoff
proposals. There is no separate event queue, acknowledgment API or retained
history of obsolete plan notifications.

Within an update, locate progress from usable position and directly handle:

- Energy point reached: evaluate remaining height and choose continue or loiter
  before processing later approach transitions.
- Final entry crossed: enter the final-approach portion.
- Target finish plane crossed: assess a replacement, preserving the agreed
  final-line guidance while assessment is pending or no feasible return exists.
  Flare remains separately triggered.

Record handled transitions in persistent mode/progress state so repeated calls
do not forget or repeatedly process a detected crossing. Process coincident
energy/final-entry markers in order, with the energy decision first. No external
consumer must observe a one-update notification pulse for control to proceed.
Logging may report transitions, but delivery/acknowledgment does not gate control.

Accept a valid replacement immediately and reset progress/mode for that route;
old route markers cease to apply. They are not queued to affect the new route.
A failed replacement calculation preserves the active plan. Attitude remains
independent. Preserve usable progress/state on invalid position input rather
than inferring marker crossings from it.

"Cannot be missed" now means every detected crossing is handled by guidance,
not that obsolete notifications are preserved indefinitely. Existing position
quality and arc-progress sampling assumptions still limit crossing detection;
this is not power-loss retention or a guarantee across arbitrary unsampled
motion. Runtime state needs the active path, segment/progress, current mode,
minimal handled-transition bookkeeping and the agreed reassessment timers.
Loiter exit geometry and off-path reassessment details still need agreement.
This runtime update has not been implemented yet.

## Required pre-launch calibration and validation

- Replace simulation-only weak-wind and other physical placeholders using data.
  Preserve the chosen final bearing while following the current plan.
- X/Y values and off-path recovery-cost assessment. Expensive candidate
  generation should not run on
  every guidance/controller update.
- Verify full/partial loiter exit budgeting and qA reconnection in simulation
  across the expected 600 m secondary-descent envelope.
- Validate numerical integration and ranking settings across simulation cases;
  implement manager processing of the energy and final-entry markers.
- Set physical height reserve/tolerance, sink rate, turn radius, settling and
  identification settings, progress tolerances, look-ahead, controller gains,
  servo endpoints and terminal flare authority/trigger.

The phase sequence, wind/fallback behavior, path representation, segment/LOS
rules, overrun replanning, and candidate ranking above are approved. The remaining
host-tested behavior is not flight validation.
