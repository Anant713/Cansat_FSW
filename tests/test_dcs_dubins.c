#include "../dcs_dubins.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846

static double angle_error(double actual, double expected)
{
    return fabs(remainder(actual - expected, 2.0 * PI));
}

/* Independent double-precision kinematic integration, plus stored arc checks. */
static void check_path(const DcsDubinsPose *start, const DcsDubinsPose *goal,
                       float radius, DcsDubinsFamily family,
                       const DcsDubinsPath *path)
{
    static const int signs[6][3] = {
        {-1, 0, -1}, {1, 0, 1}, {-1, 0, 1},
        {1, 0, -1}, {1, -1, 1}, {-1, 1, -1}
    };
    double n = start->position.north_m;
    double e = start->position.east_m;
    double heading = start->course_rad;
    double total_length = 0.0, turns = 0.0;
    const double tolerance = 0.002 + 0.00002 * radius;
    for (int i = 0; i < 3; ++i) {
        const DcsPathSegment *segment = &path->segments[i];
        const int sign = signs[family][i];
        if (sign == 0) {
            assert(segment->type == DCS_PATH_STRAIGHT);
            assert(hypot(segment->geometry.straight.start.north_m - n,
                         segment->geometry.straight.start.east_m - e) < tolerance);
            const double dn = segment->geometry.straight.end.north_m - n;
            const double de = segment->geometry.straight.end.east_m - e;
            const double length = hypot(dn, de);
            /* Direction matters: the straight must not run backwards. */
            if (length > tolerance) {
                assert(angle_error(atan2(de, dn), heading) < 0.002);
            }
            n += length * cos(heading);
            e += length * sin(heading);
            total_length += length;
        } else {
            assert(segment->type == DCS_PATH_ARC);
            const double sweep = segment->geometry.arc.sweep_rad;
            const double angle = segment->geometry.arc.start_angle_rad;
            assert(sign * sweep >= 0.0 && fabs(sweep) <= 2.0 * PI + 0.00001);
            assert(segment->geometry.arc.radius_m == radius);
            assert(hypot(segment->geometry.arc.centre.north_m + radius * cos(angle) - n,
                         segment->geometry.arc.centre.east_m + radius * sin(angle) - e)
                   < tolerance);
            assert(angle_error(angle + sign * PI / 2.0, heading) < 0.00001);
            n += sign * radius * (sin(heading + sweep) - sin(heading));
            e += sign * radius * (cos(heading) - cos(heading + sweep));
            heading += sweep;
            total_length += radius * fabs(sweep);
            turns += fabs(sweep);
        }
    }
    assert(hypot(n - goal->position.north_m, e - goal->position.east_m) < tolerance);
    assert(angle_error(heading, goal->course_rad) < 0.00002);
    assert(fabs(total_length - path->length_m) < tolerance);
    assert(fabs(turns - path->absolute_turn_rad) < 0.00001);
}

static void basic_cases(void)
{
    DcsDubinsPose start = {{0.0f, 0.0f}, 0.0f};
    DcsDubinsPose goal = {{100.0f, 0.0f}, 0.0f};
    DcsDubinsPath path;
    for (int family = 0; family < 4; ++family) {
        assert(dcs_dubins_generate(&start, &goal, 10.0f,
                                  (DcsDubinsFamily) family, &path));
        check_path(&start, &goal, 10.0f, (DcsDubinsFamily) family, &path);
        assert(fabsf(path.length_m - 100.0f) < 0.001f);
        assert(path.absolute_turn_rad < 0.00001f);
    }
    /* A quarter-turn has a known shortest length, independent of the solver. */
    goal = (DcsDubinsPose){{10.0f, 10.0f}, (float) (PI / 2.0)};
    assert(dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_RSR, &path));
    check_path(&start, &goal, 10.0f, DCS_DUBINS_RSR, &path);
    assert(fabs(path.length_m - 5.0 * PI) < 0.002);
    goal.position.east_m = -10.0f;
    goal.course_rad = (float) (-PI / 2.0);
    assert(dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_LSL, &path));
    check_path(&start, &goal, 10.0f, DCS_DUBINS_LSL, &path);
    assert(fabs(path.length_m - 5.0 * PI) < 0.002);

    /* CCC limiting case: same headings, centres four radii apart. */
    goal = (DcsDubinsPose){{40.0f, 0.0f}, 0.0f};
    for (int family = DCS_DUBINS_RLR; family <= DCS_DUBINS_LRL; ++family) {
        assert(dcs_dubins_generate(&start, &goal, 10.0f,
                                  (DcsDubinsFamily) family, &path));
        check_path(&start, &goal, 10.0f, (DcsDubinsFamily) family, &path);
        assert(fabs(path.length_m - 20.0 * PI) < 0.002);
    }
    goal.position.north_m = 41.0f;
    assert(!dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_RLR, &path));
    assert(!dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_LRL, &path));

    /* Opposite-turn circles overlap: no forward inner tangent. */
    goal = (DcsDubinsPose){{0.0f, -10.0f}, 0.0f};
    assert(!dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_LSR, &path));
    goal.position.east_m = -40.0f; /* Touching inner-tangent boundary. */
    assert(dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_LSR, &path));
    check_path(&start, &goal, 10.0f, DCS_DUBINS_LSR, &path);

    /* Same position/course should allow a zero-length same-turn connection. */
    start = (DcsDubinsPose){{15.0f, -8.0f}, 0.7f};
    goal = start;
    assert(dcs_dubins_generate(&start, &goal, 10.0f, DCS_DUBINS_LSL, &path));
    check_path(&start, &goal, 10.0f, DCS_DUBINS_LSL, &path);
    assert(path.length_m == 0.0f);
}

static void closure_and_symmetry(void)
{
    unsigned int feasible[6] = {0};
    for (int i = 0; i < 480; ++i) {
        DcsDubinsPose start = {
            {17.0f, -23.0f}, (float) (i % 17 - 8) * 0.39f
        };
        DcsDubinsPose goal = {
            {(float) (i % 19 - 9) * 7.0f,
             (float) ((i * 7) % 23 - 11) * 5.0f},
            (float) ((i * 11) % 29 - 14) * 0.31f
        };
        const float radius = 5.0f + (float) (i % 7) * 4.0f;
        for (int f = 0; f < DCS_DUBINS_FAMILY_COUNT; ++f) {
            DcsDubinsPath path, mirror;
            DcsDubinsPose reflected_start = start, reflected_goal = goal;
            reflected_start.position.east_m *= -1.0f;
            reflected_goal.position.east_m *= -1.0f;
            reflected_start.course_rad *= -1.0f;
            reflected_goal.course_rad *= -1.0f;
            const DcsDubinsFamily family = (DcsDubinsFamily) f;
            const DcsDubinsFamily reflected_family = (DcsDubinsFamily) (f ^ 1);
            const bool ok = dcs_dubins_generate(&start, &goal, radius, family, &path);
            assert(ok == dcs_dubins_generate(&reflected_start, &reflected_goal,
                                             radius, reflected_family, &mirror));
            if (ok) {
                ++feasible[f];
                check_path(&start, &goal, radius, family, &path);
                check_path(&reflected_start, &reflected_goal, radius,
                           reflected_family, &mirror);
                assert(fabsf(path.length_m - mirror.length_m) < 0.005f);
            }
        }
    }
    for (int f = 0; f < 6; ++f) {
        assert(feasible[f] > 25u);
    }
}

static void invalid_inputs_preserve_output(void)
{
    const DcsDubinsPose pose = {{0.0f, 0.0f}, 0.0f};
    DcsDubinsPath output = {0}, saved;
    output.length_m = 123.0f;
    saved = output;
    assert(!dcs_dubins_generate(NULL, &pose, 1.0f, DCS_DUBINS_LSL, &output));
    assert(!dcs_dubins_generate(&pose, NULL, 1.0f, DCS_DUBINS_LSL, &output));
    assert(!dcs_dubins_generate(&pose, &pose, 1.0f, DCS_DUBINS_LSL, NULL));
    const float invalid_radii[] = {0.0f, -1.0f, NAN, INFINITY};
    for (unsigned int i = 0; i < sizeof(invalid_radii) / sizeof(invalid_radii[0]); ++i) {
        assert(!dcs_dubins_generate(&pose, &pose, invalid_radii[i],
                                   DCS_DUBINS_LSL, &output));
    }
    assert(!dcs_dubins_generate(&pose, &pose, 1.0f, (DcsDubinsFamily) -1, &output));
    assert(!dcs_dubins_generate(&pose, &pose, 1.0f, DCS_DUBINS_FAMILY_COUNT, &output));
    DcsDubinsPose invalid = pose;
    invalid.course_rad = NAN;
    assert(!dcs_dubins_generate(&invalid, &pose, 1.0f, DCS_DUBINS_LSL, &output));
    invalid = pose;
    invalid.position.north_m = INFINITY;
    assert(!dcs_dubins_generate(&pose, &invalid, 1.0f, DCS_DUBINS_LSL, &output));
    invalid = pose;
    invalid.position.north_m = FLT_MAX;
    assert(!dcs_dubins_generate(&pose, &invalid, FLT_MIN, DCS_DUBINS_LSL, &output));
    assert(memcmp(&output, &saved, sizeof(output)) == 0);
}

int main(void)
{
    basic_cases();
    closure_and_symmetry();
    invalid_inputs_preserve_output();
    puts("Dubins geometry tests passed.");
    return 0;
}
