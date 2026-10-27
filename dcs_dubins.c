/*
 * Normalized six-family equations adapted from Andrew Walker's Dubins-Curves:
 * https://github.com/AndrewWalker/Dubins-Curves/blob/master/src/dubins.c
 * Copyright (c) 2008-2018, Andrew Walker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "dcs_dubins.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

/* -1 left, 0 straight, +1 right in North/East coordinates. */
static const int directions[DCS_DUBINS_FAMILY_COUNT][3] = {
    {-1, 0, -1}, {1, 0, 1}, {-1, 0, 1},
    {1, 0, -1}, {1, -1, 1}, {-1, 1, -1}
};

static float wrap_angle(float angle)
{
    float wrapped = fmodf(angle, TWO_PI_F);
    if (wrapped < 0.0f) {
        wrapped += TWO_PI_F;
    }
    return wrapped;
}

/* Snap only float-roundoff-sized boundary sweeps, not a flight tolerance. */
static float wrap_sweep(float angle)
{
    float wrapped = wrap_angle(angle);
    const float epsilon = 8.0f * FLT_EPSILON;
    return (wrapped < epsilon || TWO_PI_F - wrapped < epsilon) ? 0.0f : wrapped;
}

static bool valid_pose(const DcsDubinsPose *pose)
{
    return pose != NULL && isfinite(pose->position.north_m) &&
           isfinite(pose->position.east_m) && isfinite(pose->course_rad);
}

static bool parameters(float a, float b, float d, DcsDubinsFamily family,
                       float travel[3])
{
    const float sa = sinf(a), sb = sinf(b);
    const float ca = cosf(a), cb = cosf(b);
    const float cab = cosf(a - b);
    const float d2 = d * d;
    float middle, tangent;
    if (!isfinite(d2)) {
        return false;
    }
    if (family == DCS_DUBINS_LSL || family == DCS_DUBINS_RSR) {
        const bool left = family == DCS_DUBINS_LSL;
        const float x = left ? d + sa - sb : d - sa + sb;
        const float y = left ? cb - ca : ca - cb;
        middle = hypotf(x, y);
        tangent = atan2f(y, x);
        /* Coincident equal-turn circles: connect directly along that circle. */
        if (middle <= 8.0f * FLT_EPSILON * (d + 2.0f)) {
            travel[0] = wrap_sweep(left ? b - a : a - b);
            travel[1] = 0.0f;
            travel[2] = 0.0f;
            return true;
        }
        travel[0] = wrap_sweep(left ? tangent - a : a - tangent);
        travel[1] = middle;
        travel[2] = wrap_sweep(left ? b - tangent : tangent - b);
    } else if (family == DCS_DUBINS_LSR || family == DCS_DUBINS_RSL) {
        const bool left_first = family == DCS_DUBINS_LSR;
        float squared = d2 - 2.0f + 2.0f * cab +
                        (left_first ? 2.0f : -2.0f) * d * (sa + sb);
        /* Roundoff guard at touching-circle boundaries, not a physical margin. */
        const float epsilon = 8.0f * FLT_EPSILON * (d2 + 8.0f);
        if (squared < -epsilon) {
            return false;
        }
        middle = sqrtf(fmaxf(0.0f, squared));
        if (left_first) {
            tangent = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0f, middle);
            travel[0] = wrap_sweep(tangent - a);
            travel[2] = wrap_sweep(tangent - b);
        } else {
            tangent = atan2f(ca + cb, d - sa - sb) - atan2f(2.0f, middle);
            travel[0] = wrap_sweep(a - tangent);
            travel[2] = wrap_sweep(b - tangent);
        }
        travel[1] = middle;
    } else {
        const bool right_first = family == DCS_DUBINS_RLR;
        float cosine = (6.0f - d2 + 2.0f * cab +
                        (right_first ? 2.0f : -2.0f) * d * (sa - sb)) / 8.0f;
        const float epsilon = 8.0f * FLT_EPSILON;
        if (cosine < -1.0f - epsilon || cosine > 1.0f + epsilon) {
            return false;
        }
        cosine = fmaxf(-1.0f, fminf(1.0f, cosine));
        middle = wrap_angle(TWO_PI_F - acosf(cosine));
        if (right_first) {
            tangent = atan2f(ca - cb, d - sa + sb);
            travel[0] = wrap_sweep(a - tangent + 0.5f * middle);
            travel[2] = wrap_sweep(a - b - travel[0] + middle);
        } else {
            tangent = atan2f(ca - cb, d + sa - sb);
            travel[0] = wrap_sweep(-a - tangent + 0.5f * middle);
            travel[2] = wrap_sweep(b - a - travel[0] + middle);
        }
        travel[1] = middle;
    }
    return isfinite(travel[0]) && isfinite(travel[1]) && isfinite(travel[2]);
}

bool dcs_dubins_generate(const DcsDubinsPose *start,
                        const DcsDubinsPose *goal,
                        float radius_m, DcsDubinsFamily family,
                        DcsDubinsPath *output)
{
    if (!valid_pose(start) || !valid_pose(goal) || output == NULL ||
        !isfinite(radius_m) || radius_m <= 0.0f ||
        (unsigned int) family >= DCS_DUBINS_FAMILY_COUNT) {
        return false;
    }

    const float dn = goal->position.north_m - start->position.north_m;
    const float de = goal->position.east_m - start->position.east_m;
    const float d = hypotf(dn, de) / radius_m;
    if (!isfinite(d)) {
        return false;
    }
    /* Reflect East: standard mathematical L-positive equations use x=N,y=-E. */
    const float bearing = d > 0.0f ? atan2f(-de, dn) : 0.0f;
    const float a = wrap_angle(-wrap_angle(start->course_rad) - bearing);
    const float b = wrap_angle(-wrap_angle(goal->course_rad) - bearing);
    float travel[3];
    if (!parameters(a, b, d, family, travel)) {
        return false;
    }

    DcsDubinsPath candidate = {0};
    DcsPathPoint position = start->position;
    float course = wrap_angle(start->course_rad);
    for (unsigned int i = 0; i < 3u; ++i) {
        DcsPathSegment *segment = &candidate.segments[i];
        const int direction = directions[family][i];
        const float length = radius_m * travel[i];
        candidate.length_m += length;
        if (direction == 0) {
            segment->type = DCS_PATH_STRAIGHT;
            segment->geometry.straight.start = position;
            position.north_m += length * cosf(course);
            position.east_m += length * sinf(course);
            segment->geometry.straight.end = position;
        } else {
            const float sign = (float) direction;
            const float angle = course - sign * 0.5f * PI_F;
            DcsPathPoint centre = {
                position.north_m - sign * radius_m * sinf(course),
                position.east_m + sign * radius_m * cosf(course)
            };
            segment->type = DCS_PATH_ARC;
            segment->geometry.arc.centre = centre;
            segment->geometry.arc.radius_m = radius_m;
            segment->geometry.arc.start_angle_rad = wrap_angle(angle);
            segment->geometry.arc.sweep_rad = sign * travel[i];
            candidate.absolute_turn_rad += travel[i];
            const float end_angle = angle + sign * travel[i];
            position.north_m = centre.north_m + radius_m * cosf(end_angle);
            position.east_m = centre.east_m + radius_m * sinf(end_angle);
            course = wrap_angle(course + sign * travel[i]);
            if (!isfinite(centre.north_m) || !isfinite(centre.east_m)) {
                return false;
            }
        }
        if (!isfinite(position.north_m) || !isfinite(position.east_m)) {
            return false;
        }
    }
    if (!isfinite(candidate.length_m)) {
        return false;
    }
    *output = candidate;
    return true;
}
