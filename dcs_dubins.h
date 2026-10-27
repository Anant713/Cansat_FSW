#ifndef DCS_DUBINS_H
#define DCS_DUBINS_H

#include "dcs_planner.h"

/* Order also supplies the agreed final tie-break order for future selection. */
typedef enum {
    DCS_DUBINS_LSL,
    DCS_DUBINS_RSR,
    DCS_DUBINS_LSR,
    DCS_DUBINS_RSL,
    DCS_DUBINS_RLR,
    DCS_DUBINS_LRL,
    DCS_DUBINS_FAMILY_COUNT
} DcsDubinsFamily;

typedef struct {
    DcsPathPoint position;
    float course_rad;
} DcsDubinsPose;

typedef struct {
    /* Three slots, including zero-length segments in degenerate cases. */
    DcsPathSegment segments[3];
    float length_m;
    float absolute_turn_rad;
} DcsDubinsPath;

/*
 * Construct ONE selected family; the future planner evaluates all six.
 * Geometry only: no wind, height ranking, final line, loiter or actuation.
 * L/R mean physical left/right in North/East; positive arc sweep is right.
 * Returns false for invalid input, unrepresentable geometry or a family with
 * no geometric solution. Output is unchanged on failure. No allocation or
 * iteration dependent on path length; fixed three-segment construction.
 */
bool dcs_dubins_generate(const DcsDubinsPose *start,
                        const DcsDubinsPose *goal,
                        float radius_m, DcsDubinsFamily family,
                        DcsDubinsPath *output);

#endif
