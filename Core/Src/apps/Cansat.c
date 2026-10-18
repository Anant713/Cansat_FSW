#include "apps/Cansat.h"
#include <stddef.h>

void Cansat_Init(Cansat *cansat)
{
    if (cansat != NULL) {
        *cansat = (Cansat){0};
        const char teamId[] = CANSAT_DEFAULT_TEAM_ID;
        _Static_assert(sizeof(teamId) <= sizeof(cansat->telemetry.teamId), "Team ID too long");
        for (size_t i = 0; i < sizeof(teamId); ++i) cansat->telemetry.teamId[i] = teamId[i];
    }
}
