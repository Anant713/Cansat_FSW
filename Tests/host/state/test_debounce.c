#define main legacy_state_tests
#include "test_state.c"
#undef main

static void recover(Cansat *c, CansatMissionState state)
{
    memset(flash, 0xff, sizeof(flash));
    writesLeft = -1;
    uint32_t ignored;
    (void)MissionStore_Load(&ignored);
    assert(MissionStore_Save((uint32_t)state));
    *c = (Cansat){0}; tick = 0; reset_outputs();
    assert(StateMachine_Init(c));
}
int main(void)
{
    assert(STATE_HEIGHT_CONFIRM_SAMPLES > 1);
    Cansat boundary;
    recover(&boundary, CANSAT_STATE_ASCENT);
    sample(&boundary, ++tick, STATE_DESCENT_MIN_PEAK_M);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES + 1; ++i)
        sample(&boundary, ++tick, STATE_DESCENT_MIN_PEAK_M - STATE_DESCENT_DROP_M - 1);
    assert(boundary.missionState == CANSAT_STATE_ASCENT);
    assert(StateMachine_GetStatus()->heightConfirmCount == 0);
    sample(&boundary, ++tick, STATE_DESCENT_MIN_PEAK_M + 1);
    for (unsigned i = 0; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i)
        sample(&boundary, ++tick, STATE_DESCENT_MIN_PEAK_M - STATE_DESCENT_DROP_M);
    assert(boundary.missionState == CANSAT_STATE_DESCENT_1);
    for (unsigned state = 0; state < CANSAT_STATE_TOUCHDOWN; ++state) {
        Cansat c;
        recover(&c, (CansatMissionState)state);
        float pass = state == 0 ? 31 : state == 1 ? 779 : state == 2 ? 599 : 19;
        float fail = state == 0 ? 30 : state == 1 ? 780 : state == 2 ? 600 : 20;
        if (state == 1) sample(&c, ++tick, 800); /* Seed peak. */
        sample(&c, ++tick, pass);
        assert(StateMachine_GetStatus()->heightConfirmCount == 1);
        for (unsigned i = 0; i < 20; ++i) StateMachine_Update(&c);
        assert(StateMachine_GetStatus()->heightConfirmCount == 1);
        sample(&c, ++tick, fail);
        assert(StateMachine_GetStatus()->heightConfirmCount == 0);
        sample(&c, ++tick, pass);
        c.bmp.heightValid = false; StateMachine_Update(&c);
        assert(StateMachine_GetStatus()->heightConfirmCount == 0);
        sample(&c, ++tick, pass);
        tick += STATE_ALTITUDE_MAX_AGE_MS + 1;
        StateMachine_Update(&c);
        assert(StateMachine_GetStatus()->heightConfirmCount == 0);
        sample(&c, ++tick, pass);
        tick += STATE_ALTITUDE_MAX_AGE_MS + 1;
        sample(&c, tick, pass); /* Gap resets even without intermediate stale check. */
        assert(StateMachine_GetStatus()->heightConfirmCount == 1);
        for (unsigned i = 1; i < STATE_HEIGHT_CONFIRM_SAMPLES; ++i) {
            assert(c.missionState == (CansatMissionState)state);
            sample(&c, ++tick, pass);
        }
        assert(c.missionState == (CansatMissionState)(state + 1));
        assert(StateMachine_GetStatus()->heightConfirmCount == 0);
        StateMachine_Update(&c); /* Transition sample cannot count in the next state. */
        assert(StateMachine_GetStatus()->heightConfirmCount == 0);
    }
    Cansat c; recover(&c, CANSAT_STATE_ASCENT);
    sample(&c, UINT32_MAX - 1, 800);
    sample(&c, UINT32_MAX, 770);
    sample(&c, 0, 770);
    assert(StateMachine_GetStatus()->heightConfirmCount == 2);
    sample(&c, 1, 810); /* New peak cancels a pending descent. */
    assert(StateMachine_GetStatus()->heightConfirmCount == 0);
    assert(StateMachine_GetStatus()->maxHeightM == 810);
    assert(StateMachine_Init(&c));
    assert(StateMachine_GetStatus()->heightConfirmCount == 0);
    puts("Debounce tests passed: all gates, duplicate samples, counter resets, gap, rollover and new peak.");
    return 0;
}

