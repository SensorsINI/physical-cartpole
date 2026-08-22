#include "secloc_logic.h"

#define SECLOC_PI_F 3.14159265358979323846f

void secloc_reset(SeclocState* state)
{
    state->ang_last_shift = 0.0001f;
    state->pos_last_shift = 0.0001f;
    state->last_Q = 0.0f;
    state->has_init = 1;
    state->time_last = 0.0f;
    state->tick_last = -1;
}

float secloc_angle_shift_from_target(float a, float te)
{
    float shift = a;
    if (shift < 0.0f) {
        shift = -shift;
    }
    if (te < 0.0f) {
        /* Target down: distance from hanging (angle wraps at +/-pi). */
        shift = SECLOC_PI_F - shift;
    }
    return shift;
}

int32_t secloc_tick_from_time(float time, float time_quantum_s)
{
    return (int32_t)(time / time_quantum_s + 0.5f);
}

int secloc_gate_evaluated_tick(
    const SeclocState* state,
    const SeclocConfig* config,
    int32_t tick
)
{
    if (config->ref_period_ticks <= 0) {
        return 1;
    }

    /* tick < 0 means the caller has no tick source (time_quantum_s <= 0 in
     * the time-based API); the throttle is bypassed then. */
    if (tick < 0) {
        return 1;
    }

    if (state->tick_last < 0) {
        return 1;
    }

    return (tick - state->tick_last >= config->ref_period_ticks) ? 1 : 0;
}

int secloc_gate_evaluated(
    const SeclocState* state,
    const SeclocConfig* config,
    float time
)
{
    int32_t tick = -1;

    /* ref_period_ticks > 0 requires time_quantum_s > 0 (the Python gate raises
     * in this case); callers wire the quantum from the control loop period. */
    if (config->time_quantum_s > 0.0f) {
        tick = secloc_tick_from_time(time, config->time_quantum_s);
    }

    return secloc_gate_evaluated_tick(state, config, tick);
}

int secloc_should_sample_tick(
    SeclocState* state,
    const SeclocConfig* config,
    float p,
    float a,
    float tp,
    float te,
    int32_t tick
)
{
    if (!secloc_gate_evaluated_tick(state, config, tick)) {
        return 0;
    }

    float ang_shift = secloc_angle_shift_from_target(a, te);
    float pos_shift = p - tp;

    if (pos_shift < 0.0f) {
        pos_shift = -pos_shift;
    }

    if (state->has_init == 0) {
        state->has_init = 1;
        state->ang_last_shift = 0.0001f;
        state->pos_last_shift = 0.0001f;
    }

    /* Independent per-axis checks; either can fire, and on update only the
     * reference of the axis (or axes) that fired is refreshed. */
    int ang_spike = 0;
    if ((ang_shift > config->ang_dead_band) && (state->ang_last_shift != 0.0f)) {
        float ang_ratio_inc = ang_shift / state->ang_last_shift;
        float ang_ratio_dec = 1.0f / ang_ratio_inc;
        if ((ang_ratio_inc >= config->log_base) || (ang_ratio_dec >= config->log_base)) {
            ang_spike = 1;
        }
    }

    int pos_spike = 0;
    if ((pos_shift > config->pos_dead_band) && (state->pos_last_shift != 0.0f)) {
        float pos_ratio_inc = pos_shift / state->pos_last_shift;
        float pos_ratio_dec = 1.0f / pos_ratio_inc;
        if ((pos_ratio_inc >= config->log_base) || (pos_ratio_dec >= config->log_base)) {
            pos_spike = 1;
        }
    }

    if (ang_spike) {
        state->ang_last_shift = ang_shift;
    }
    if (pos_spike) {
        state->pos_last_shift = pos_shift;
    }

    if (ang_spike || pos_spike) {
        if (tick >= 0) {
            state->tick_last = tick;
        }
        return 1;
    }

    return 0;
}

int secloc_should_sample(
    SeclocState* state,
    const SeclocConfig* config,
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te,
    float time
)
{
    (void)pd;
    (void)ad;

    int32_t tick = -1;
    if (config->time_quantum_s > 0.0f) {
        tick = secloc_tick_from_time(time, config->time_quantum_s);
    }

    int fired = secloc_should_sample_tick(state, config, p, a, tp, te, tick);
    if (fired) {
        state->time_last = time;
    }
    return fired;
}
