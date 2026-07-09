#include "secloc.h"

#define SECLOC_PI_F 3.14159265358979323846f

void secloc_reset(SeclocState* state)
{
    state->ang_last_shift = 0.0001f;
    state->pos_last_shift = 0.0001f;
    state->last_Q = 0.0f;
    state->has_init = 1;
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

int secloc_should_sample(
    SeclocState* state,
    const SeclocConfig* config,
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te
)
{
    (void)pd;
    (void)ad;

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

    return (ang_spike || pos_spike) ? 1 : 0;
}
