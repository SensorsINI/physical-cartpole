#include "controller_api_helper.h"
#include <string.h>   /* strcmp */

static float* find_ptr_for_token(const char* token,
                                 const SignalEntry* table, uint8_t table_len,
                                 float* fallback_zero)
{
    /* Linear scan is fine for ~10 signals; no work in the hot path anyway. */
    for (uint8_t i = 0; i < table_len; ++i) {
        if (table[i].name && !strcmp(table[i].name, token)) {
            return table[i].ptr ? table[i].ptr : fallback_zero;
        }
    }
    return fallback_zero;
}

void CB_Init(ControllerBinding* cb)
{
    cb->ops_last  = 0;
    cb->n_inputs  = 0;
    cb->zero      = 0.0f;
    for (uint8_t i = 0; i < CB_MAX_INPUTS; ++i) cb->in_ptrs[i] = &cb->zero;
}

void CB_Reset(ControllerBinding* cb)
{
    if (cb->ops_last && cb->ops_last->release) cb->ops_last->release();
    CB_Init(cb);
}

void CB_RebindOnChange(ControllerBinding* cb,
                       const ControllerOps* desired_ops,
                       const SignalEntry* table, uint8_t table_len)
{
    if (desired_ops == cb->ops_last && cb->ops_last) return;

    /* Tidy up previous controller, if any. */
    if (cb->ops_last && cb->ops_last->release) cb->ops_last->release();

    /* Switch to the new ops and run its init. */
    cb->ops_last = desired_ops;
    if (cb->ops_last && cb->ops_last->init) cb->ops_last->init();

    /* Bind token→pointer once, based on the controller's declared spec. */
    const ControllerSpec* S = cb->ops_last->spec();
    cb->n_inputs = S->n_inputs;

    const uint8_t n = (cb->n_inputs < CB_MAX_INPUTS) ? cb->n_inputs : CB_MAX_INPUTS;
    for (uint8_t i = 0; i < n; ++i) {
        cb->in_ptrs[i] = find_ptr_for_token(S->names[i], table, table_len, &cb->zero);
    }
    for (uint8_t i = n; i < CB_MAX_INPUTS; ++i) cb->in_ptrs[i] = &cb->zero;
}

float CB_Eval(ControllerBinding* cb)
{
    /* Non-obvious bit: we dereference pointers now, so every input reflects the latest
       sensor/state values without any copies at binding time. */
    float in [CB_MAX_INPUTS];
    float out[CB_MAX_OUTPUTS];

    const uint8_t n = cb->n_inputs < CB_MAX_INPUTS ? cb->n_inputs : CB_MAX_INPUTS;
    for (uint8_t i = 0; i < n; ++i) in[i] = *cb->in_ptrs[i];

    cb->ops_last->evaluate(in, out);
    return out[0];  /* all your current controllers are single-output (Q) */
}
