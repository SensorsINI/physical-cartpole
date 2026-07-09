/*
 * Generic SecLoc wrapper around an on-chip inner controller.
 *
 *   inputs = position, positionD, angle, angleD, target_position, target_equilibrium, time
 *   On gate spike: run inner controller (LQR or PID, see secloc_inner_controller).
 *   Otherwise: hold previous Q.
 */

#include "secloc_controller.h"
#include "secloc.h"
#include "lqr.h"
#include "hardware_pid.h"

static const char* const SECLOC_InputNames[] = {
    "position", "positionD", "angle", "angleD", "target_position", "target_equilibrium", "time"
};

static const ControllerSpec SECLOC_Spec = {
    .version   = 1,
    .n_inputs  = 7,
    .n_outputs = 1,
    .names     = SECLOC_InputNames
};

static const SeclocConfig secloc_config = {
    .log_base       = 1.05f,
    .ang_dead_band  = 0.0f,
    .pos_dead_band  = 0.0f,
};

static SeclocState secloc_state;

SeclocInnerController secloc_inner_controller = SECLOC_INNER_LQR;

void secloc_set_inner_controller(SeclocInnerController inner)
{
    secloc_inner_controller = inner;
}

SeclocInnerController secloc_get_inner_controller(void)
{
    return secloc_inner_controller;
}

static void secloc_inner_init(void)
{
    switch (secloc_inner_controller) {
    case SECLOC_INNER_LQR:
        if (LQR_Ops.init) {
            LQR_Ops.init();
        }
        break;
    case SECLOC_INNER_PID:
        if (PID_Ops.init) {
            PID_Ops.init();
        }
        break;
    default:
        break;
    }
}

static float secloc_inner_evaluate(
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float time
)
{
    switch (secloc_inner_controller) {
    case SECLOC_INNER_LQR:
        return lqr_evaluate(p, pd, a, ad, tp, &LQR_DefaultGains);
    case SECLOC_INNER_PID:
        return pid_step(a, ad, p, pd, tp, time);
    default:
        return secloc_state.last_Q;
    }
}

static void SECLOC_Init(void)
{
    secloc_reset(&secloc_state);
    secloc_inner_init();
}

static void SECLOC_Release(void) { }

static void SECLOC_Evaluate(const float* in, float* out)
{
    const float p  = in[0];
    const float pd = in[1];
    const float a  = in[2];
    const float ad = in[3];
    const float tp = in[4];
    const float te = in[5];
    const float time = in[6];

    if (secloc_should_sample(&secloc_state, &secloc_config, p, pd, a, ad, tp, te)) {
        secloc_state.last_Q = secloc_inner_evaluate(p, pd, a, ad, tp, time);
    }

    out[0] = secloc_state.last_Q;
}

static const ControllerSpec* SECLOC_GetSpec(void) { return &SECLOC_Spec; }

const ControllerOps SECLOC_Ops = {
    .spec     = SECLOC_GetSpec,
    .init     = SECLOC_Init,
    .evaluate = SECLOC_Evaluate,
    .release  = SECLOC_Release
};
