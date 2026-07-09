/*
 * Generic SecLoc wrapper around an on-chip inner controller.
 *
 *   inputs = position, positionD, angle, angleD, target_position, target_equilibrium, time
 *   On gate spike: run inner controller (LQR, PID or the pure-C neural imitator,
 *   see secloc_inner_controller).
 *   Otherwise: hold previous Q.
 */

#include <math.h>

#include "secloc_controller.h"
#include "secloc.h"
#include "secloc_defaults.h"
#include "lqr.h"
#include "hardware_pid.h"
#include "neural_controller_C.h"

static const char* const SECLOC_InputNames[] = {
    "position", "positionD", "angle", "angleD", "target_position", "target_equilibrium", "time"
};

static const ControllerSpec SECLOC_Spec = {
    .version   = 1,
    .n_inputs  = 7,
    .n_outputs = 1,
    .names     = SECLOC_InputNames
};

static SeclocConfig secloc_config = {
    .log_base         = SECLOC_DEFAULT_LOG_BASE,
    .ref_period_ticks = SECLOC_DEFAULT_REF_PERIOD_TICKS,
    .ang_dead_band    = SECLOC_DEFAULT_DEAD_ANG,
    .pos_dead_band    = SECLOC_DEFAULT_DEAD_POS,
    .time_quantum_s   = SECLOC_DEFAULT_TIME_QUANTUM_S,
};

static SeclocState secloc_state;
static uint8_t secloc_last_skipped_update = 0;
static uint8_t secloc_last_gate_skipped = 0;

void secloc_controller_set_config(
    float log_base,
    int32_t ref_period_ticks,
    float ang_dead_band,
    float pos_dead_band
)
{
    secloc_config.log_base         = log_base;
    secloc_config.ref_period_ticks = ref_period_ticks;
    secloc_config.ang_dead_band    = ang_dead_band;
    secloc_config.pos_dead_band    = pos_dead_band;
}

void secloc_controller_set_time_quantum(float time_quantum_s)
{
    secloc_config.time_quantum_s = time_quantum_s;
}

const SeclocState* secloc_controller_get_state(void)
{
    return &secloc_state;
}

uint8_t secloc_controller_telemetry_flags(void)
{
    return (uint8_t)(secloc_last_skipped_update | (secloc_last_gate_skipped << 1));
}

SeclocInnerController secloc_inner_controller = SECLOC_DEFAULT_INNER_CONTROLLER;

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
    case SECLOC_INNER_NNC:
        if (NNC_Ops.init) {
            NNC_Ops.init();
        }
        break;
    default:
        break;
    }
}

/* Neural imitator expects a different input vector than the SecLoc spec:
 *   ["angleD","angle_cos","angle_sin","position","positionD",
 *    "target_equilibrium","target_position"]
 * Build it here (cos/sin computed on the fly) and delegate to NNC_Ops. */
static float secloc_nnc_evaluate(
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te
)
{
    float nn_in[7];
    float nn_out[1];

    nn_in[0] = ad;
    nn_in[1] = cosf(a);
    nn_in[2] = sinf(a);
    nn_in[3] = p;
    nn_in[4] = pd;
    nn_in[5] = te;
    nn_in[6] = tp;

    NNC_Ops.evaluate(nn_in, nn_out);
    return nn_out[0];
}

static float secloc_inner_evaluate(
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te,
    float time
)
{
    switch (secloc_inner_controller) {
    case SECLOC_INNER_LQR:
        return lqr_evaluate(p, pd, a, ad, tp, &LQR_DefaultGains);
    case SECLOC_INNER_PID:
        return pid_step(a, ad, p, pd, tp, time);
    case SECLOC_INNER_NNC:
        return secloc_nnc_evaluate(p, pd, a, ad, tp, te);
    default:
        return secloc_state.last_Q;
    }
}

static void SECLOC_Init(void)
{
    secloc_reset(&secloc_state);
    secloc_last_skipped_update = 0;
    secloc_last_gate_skipped = 0;
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

    /* Matches the Python CSV semantics: gate_skipped is only set when the gate
     * was actually consulted (ref_period elapsed) and declined; rows where the
     * throttle blocked the call hold Q but are not gate decisions. */
    int gate_evaluated = secloc_gate_evaluated(&secloc_state, &secloc_config, time);

    if (secloc_should_sample(&secloc_state, &secloc_config, p, pd, a, ad, tp, te, time)) {
        secloc_state.last_Q = secloc_inner_evaluate(p, pd, a, ad, tp, te, time);
        secloc_last_skipped_update = 0;
        secloc_last_gate_skipped = 0;
    } else {
        secloc_last_skipped_update = 1;
        secloc_last_gate_skipped = gate_evaluated ? 1 : 0;
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
